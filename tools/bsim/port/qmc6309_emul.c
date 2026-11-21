/*
 * QMC6309 I2C emulator for nrf52_bsim (tools/bsim): a timing- and error-faithful chip model.
 *
 * Datasheet (QMC6309 Rev C / QMC6309H Rev E) behaviour:
 * - registers: 0x00 chip ID 0x90, 0x01..0x06 XYZ int16 LE, 0x09 status (DRDY bit0, OVFL bit1),
 *   0x0A CTRL1 {OSR2:3, OSR1:2, -, MODE:2}, 0x0B CTRL2 {SOFT_RST, ODR:3, RNG:2, SET/RESET:2}
 * - modes: suspend (no measurement), normal (ODR from the chip's own clock), single (one
 *   measurement, then suspend), continuous (back to back); soft reset restores defaults
 * - data registers hold the last sample; they are updated one axis after another, so a read
 *   during an update can mix two measurements (QMC6309H 5.7); DRDY clears on a status read;
 *   OVFL when |code| > 32000; output saturates at int16
 * - range 32/16/8 G = 1000/2000/4000 LSB/G; noise (OSR1 8) 2.5/3.5/5.0/7.0 mG for OSR2 8/4/2/1
 * - sensitivity +-5 %, tempco +-0.05 %/C, orthogonality 90+-1 deg, null-field offset +-1 G
 * - set/reset off or set-only: offset is not renewed (shifts after strong fields persist)
 *
 * Not in the datasheet (assumptions, override with env SLIME_SIM_<name>):
 *   MAG_CLK_ERR   chip clock error, fraction (default 0.02: 50 Hz runs at 51 Hz)
 *   MAG_CONV_US   measurement window at OSR1 8 (default 2800, from the 500 uA normal vs
 *                 3500 uA continuous current at 50 Hz); scales with OSR1
 *   MAG_AXIS_US   delay between X, Y and Z register updates (default 50)
 *   MAG_SHIFT_G   persistent offset shift after a > 4 G field without set/reset (default 0.02)
 *   MAG_IDEAL=1   no chip errors (noise, sensitivity, orthogonality, offset)
 *   MAG_SEED      per-chip error seed (default 1)
 * The I2C bus costs 22.5 us per byte (400 kHz). On the Mumo the chip sits behind the ICM
 * aux I2C master; here it is on its own bus (the firmware's direct-I2C path).
 */
#define DT_DRV_COMPAT slimevr_qmc6309_emul

#include <math.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/kernel.h>

#include "slime_sim.h"

#define REG_CTRL1 0x0A
#define REG_CTRL2 0x0B
#define MODE_SUSPEND 0
#define MODE_NORMAL 1
#define MODE_SINGLE 2
#define MODE_CONT 3

struct axis_update {
	int64_t at_us; /* register becomes visible */
	int16_t value;
};

struct qmc6309_emul_data {
	uint8_t regs[0x40];
	/* chip errors */
	float sens[3], ortho[3][3], offset[3], tempco[3], shift[3];
	bool ideal;
	uint32_t rng;
	/* measurement engine */
	int64_t next_us;    /* end of the next measurement, 0 = none scheduled */
	int64_t last_us;    /* time up to which measurements were produced */
	struct axis_update pending[3];
	bool drdy, ovfl;
};

static float param(const char *name, double def)
{
	return (float)slime_sim_param(name, def);
}

static float urand(struct qmc6309_emul_data *d)
{
	d->rng ^= d->rng << 13;
	d->rng ^= d->rng >> 17;
	d->rng ^= d->rng << 5;
	return (float)(d->rng >> 8) / 16777216.0f;
}

static float nrand(struct qmc6309_emul_data *d)
{
	float u = fmaxf(urand(d), 1e-7f), v = urand(d);
	return sqrtf(-2.0f * logf(u)) * cosf(6.2831853f * v);
}

static int64_t now_us(void)
{
	return (int64_t)k_ticks_to_us_floor64(k_uptime_ticks());
}

static int mode(const struct qmc6309_emul_data *d)
{
	return d->regs[REG_CTRL1] & 0x3;
}

static float lsb_per_g(const struct qmc6309_emul_data *d)
{
	static const float lsb[4] = {1000, 2000, 4000, 4000};
	return lsb[(d->regs[REG_CTRL2] >> 2) & 0x3];
}

static int64_t period_us(const struct qmc6309_emul_data *d)
{
	static const float odr[8] = {1, 10, 50, 100, 200, 200, 200, 200};
	float hz = odr[(d->regs[REG_CTRL2] >> 4) & 0x7] * (1.0f + param("MAG_CLK_ERR", 0.02));
	return (int64_t)(1e6f / hz);
}

static int64_t conv_us(const struct qmc6309_emul_data *d)
{
	static const int osr1[4] = {8, 4, 2, 1};
	return (int64_t)(param("MAG_CONV_US", 2800) * osr1[(d->regs[REG_CTRL1] >> 3) & 0x3] / 8.0f);
}

static float noise_g(const struct qmc6309_emul_data *d)
{
	static const float osr2_mg[8] = {7.0f, 5.0f, 3.5f, 2.5f, 2.0f, 2.0f, 2.0f, 2.0f}; /* 1,2,4,8,16.. */
	static const float osr1[4] = {8, 4, 2, 1};
	return osr2_mg[(d->regs[REG_CTRL1] >> 5) & 0x7] * 1e-3f
		* sqrtf(8.0f / osr1[(d->regs[REG_CTRL1] >> 3) & 0x3]);
}

/* One measurement ending at t_end: field averaged over the conversion window. */
static void measure(struct qmc6309_emul_data *d, int64_t t_end)
{
	int64_t w = conv_us(d);
	float b[3] = {0}, m[3], temp = 25;
	int n = 0;
	for (int k = 0; k < 4; k++) {
		double t = (double)(t_end - w + (2 * k + 1) * w / 8) / 1e6;
		if (slime_sim_field(t, m, &temp) == 0) {
			for (int i = 0; i < 3; i++) {
				b[i] += m[i];
			}
			n++;
		}
	}
	if (!n) {
		return;
	}
	float lsb = lsb_per_g(d);
	uint8_t set_reset = d->regs[REG_CTRL2] & 0x3;
	float mag = 0;
	for (int i = 0; i < 3; i++) {
		b[i] /= n;
		mag += b[i] * b[i];
	}
	if (set_reset != 0 && sqrtf(mag) > 4.0f) {
		/* without set/reset the sensor's magnetic state is not restored */
		for (int i = 0; i < 3; i++) {
			d->shift[i] = copysignf(param("MAG_SHIFT_G", 0.02), b[i]);
		}
	} else if (set_reset == 0) {
		memset(d->shift, 0, sizeof(d->shift));
	}
	d->ovfl = false;
	for (int i = 0; i < 3; i++) {
		float v = b[i];
		if (!d->ideal) {
			v = 0;
			for (int j = 0; j < 3; j++) {
				v += d->ortho[i][j] * b[j];
			}
			v = v * d->sens[i] * (1.0f + d->tempco[i] * (temp - 25.0f)) + d->offset[i] + d->shift[i]
				+ noise_g(d) * nrand(d);
		}
		float code = roundf(v * lsb);
		d->ovfl |= fabsf(code) > 32000.0f;
		d->pending[i].value = (int16_t)CLAMP(code, -32768.0f, 32767.0f);
		d->pending[i].at_us = t_end + i * (int64_t)param("MAG_AXIS_US", 50);
	}
}

/* Run the measurement engine up to now; publish register updates that are due. */
static void advance(struct qmc6309_emul_data *d)
{
	int64_t now = now_us();
	for (;;) {
		for (int i = 0; i < 3; i++) {
			if (d->pending[i].at_us && d->pending[i].at_us <= now) {
				d->regs[1 + 2 * i] = (uint8_t)d->pending[i].value;
				d->regs[2 + 2 * i] = (uint8_t)((uint16_t)d->pending[i].value >> 8);
				d->pending[i].at_us = 0;
				if (i == 2) {
					d->drdy = true;
				}
			}
		}
		if (!d->next_us || d->next_us > now || d->pending[2].at_us) {
			return;
		}
		measure(d, d->next_us);
		switch (mode(d)) {
		case MODE_NORMAL:
			d->next_us += period_us(d);
			break;
		case MODE_CONT:
			d->next_us += conv_us(d);
			break;
		default: /* single: back to suspend */
			d->regs[REG_CTRL1] &= ~0x3;
			d->next_us = 0;
		}
	}
}

static void control_written(struct qmc6309_emul_data *d, uint8_t reg)
{
	if (reg == REG_CTRL2 && (d->regs[REG_CTRL2] & 0x80)) {
		d->regs[REG_CTRL1] = 0;
		d->regs[REG_CTRL2] = 0x80; /* SOFT_RST is not auto-cleared */
		d->next_us = 0;
		memset(d->pending, 0, sizeof(d->pending));
		d->drdy = d->ovfl = false;
		return;
	}
	if (reg != REG_CTRL1) {
		return;
	}
	int64_t now = now_us();
	switch (mode(d)) {
	case MODE_SUSPEND:
		d->next_us = 0;
		break;
	case MODE_NORMAL:
		d->next_us = now + period_us(d);
		break;
	default:
		d->next_us = now + conv_us(d);
	}
}

static uint8_t read_reg(struct qmc6309_emul_data *d, uint8_t reg)
{
	advance(d);
	if (reg == 0x00) {
		return 0x90;
	}
	if (reg == 0x09) {
		uint8_t st = (d->drdy ? 0x01 : 0) | (d->ovfl ? 0x02 : 0);
		d->drdy = false;
		return st;
	}
	return d->regs[reg % sizeof(d->regs)];
}

static int qmc6309_emul_transfer(const struct emul *target, struct i2c_msg *msgs, int num_msgs, int addr)
{
	ARG_UNUSED(addr);
	struct qmc6309_emul_data *d = target->data;
	uint8_t reg = 0;
	k_busy_wait(23); /* address byte at 400 kHz */
	for (int i = 0; i < num_msgs; i++) {
		struct i2c_msg *msg = &msgs[i];
		if (msg->flags & I2C_MSG_READ) {
			for (uint32_t j = 0; j < msg->len; j++) {
				k_busy_wait(23);
				msg->buf[j] = read_reg(d, reg++);
			}
		} else if (msg->len > 0) {
			reg = msg->buf[0];
			for (uint32_t j = 1; j < msg->len; j++) {
				k_busy_wait(23);
				advance(d);
				d->regs[reg % sizeof(d->regs)] = msg->buf[j];
				control_written(d, reg);
				reg++;
			}
		}
	}
	return 0;
}

static int qmc6309_emul_init(const struct emul *target, const struct device *parent)
{
	ARG_UNUSED(parent);
	struct qmc6309_emul_data *d = target->data;
	memset(d, 0, sizeof(*d));
	d->rng = 0x9E3779B9u * (uint32_t)(param("MAG_SEED", 1) + 1);
	d->ideal = param("MAG_IDEAL", 0) != 0;
	const float deg = 0.01745329f;
	for (int i = 0; i < 3; i++) {
		d->sens[i] = 1.0f + CLAMP(0.02f * nrand(d), -0.05f, 0.05f);
		d->tempco[i] = 0.0005f * (2.0f * urand(d) - 1.0f);
		d->offset[i] = CLAMP(0.3f * nrand(d), -1.0f, 1.0f);
		for (int j = 0; j < 3; j++) {
			d->ortho[i][j] = i == j ? 1.0f : CLAMP(0.5f * deg * nrand(d), -3 * deg, 3 * deg);
		}
	}
	return 0;
}

static const struct i2c_emul_api qmc6309_emul_api = {
	.transfer = qmc6309_emul_transfer,
};

#define QMC6309_EMUL(n)                                                                            \
	static struct qmc6309_emul_data qmc6309_emul_data_##n;                                     \
	DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL, 99, NULL);                 \
	EMUL_DT_INST_DEFINE(n, qmc6309_emul_init, &qmc6309_emul_data_##n, NULL,                    \
			    &qmc6309_emul_api, NULL)

DT_INST_FOREACH_STATUS_OKAY(QMC6309_EMUL)
