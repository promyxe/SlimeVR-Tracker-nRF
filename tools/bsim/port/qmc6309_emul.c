/*
 * QMC6309 I2C register emulator for nrf52_bsim (tools/bsim).
 * Chip ID 0x90 at 0x00, data at 0x01..0x06 (int16 LE, 4000 LSB/G for the 8 G
 * range the driver selects), status 0x09 bit0 DRDY when a newer mag sample of
 * the SLIME_SIM_CSV recording exists than the last one read. Writes are stored.
 * On the real Mumo board the QMC6309 sits behind the ICM aux I2C master; here it
 * is on its own I2C bus (the firmware's direct-I2C mag path).
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

struct qmc6309_emul_data {
	uint8_t regs[0x40];
	int32_t last_mag_idx; /* recording index of the sample last latched into 0x01..0x06 */
};

/* Latest recording index at or before now that carries a mag sample. */
static int32_t latest_mag_idx(float m[3])
{
	uint64_t us = k_ticks_to_us_floor64(k_uptime_ticks());
	int32_t idx = (int32_t)(us * (uint64_t)slime_sim_rate_hz() / 1000000ULL);
	float g[3], a[3];
	int has_mag;
	for (int32_t i = idx; i >= 0 && i > idx - 64; i--) {
		if (slime_sim_sample((uint32_t)i, g, a, m, &has_mag) == 0 && has_mag) {
			return i;
		}
	}
	return -1;
}

static uint8_t read_reg(struct qmc6309_emul_data *d, uint8_t reg)
{
	if (reg == 0x00) {
		return 0x90;
	}
	if (reg == 0x09) {
		float m[3];
		int32_t idx = latest_mag_idx(m);
		if (idx > d->last_mag_idx) {
			for (int i = 0; i < 3; i++) {
				float raw = roundf(m[i] * 4000.0f);
				int16_t v = (int16_t)CLAMP(raw, -32768.0f, 32767.0f);
				d->regs[1 + 2 * i] = (uint8_t)v;
				d->regs[2 + 2 * i] = (uint8_t)((uint16_t)v >> 8);
			}
			d->last_mag_idx = idx;
			return 0x01;
		}
		return 0x00;
	}
	return d->regs[reg % sizeof(d->regs)];
}

static int qmc6309_emul_transfer(const struct emul *target, struct i2c_msg *msgs, int num_msgs, int addr)
{
	ARG_UNUSED(addr);
	struct qmc6309_emul_data *d = target->data;
	uint8_t reg = 0;
	for (int i = 0; i < num_msgs; i++) {
		struct i2c_msg *msg = &msgs[i];
		if (msg->flags & I2C_MSG_READ) {
			for (uint32_t j = 0; j < msg->len; j++) {
				msg->buf[j] = read_reg(d, reg++);
			}
		} else if (msg->len > 0) {
			reg = msg->buf[0];
			for (uint32_t j = 1; j < msg->len; j++) {
				d->regs[reg++ % sizeof(d->regs)] = msg->buf[j];
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
	d->last_mag_idx = -1;
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
