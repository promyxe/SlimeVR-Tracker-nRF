/*
 * ICM-45686 SPI register emulator for nrf52_bsim (tools/bsim).
 *
 * Register file that stores writes and returns them on reads (so the driver's
 * readback checks pass), plus:
 *   0x72 WHO_AM_I          -> 0xE9
 *   0x12/0x13 FIFO_COUNT   -> frames available (big-endian), from simulated time
 *   0x14 FIFO_DATA         -> 20-byte high-resolution frames built from the
 *                             SLIME_SIM_CSV recording (sensor frame, +-32 g / +-4000 dps)
 * Indirect (IREG) registers read as 0. Overflow is simplified: when more than
 * a full FIFO is pending, the oldest samples are dropped (the real chip in
 * stop-on-full mode drops the newest).
 */
#define DT_DRV_COMPAT slimevr_icm45686_emul

#include <math.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi_emul.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "slime_sim.h"

LOG_MODULE_REGISTER(icm45686_emul, LOG_LEVEL_INF);

#define REG_FIFO_COUNT_0 0x12
#define REG_FIFO_COUNT_1 0x13
#define REG_FIFO_DATA 0x14
#define REG_WHO_AM_I 0x72
#define FRAME_SIZE 20
#define FIFO_FRAMES (2048 / FRAME_SIZE)

struct icm45686_emul_data {
	uint8_t regs[128];
	uint32_t next_idx; /* next recording sample to put into the FIFO */
	uint8_t frame[FRAME_SIZE];
	uint8_t frame_pos; /* bytes of frame[] already clocked out; FRAME_SIZE = need new frame */
	bool started;
	uint32_t start_idx; /* recording index at first FIFO access (simulated time zero point) */
};

static uint32_t now_idx(struct icm45686_emul_data *d)
{
	uint64_t us = k_ticks_to_us_floor64(k_uptime_ticks());
	return (uint32_t)(us * (uint64_t)slime_sim_rate_hz() / 1000000ULL);
}

static uint32_t frames_available(struct icm45686_emul_data *d)
{
	uint32_t now = now_idx(d);
	if (!d->started) {
		d->started = true;
		d->next_idx = now;
	}
	if (now - d->next_idx > FIFO_FRAMES) {
		d->next_idx = now - FIFO_FRAMES;
	}
	return now - d->next_idx;
}

static void put20(uint8_t *hi, uint8_t *mid, uint8_t *nib, float value, float full_scale)
{
	float scaled = roundf(value / full_scale * 524288.0f); /* 2^19 */
	if (scaled > 524287.0f) {
		scaled = 524287.0f;
	} else if (scaled < -524288.0f) {
		scaled = -524288.0f;
	}
	uint32_t left = (uint32_t)((int32_t)scaled) << 12; /* left-aligned in int32 */
	*hi = (uint8_t)(left >> 24);
	*mid = (uint8_t)(left >> 16);
	*nib = (uint8_t)((left >> 12) & 0x0F);
}

static void build_frame(struct icm45686_emul_data *d)
{
	float g[3], a[3], m[3];
	int has_mag;
	memset(d->frame, 0, sizeof(d->frame));
	if (slime_sim_sample(d->next_idx, g, a, m, &has_mag)) {
		d->frame[0] = 0xFF; /* empty frame marker: decoder skips it */
	} else {
		d->frame[0] = 0x78; /* hi-res, accel + gyro */
		for (int i = 0; i < 3; i++) {
			uint8_t an, gn;
			put20(&d->frame[1 + 2 * i], &d->frame[2 + 2 * i], &an, a[i], 32.0f);
			put20(&d->frame[7 + 2 * i], &d->frame[8 + 2 * i], &gn, g[i], 4000.0f);
			d->frame[17 + i] = (uint8_t)((an << 4) | gn);
		}
		int16_t traw = (int16_t)((30.0f - 25.0f) * 128.0f);
		d->frame[13] = (uint8_t)((uint16_t)traw >> 8);
		d->frame[14] = (uint8_t)traw;
	}
	d->next_idx++;
	d->frame_pos = 0;
}

static uint8_t read_reg(struct icm45686_emul_data *d, uint8_t reg)
{
	switch (reg) {
	case REG_WHO_AM_I:
		return 0xE9;
	case REG_FIFO_COUNT_0:
		return (uint8_t)(frames_available(d) >> 8);
	case REG_FIFO_COUNT_1:
		return (uint8_t)frames_available(d);
	case 0x7E: /* IREG_DATA */
		return 0;
	default:
		return d->regs[reg & 0x7F];
	}
}

static int icm45686_emul_io(const struct emul *target, const struct spi_config *config,
			    const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	ARG_UNUSED(config);
	struct icm45686_emul_data *d = target->data;
	size_t tx_len = 0, rx_len = 0;
	for (size_t i = 0; tx_bufs && i < tx_bufs->count; i++) tx_len += tx_bufs->buffers[i].len;
	for (size_t i = 0; rx_bufs && i < rx_bufs->count; i++) rx_len += rx_bufs->buffers[i].len;
	size_t total = MAX(tx_len, rx_len);
	if (total == 0) {
		return 0;
	}

	/* Flatten MOSI; generate MISO byte by byte. */
	uint8_t mosi[64] = {0};
	size_t pos = 0;
	for (size_t i = 0; tx_bufs && i < tx_bufs->count; i++) {
		const uint8_t *b = tx_bufs->buffers[i].buf;
		for (size_t j = 0; j < tx_bufs->buffers[i].len && pos < sizeof(mosi); j++, pos++) {
			mosi[pos] = b ? b[j] : 0;
		}
	}
	uint8_t cmd = mosi[0];
	bool read = cmd & 0x80;
	uint8_t reg = cmd & 0x7F;

	size_t rx_i = 0, rx_off = 0;
	for (size_t k = 0; k < total; k++) {
		uint8_t out = 0xFF;
		if (k > 0) {
			if (read) {
				if (reg == REG_FIFO_DATA) {
					if (d->frame_pos >= FRAME_SIZE) {
						build_frame(d);
					}
					out = d->frame[d->frame_pos++];
				} else {
					out = read_reg(d, reg);
					reg = (reg + 1) & 0x7F;
				}
			} else if (k < tx_len && k < sizeof(mosi)) {
				d->regs[reg] = mosi[k];
				reg = (reg + 1) & 0x7F;
			}
		}
		/* place into rx buffers (first byte of the transfer clocks out during the command) */
		while (rx_bufs && rx_i < rx_bufs->count && rx_off >= rx_bufs->buffers[rx_i].len) {
			rx_i++;
			rx_off = 0;
		}
		if (rx_bufs && rx_i < rx_bufs->count) {
			uint8_t *b = rx_bufs->buffers[rx_i].buf;
			if (b) {
				b[rx_off] = out;
			}
			rx_off++;
		}
	}
	return 0;
}

static int icm45686_emul_init(const struct emul *target, const struct device *parent)
{
	ARG_UNUSED(parent);
	struct icm45686_emul_data *d = target->data;
	memset(d, 0, sizeof(*d));
	d->frame_pos = FRAME_SIZE;
	return 0;
}

static const struct spi_emul_api icm45686_emul_api = {
	.io = icm45686_emul_io,
};

#define ICM45686_EMUL(n)                                                                           \
	static struct icm45686_emul_data icm45686_emul_data_##n;                                   \
	DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL, 99, NULL);                 \
	EMUL_DT_INST_DEFINE(n, icm45686_emul_init, &icm45686_emul_data_##n, NULL,                  \
			    &icm45686_emul_api, NULL)

DT_INST_FOREACH_STATUS_OKAY(ICM45686_EMUL)
