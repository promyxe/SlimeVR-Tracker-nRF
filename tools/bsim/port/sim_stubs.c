/* nrf52_bsim stubs (tools/bsim): no SAADC model (battery.c replaced by a full,
 * unplugged battery) and a few Cortex-M/linker symbols the firmware uses. */
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "sensor/calibration/calibration.h"
#include "sensor/calibration/online_mag.h"
#include "sensor/sensor.h"
#include "slime_sim.h"
#include "system/battery.h"

int battery_measure_enable(bool enable)
{
	(void)enable;
	return 0;
}

int battery_sample(void)
{
	return 4100;
}

int battery_charger_state(bool *plugged, bool *charging, bool *charged)
{
	*plugged = false;
	*charging = false;
	*charged = false;
	return 0;
}

int battery_charger_snapshot(bool *plugged, bool *charging, bool *charged)
{
	return battery_charger_state(plugged, charging, charged);
}

unsigned int battery_level_pptt(unsigned int batt_mV, const struct battery_level_point *curve)
{
	(void)batt_mV;
	(void)curve;
	return 9000;
}

/* Returns the level in pptt (0.01 %) and writes the voltage, like battery.c. */
int read_batt_mV(int *out)
{
	*out = 4100;
	return 9000;
}

/* CMSIS byte reverse (used by NCS esb.c); not in the bsim CMSIS replacement. */
uint32_t __REV(uint32_t value)
{
	return __builtin_bswap32(value);
}

/* Linker-script symbol (end of the image) on real targets; esb_ota.c only reports it. */
char _flash_used[1];

FUNC_NORETURN void sys_poweroff(void)
{
	printk("sim: sys_poweroff\n");
	for (;;) {
		k_sleep(K_FOREVER);
	}
}

/* Retained settings start invalidated in simulation, so the magnetometer would stay
 * disabled; enable it shortly after boot like the console "mag" command does. */
/* Then start a manual calibration like the console "mag cal" command. */
static void sim_mag_cal(struct k_work *work)
{
	(void)work;
	int err = sensor_request_calibration_mag();
	printk("sim: mag cal requested (%d)\n", err);
}
static K_WORK_DELAYABLE_DEFINE(sim_mag_cal_work, sim_mag_cal);

static void sim_mag_enable(struct k_work *work)
{
	(void)work;
	if (slime_sim_mag_requested()) {
		sensor_calibration_set_online_mag_debug(true);
		sensor_set_mag_enabled(true);
		if (slime_sim_magcal_requested()) {
			k_work_schedule(&sim_mag_cal_work, K_SECONDS(1));
		}
	}
}
static K_WORK_DELAYABLE_DEFINE(sim_mag_work, sim_mag_enable);

static int sim_mag_init(void)
{
	k_work_schedule(&sim_mag_work, K_MSEC(100));
	return 0;
}
SYS_INIT(sim_mag_init, APPLICATION, 99);
