/* Bridge between the firmware (embedded side) and the host-side data reader
 * (sim_host.c, linked into the native simulator runner). nrf52_bsim only. */
#ifndef SLIME_SIM_H
#define SLIME_SIM_H

#include <stdint.h>

/* Sample idx of the SLIME_SIM_CSV recording (raw-collection format: gyro dps,
 * accel g, mag gauss in sensor frame). Returns 0 and fills the vectors, or -1
 * past the end. *has_mag is set when the row carries a mag sample. */
int slime_sim_sample(uint32_t idx, float g[3], float a[3], float m[3], int *has_mag);
/* Sample rate of the recording (gyro_odr_hz of its .meta.txt, default 400). */
float slime_sim_rate_hz(void);
/* 1 unless SLIME_SIM_MAG=0: enable the magnetometer after boot like the console "mag" command. */
int slime_sim_mag_requested(void);
/* 1 unless SLIME_SIM_MAGCAL=0: run a manual "mag cal" 1 s after enabling the mag. */
int slime_sim_magcal_requested(void);

int slime_sim_cpu_us_per_sample(void);
/* Append "t_us,qw,qx,qy,qz" to SLIME_SIM_OUT. */
void slime_sim_log_quat(uint64_t t_us, const float q[4]);

#endif
