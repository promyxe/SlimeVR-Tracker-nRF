/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "globals.h"
#include "sensor/sensor.h"
#include "system/system.h"
#include "util.h"

#include <math.h>
#include <string.h>

#include "mag_common.h"
#include "mag_temp.h"

#if IS_ENABLED(CONFIG_SENSOR_MAG_TEMP_COMPENSATION)

LOG_MODULE_REGISTER(cal_mag_temp, LOG_LEVEL_INF);

/*
 * The T-Cal pipeline compensates gyro bias vs temperature, but nothing
 * compensates the magnetometer: the QMC6309 has no on-chip temperature
 * sensor and its offset drifts as trackers heat up 10-15°C during a
 * session. The continuous hard-iron tracker corrects that drift in closed
 * loop but only slowly; this module learns the repeatable offset-vs-
 * temperature relation so fast thermal transitions are compensated
 * open-loop, using the IMU temperature (mounted millimeters away) as proxy.
 *
 * MODEL — a table of calibrated-space offset values in 1°C buckets. Only
 * DIFFERENCES between bucket values matter: compensation subtracts
 * C(T) - C(T_anchor) where T_anchor is the temperature at which the current
 * hard-iron state was established (module reset / calibration change), so
 * enabling compensation never introduces a step at the current temperature.
 *
 * LEARNING — at rest with an undisturbed field, the field in the sensor
 * frame is constant, so any change of the compensated calibrated sample as
 * temperature drifts is residual offset drift. Rest sessions chain the
 * baseline from bucket to bucket, updating the curve incrementally; because
 * learning runs on already-compensated samples it converges (residuals go
 * to zero once the curve is right).
 */

#define MTC_BUCKETS MAG_TEMP_COMP_BUCKETS
#define MTC_TEMP_MIN ((float)CONFIG_SENSOR_POLY_TEMP_MIN)
#define MTC_TEMP_MAX ((float)CONFIG_SENSOR_POLY_TEMP_MAX)
#define MTC_BUCKET_TEMP(i) (MTC_TEMP_MIN + (float)(i) + 0.5f)

#define MTC_EMA_ALPHA 0.05f          /* rest-sample smoothing */
#define MTC_BASELINE_MIN_SAMPLES 40  /* samples before the baseline is trusted */
#define MTC_STEP_MIN_SAMPLES 20      /* samples between chained updates */
#define MTC_TEMP_STEP_C 0.5f         /* minimum temperature change per update */
#define MTC_BLEND 0.3f               /* blend of new observation into a bucket */
#define MTC_RESIDUAL_MAX_FRAC 0.1f   /* residual sanity vs field norm */
#define MTC_DELTA_MAX_FRAC 0.5f      /* absolute curve value sanity vs field norm */
#define MTC_NVS_MIN_INTERVAL_MS 300000

/* Compensation anchor */
static float mtc_anchor_temp;
static bool mtc_anchor_set;
static float mtc_cal_snapshot[4][3];
static bool mtc_snapshot_set;

/* Rest learning session */
static bool mtc_session_active;
static float mtc_ema[3];
static uint32_t mtc_ema_count;
static bool mtc_baseline_set;
static float mtc_baseline[3];
static float mtc_base_temp;
static uint32_t mtc_count_since_baseline;

static int64_t mtc_last_nvs_write_ms;
static bool mtc_nvs_dirty;

static void mtc_session_reset(void)
{
	mtc_session_active = false;
	mtc_baseline_set = false;
	mtc_ema_count = 0;
	mtc_count_since_baseline = 0;
}

/* Interpolate the curve at a temperature from the nearest valid buckets.
 * Returns false when no bucket is set at all. */
static bool mtc_curve_value(float temp, float out[3])
{
	int lo = -1;
	int hi = -1;
	for (int i = 0; i < MTC_BUCKETS; i++) {
		if (retained->magTempComp.weight[i] == 0) {
			continue;
		}
		float t = MTC_BUCKET_TEMP(i);
		if (t <= temp && (lo < 0 || t > MTC_BUCKET_TEMP(lo))) {
			lo = i;
		}
		if (t >= temp && (hi < 0 || t < MTC_BUCKET_TEMP(hi))) {
			hi = i;
		}
	}

	if (lo < 0 && hi < 0) {
		return false;
	}
	if (lo < 0) {
		lo = hi; // below coverage: hold the lowest valid value
	}
	if (hi < 0) {
		hi = lo; // above coverage: hold the highest valid value
	}
	if (lo == hi) {
		memcpy(out, retained->magTempComp.delta[lo], sizeof(float) * 3);
		return true;
	}

	float t_lo = MTC_BUCKET_TEMP(lo);
	float t_hi = MTC_BUCKET_TEMP(hi);
	float f = (temp - t_lo) / (t_hi - t_lo);
	for (int i = 0; i < 3; i++) {
		out[i] = retained->magTempComp.delta[lo][i]
		       + f * (retained->magTempComp.delta[hi][i] - retained->magTempComp.delta[lo][i]);
	}
	return true;
}

/* Re-anchor when the hard-iron state changes: a Magneto fit or a hard-iron
 * tracker fold absorbed the offset at the CURRENT temperature, so the
 * relative correction must be zero here from now on. */
static void mtc_check_anchor(float temp)
{
	if (!mtc_snapshot_set || memcmp(mtc_cal_snapshot, magBAinv, sizeof(mtc_cal_snapshot)) != 0) {
		memcpy(mtc_cal_snapshot, magBAinv, sizeof(mtc_cal_snapshot));
		mtc_snapshot_set = true;
		mtc_anchor_temp = temp;
		mtc_anchor_set = true;
		mtc_session_reset();
	}
}

void sensor_calibration_mag_temp_apply(float m[3])
{
	if (!retained) {
		return;
	}
	if (m[0] == 0.0f && m[1] == 0.0f && m[2] == 0.0f) {
		return; // uncalibrated
	}

	float temp = sensor_get_current_imu_temperature();
	if (isnan(temp)) {
		return;
	}
	temp = CLAMP(temp, MTC_TEMP_MIN, MTC_TEMP_MAX);

	mtc_check_anchor(temp);
	if (!mtc_anchor_set) {
		mtc_anchor_temp = temp;
		mtc_anchor_set = true;
	}

	float c_now[3];
	float c_anchor[3];
	if (!mtc_curve_value(temp, c_now) || !mtc_curve_value(mtc_anchor_temp, c_anchor)) {
		return;
	}
	for (int i = 0; i < 3; i++) {
		m[i] -= c_now[i] - c_anchor[i];
	}
}

static void mtc_persist(void)
{
	mtc_nvs_dirty = true;
	int64_t now = k_uptime_get();
	if (mtc_last_nvs_write_ms != 0 && now - mtc_last_nvs_write_ms < MTC_NVS_MIN_INTERVAL_MS) {
		return;
	}
	sys_write(MAIN_MAG_TEMP_ID, &retained->magTempComp, &retained->magTempComp, sizeof(retained->magTempComp));
	mtc_last_nvs_write_ms = now;
	mtc_nvs_dirty = false;
}

void sensor_calibration_mag_temp_sample(const float m[3])
{
	if (!retained) {
		return;
	}

	// The field in the sensor frame is only constant while truly at rest
	if (!sensor_fusion_get_rest_detected()) {
		mtc_session_reset();
		return;
	}

	float temp = sensor_get_current_imu_temperature();
	if (isnan(temp) || temp < MTC_TEMP_MIN || temp >= MTC_TEMP_MAX) {
		return;
	}

	if (!mtc_session_active) {
		memcpy(mtc_ema, m, sizeof(mtc_ema));
		mtc_ema_count = 1;
		mtc_baseline_set = false;
		mtc_session_active = true;
		return;
	}

	for (int i = 0; i < 3; i++) {
		mtc_ema[i] += MTC_EMA_ALPHA * (m[i] - mtc_ema[i]);
	}
	mtc_ema_count++;

	if (!mtc_baseline_set) {
		if (mtc_ema_count < MTC_BASELINE_MIN_SAMPLES) {
			return;
		}
		memcpy(mtc_baseline, mtc_ema, sizeof(mtc_baseline));
		mtc_base_temp = temp;
		mtc_count_since_baseline = 0;
		mtc_baseline_set = true;

		// Seed the session-start bucket so later updates in this session
		// have a trusted reference to chain from
		int base_idx = (int)(mtc_base_temp - MTC_TEMP_MIN);
		if (base_idx >= 0 && base_idx < MTC_BUCKETS && retained->magTempComp.weight[base_idx] == 0) {
			float seed[3] = {0};
			mtc_curve_value(mtc_base_temp, seed); // keeps {0,0,0} for an empty curve
			memcpy(retained->magTempComp.delta[base_idx], seed, sizeof(seed));
			retained->magTempComp.weight[base_idx] = 1;
		}
		return;
	}

	mtc_count_since_baseline++;
	if (fabsf(temp - mtc_base_temp) < MTC_TEMP_STEP_C || mtc_count_since_baseline < MTC_STEP_MIN_SAMPLES) {
		return;
	}

	// Residual drift of the compensated sample since the baseline. Anything
	// large is not thermal offset drift (field change slipping through the
	// disturbance gates) — drop the session rather than learn it.
	float field_norm = sqrtf(magneto_norm_sq(mtc_ema));
	if (field_norm < 1e-6f) {
		return;
	}
	float residual[3];
	for (int i = 0; i < 3; i++) {
		residual[i] = mtc_ema[i] - mtc_baseline[i];
	}
	float residual_norm = sqrtf(magneto_norm_sq(residual));
	if (residual_norm > MTC_RESIDUAL_MAX_FRAC * field_norm) {
		LOG_WRN("Mag temp comp: residual %.1f%% of field at rest, dropping session",
		        (double)(100.0f * residual_norm / field_norm));
		mtc_session_reset();
		return;
	}

	int idx = (int)(temp - MTC_TEMP_MIN);
	if (idx < 0 || idx >= MTC_BUCKETS) {
		return;
	}

	// Required curve value at this temperature: current interpolation plus
	// the observed residual (learning runs on compensated samples, so this
	// is an incremental refinement that converges)
	float target[3] = {0};
	mtc_curve_value(temp, target);
	for (int i = 0; i < 3; i++) {
		target[i] += residual[i];
	}

	bool sane = true;
	for (int i = 0; i < 3; i++) {
		if (fabsf(target[i]) > MTC_DELTA_MAX_FRAC * field_norm) {
			sane = false;
		}
	}
	if (!sane) {
		mtc_session_reset();
		return;
	}

	if (retained->magTempComp.weight[idx] == 0) {
		memcpy(retained->magTempComp.delta[idx], target, sizeof(target));
		retained->magTempComp.weight[idx] = 1;
	} else {
		for (int i = 0; i < 3; i++) {
			retained->magTempComp.delta[idx][i] += MTC_BLEND * (target[i] - retained->magTempComp.delta[idx][i]);
		}
		if (retained->magTempComp.weight[idx] < UINT8_MAX) {
			retained->magTempComp.weight[idx]++;
		}
	}

	LOG_INF("Mag temp comp: bucket %.1fC updated (res %.4f %.4f %.4f, w=%u)",
	        (double)MTC_BUCKET_TEMP(idx),
	        (double)residual[0], (double)residual[1], (double)residual[2],
	        retained->magTempComp.weight[idx]);

	mtc_persist();

	// Chain: this point becomes the trusted reference for the next step
	memcpy(mtc_baseline, mtc_ema, sizeof(mtc_baseline));
	mtc_base_temp = temp;
	mtc_count_since_baseline = 0;
}

void sensor_calibration_mag_temp_clear(void)
{
	if (!retained) {
		return;
	}
	memset(&retained->magTempComp, 0, sizeof(retained->magTempComp));
	sys_write(MAIN_MAG_TEMP_ID, &retained->magTempComp, &retained->magTempComp, sizeof(retained->magTempComp));
	mtc_session_reset();
	mtc_anchor_set = false;
	mtc_snapshot_set = false;
	mtc_last_nvs_write_ms = 0;
	mtc_nvs_dirty = false;
	LOG_INF("Mag temp comp: cleared");
}

#endif /* CONFIG_SENSOR_MAG_TEMP_COMPENSATION */
