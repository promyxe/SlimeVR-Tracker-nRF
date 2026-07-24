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
#if IS_ENABLED(CONFIG_SENSOR_USE_VQF)
#include "fusion/vqf/vqf.h"
#endif

#include <math.h>
#include <string.h>

#include "sens_auto.h"

#if IS_ENABLED(CONFIG_SENSOR_USE_SENS_AUTO_CALIBRATION)

LOG_MODULE_REGISTER(cal_sens_auto, LOG_LEVEL_INF);

/*
 * The manual sensitivity calibration (cal_sens.c) asks the user to spin the
 * tracker a known number of turns. This module automates the same idea from
 * natural motion: whenever the tracker performs a large, dominantly
 * single-axis rotation between two quasi-static moments where absolute
 * orientation is anchored by gravity + magnetic field (TRIAD), the
 * gyro-integrated rotation angle is compared against the anchored orientation
 * delta. The ratio is a direct measurement of the residual sensitivity error
 * on the rotation axis and refines retained->gyroSensScale via a heavily
 * damped EMA.
 *
 * Gyro scale error is the one drift source bias estimation cannot touch:
 * 0.3% scale error accumulates ~1 deg of yaw per full turn, which the mag
 * can only slowly correct (and cannot correct at all when disabled).
 *
 * Robustness gates per event: both anchors quasi-static with undisturbed
 * mag, bounded event duration (bias residual contamination grows with
 * time), net rotation in a band away from the 180 deg angle fold, path
 * length close to net angle (direct rotation, no back-and-forth), dominant
 * single sensor axis, gyro/TRIAD rotation axis agreement, and a per-event
 * scale sanity band. The residual fusion gyro bias estimate is subtracted
 * before integration.
 */

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.01745329251994329577f
#endif

/* Anchor (quasi-static window) qualification */
#define SA_ANCHOR_GYR_MAX_DPS 2.0f
#define SA_ACCEL_NORM_MIN 0.9f
#define SA_ACCEL_NORM_MAX 1.1f
#define SA_ANCHOR_MIN_MS 500
#define SA_ANCHOR_MIN_ACCEL_SAMPLES 20
#define SA_ANCHOR_MIN_MAG_SAMPLES 5
#define SA_MAG_FRESH_MS 500
/* Event qualification */
#define SA_EVENT_MAX_MS 12000
#define SA_EVENT_MIN_ANGLE_RAD (90.0f * DEG_TO_RAD)
#define SA_EVENT_MAX_ANGLE_RAD (160.0f * DEG_TO_RAD)
#define SA_PATH_NET_TOL 0.2f
#define SA_AXIS_DOMINANCE_MIN 0.85f
#define SA_AXIS_AGREE_MIN_COS 0.9848f /* cos(10 deg) */
#define SA_EVENT_SCALE_MIN 0.97f
#define SA_EVENT_SCALE_MAX 1.03f
/* Scale update */
#define SA_EMA_ALPHA 0.05f
#define SA_SCALE_MIN 0.9f
#define SA_SCALE_MAX 1.1f
/* NVS write policy */
#define SA_NVS_MIN_INTERVAL_MS 300000
#define SA_NVS_MIN_DELTA 0.0002f

/* Quasi-static window accumulation (anchor candidate) */
static float win_accel_sum[3];
static int win_accel_count;
static float win_mag_sum[3];
static int win_mag_count;
static int64_t win_start_ms;
static bool win_active;

/* Cross-feed stillness flags */
static bool accel_in_band;
static bool mag_undisturbed;
static int64_t last_mag_ms;

/* Frozen anchor A: sensor->world rotation (rows east, north, up) */
static float anchor_R[3][3];
static bool anchor_valid;

/* Rotation event */
static bool event_active;
static float ev_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f}; /* current-sensor -> anchor-A frame */
static float ev_path_rad;
static float ev_bias_dps[3]; /* fusion residual bias at event start */
static int64_t ev_start_ms;

static int64_t last_nvs_write_ms;
static float nvs_written_scale[3];
static bool nvs_written_scale_valid;

static void sa_window_reset(void)
{
	memset(win_accel_sum, 0, sizeof(win_accel_sum));
	memset(win_mag_sum, 0, sizeof(win_mag_sum));
	win_accel_count = 0;
	win_mag_count = 0;
	win_start_ms = 0;
	win_active = false;
}

static bool sa_normalize(const float v[3], float dir[3])
{
	float norm_sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
	if (norm_sq < 1e-8f) {
		return false;
	}
	float inv = 1.0f / sqrtf(norm_sq);
	dir[0] = v[0] * inv;
	dir[1] = v[1] * inv;
	dir[2] = v[2] * inv;
	return true;
}

/* Build the TRIAD sensor->world rotation from averaged up/field directions.
 * Rows are east, north, up expressed in sensor coordinates. */
static bool sa_triad(const float accel_sum[3], const float mag_sum[3], float R[3][3])
{
	float up[3];
	float m_dir[3];
	if (!sa_normalize(accel_sum, up) || !sa_normalize(mag_sum, m_dir)) {
		return false;
	}

	// east = field x up (world: [0,cos(dip),-sin(dip)] x [0,0,1] = +east)
	float east[3] = {
		m_dir[1] * up[2] - m_dir[2] * up[1],
		m_dir[2] * up[0] - m_dir[0] * up[2],
		m_dir[0] * up[1] - m_dir[1] * up[0],
	};
	float east_norm_sq = east[0] * east[0] + east[1] * east[1] + east[2] * east[2];
	if (east_norm_sq < 0.01f) {
		return false; // field nearly vertical, no heading information
	}
	float inv = 1.0f / sqrtf(east_norm_sq);
	for (int i = 0; i < 3; i++) {
		east[i] *= inv;
	}

	float north[3] = {
		up[1] * east[2] - up[2] * east[1],
		up[2] * east[0] - up[0] * east[2],
		up[0] * east[1] - up[1] * east[0],
	};

	memcpy(R[0], east, sizeof(east));
	memcpy(R[1], north, sizeof(north));
	memcpy(R[2], up, sizeof(up));
	return true;
}

static void sa_persist_scale(void)
{
	if (!nvs_written_scale_valid) {
		memcpy(nvs_written_scale, retained->gyroSensScale, sizeof(nvs_written_scale));
		nvs_written_scale_valid = true;
		return; // baseline only; first delta accumulates against it
	}

	int64_t now = k_uptime_get();
	if (last_nvs_write_ms != 0 && now - last_nvs_write_ms < SA_NVS_MIN_INTERVAL_MS) {
		return;
	}

	float max_delta = 0;
	for (int i = 0; i < 3; i++) {
		float d = fabsf(retained->gyroSensScale[i] - nvs_written_scale[i]);
		if (d > max_delta) {
			max_delta = d;
		}
	}
	if (max_delta < SA_NVS_MIN_DELTA) {
		return;
	}

	sys_write(MAIN_GYRO_SENS_ID, &retained->gyroSensScale, retained->gyroSensScale, sizeof(retained->gyroSensScale));
	memcpy(nvs_written_scale, retained->gyroSensScale, sizeof(nvs_written_scale));
	last_nvs_write_ms = now;
	LOG_INF("Sens auto-cal: persisted scale %.5f %.5f %.5f",
	        (double)retained->gyroSensScale[0],
	        (double)retained->gyroSensScale[1],
	        (double)retained->gyroSensScale[2]);
}

static void sa_evaluate_event(const float anchor_B[3][3], int64_t now)
{
	int64_t duration = now - ev_start_ms;
	if (duration > SA_EVENT_MAX_MS) {
		return;
	}

	// True rotation (anchor B frame -> anchor A frame): R_rel = R_A^T * R_B.
	// R rows are world axes in sensor coords, so R maps sensor->world.
	float R_rel[3][3];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			R_rel[i][j] = anchor_R[0][i] * anchor_B[0][j] + anchor_R[1][i] * anchor_B[1][j]
			            + anchor_R[2][i] * anchor_B[2][j];
		}
	}

	float trace = R_rel[0][0] + R_rel[1][1] + R_rel[2][2];
	float cos_true = 0.5f * (trace - 1.0f);
	cos_true = CLAMP(cos_true, -1.0f, 1.0f);
	float angle_true = acosf(cos_true);

	if (angle_true < SA_EVENT_MIN_ANGLE_RAD || angle_true > SA_EVENT_MAX_ANGLE_RAD) {
		return;
	}

	float sin_true = sinf(angle_true);
	if (sin_true < 0.1f) {
		return;
	}
	float axis_true[3] = {
		(R_rel[2][1] - R_rel[1][2]) / (2.0f * sin_true),
		(R_rel[0][2] - R_rel[2][0]) / (2.0f * sin_true),
		(R_rel[1][0] - R_rel[0][1]) / (2.0f * sin_true),
	};

	// Measured rotation from the gyro-integrated quaternion (canonicalized)
	float q[4];
	memcpy(q, ev_quat, sizeof(q));
	if (q[0] < 0.0f) {
		for (int i = 0; i < 4; i++) {
			q[i] = -q[i];
		}
	}
	float cos_half = CLAMP(q[0], -1.0f, 1.0f);
	float angle_meas = 2.0f * acosf(cos_half);
	if (angle_meas < 1e-3f) {
		return;
	}
	float sin_half = sinf(0.5f * angle_meas);
	if (sin_half < 1e-6f) {
		return;
	}
	float axis_meas[3] = {q[1] / sin_half, q[2] / sin_half, q[3] / sin_half};

	// Direct single rotation: integrated path length must match the net angle,
	// otherwise back-and-forth motion partially cancels and the comparison is
	// meaningless
	if (fabsf(ev_path_rad - angle_meas) > SA_PATH_NET_TOL * angle_meas) {
		return;
	}

	// Dominantly single sensor axis so the scale attribution is unambiguous
	int axis = 0;
	float dominance = fabsf(axis_meas[0]);
	for (int i = 1; i < 3; i++) {
		if (fabsf(axis_meas[i]) > dominance) {
			dominance = fabsf(axis_meas[i]);
			axis = i;
		}
	}
	if (dominance < SA_AXIS_DOMINANCE_MIN) {
		return;
	}

	// Gyro and TRIAD must agree on the rotation axis; disagreement means a
	// mag anchor was corrupted (field changed between locations)
	float axis_dot = axis_meas[0] * axis_true[0] + axis_meas[1] * axis_true[1] + axis_meas[2] * axis_true[2];
	if (axis_dot < SA_AXIS_AGREE_MIN_COS) {
		return;
	}

	// Residual scale on the dominant axis. Gyro data was already scaled by
	// gyroSensScale, so this is a multiplicative refinement.
	float scale_event = angle_true / angle_meas;
	if (scale_event < SA_EVENT_SCALE_MIN || scale_event > SA_EVENT_SCALE_MAX) {
		return;
	}

	if (!retained) {
		return;
	}
	float current = retained->gyroSensScale[axis];
	if (current < SA_SCALE_MIN || current > SA_SCALE_MAX) {
		current = 1.0f;
	}
	float updated = current * (1.0f + SA_EMA_ALPHA * (scale_event - 1.0f));
	updated = CLAMP(updated, SA_SCALE_MIN, SA_SCALE_MAX);
	retained->gyroSensScale[axis] = updated;

	LOG_INF("Sens auto-cal: axis %c event %.1f deg true / %.1f deg meas (dom %.2f), scale %.5f -> %.5f",
	        "XYZ"[axis], (double)(angle_true / DEG_TO_RAD), (double)(angle_meas / DEG_TO_RAD),
	        (double)dominance, (double)current, (double)updated);

	sa_persist_scale();
}

void sensor_sens_auto_feed_accel(const float a[3])
{
	float norm_sq = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
	accel_in_band = norm_sq > SA_ACCEL_NORM_MIN * SA_ACCEL_NORM_MIN
	             && norm_sq < SA_ACCEL_NORM_MAX * SA_ACCEL_NORM_MAX;
	if (!accel_in_band) {
		sa_window_reset();
		return;
	}
	if (win_active) {
		for (int i = 0; i < 3; i++) {
			win_accel_sum[i] += a[i];
		}
		win_accel_count++;
	}
}

void sensor_sens_auto_feed_mag(const float m[3], bool mag_disturbed)
{
	last_mag_ms = k_uptime_get();
	mag_undisturbed = !mag_disturbed
#if IS_ENABLED(CONFIG_SENSOR_USE_VQF)
		&& !vqf_get_grad_cls_gradient()
#endif
		;
	if (mag_disturbed) {
		sa_window_reset();
		return;
	}
	if (win_active) {
		for (int i = 0; i < 3; i++) {
			win_mag_sum[i] += m[i];
		}
		win_mag_count++;
	}
}

void sensor_sens_auto_feed_gyro(const float g[3], float dt)
{
	if (dt <= 0.0f) {
		return;
	}
	int64_t now = k_uptime_get();

	// Integrate the active event with the fusion residual bias removed
	if (event_active) {
		if (now - ev_start_ms > SA_EVENT_MAX_MS) {
			// Too long: bias residual contamination outgrows the signal
			event_active = false;
			anchor_valid = false;
		} else {
			float w[3] = {
				(g[0] - ev_bias_dps[0]) * DEG_TO_RAD,
				(g[1] - ev_bias_dps[1]) * DEG_TO_RAD,
				(g[2] - ev_bias_dps[2]) * DEG_TO_RAD,
			};
			float w_norm = sqrtf(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
			float angle = w_norm * dt;
			ev_path_rad += angle;
			if (angle > 1e-8f) {
				float half = 0.5f * angle;
				float s = sinf(half) / w_norm;
				float step[4] = {cosf(half), s * w[0], s * w[1], s * w[2]};
				float q_new[4];
				q_multiply(ev_quat, step, q_new);
				q_normalize(q_new, ev_quat);
			}
		}
	}

	float g_norm_sq = g[0] * g[0] + g[1] * g[1] + g[2] * g[2];
	bool still = g_norm_sq < SA_ANCHOR_GYR_MAX_DPS * SA_ANCHOR_GYR_MAX_DPS && accel_in_band && mag_undisturbed;

	if (!still) {
		sa_window_reset();
		if (anchor_valid && !event_active) {
			// Leaving a mature anchor: start measuring the rotation
			event_active = true;
			ev_quat[0] = 1.0f;
			ev_quat[1] = 0.0f;
			ev_quat[2] = 0.0f;
			ev_quat[3] = 0.0f;
			ev_path_rad = 0.0f;
			ev_start_ms = now;
			if (!sensor_fusion_get_gyro_bias(ev_bias_dps)) {
				memset(ev_bias_dps, 0, sizeof(ev_bias_dps));
			}
		}
		return;
	}

	if (!win_active) {
		sa_window_reset();
		win_start_ms = now;
		win_active = true;
		return;
	}

	// Window maturity: enough time and samples, with fresh mag data
	if (now - win_start_ms >= SA_ANCHOR_MIN_MS
	    && win_accel_count >= SA_ANCHOR_MIN_ACCEL_SAMPLES
	    && win_mag_count >= SA_ANCHOR_MIN_MAG_SAMPLES
	    && now - last_mag_ms <= SA_MAG_FRESH_MS) {
		float R[3][3];
		if (sa_triad(win_accel_sum, win_mag_sum, R)) {
			if (anchor_valid && event_active) {
				sa_evaluate_event(R, now);
			}
			memcpy(anchor_R, R, sizeof(anchor_R));
			anchor_valid = true;
		}
		event_active = false;
		// Restart the window so the anchor refreshes while resting
		sa_window_reset();
		win_start_ms = now;
		win_active = true;
	}
}

#endif /* CONFIG_SENSOR_USE_SENS_AUTO_CALIBRATION */
