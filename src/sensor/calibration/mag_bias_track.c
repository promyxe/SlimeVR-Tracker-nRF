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

#include "calibration.h"
#include "mag_common.h"
#include "mag_bias_track.h"

#if IS_ENABLED(CONFIG_SENSOR_MAG_HARD_IRON_TRACKING)

LOG_MODULE_REGISTER(cal_mag_bias, LOG_LEVEL_INF);

/*
 * The full Magneto ellipsoid fit (manual and online calibration) is episodic:
 * it needs ~192 direction-diverse samples and each accepted update resets the
 * fusion mag reference (~6 s of heading re-establishment). Between fits the
 * hard-iron offset of the magnetometer wanders (QMC6309: temperature, no
 * on-chip compensation) and heading slowly follows it.
 *
 * This module tracks the RESIDUAL sphere center of the already-calibrated
 * samples with a 4-state recursive least squares fit and folds small offset
 * corrections back into magBAinv[0] continuously. With calibration
 * c = A(raw - b0) and a drifted true offset, samples satisfy |c - d| = H
 * where d is the residual center in calibrated space; correcting
 * b0 += A^-1 * d removes it. Soft iron is left to the episodic Magneto fit.
 *
 * Linear formulation: |c|^2 = 2 c.d + (H^2 - |d|^2), i.e. y = phi^T theta
 * with phi = [cx, cy, cz, 1] and theta = [2d; H^2 - |d|^2] — standard RLS.
 *
 * Corrections are applied in small rate-limited steps WITHOUT resetting the
 * fusion mag reference: each step is far below VQF's norm threshold, so the
 * reference tracks it through normal adaptation and heading never snaps.
 */

/* Forgetting factor: ~1000 accepted samples of memory (minutes of rotation) */
#define MBT_RLS_LAMBDA 0.999f
#define MBT_RLS_P0 10.0f
/* Sample acceptance */
#define MBT_MIN_INTERVAL_MS 50
#define MBT_MIN_DIR_CHANGE_COS 0.996f /* ~5 degrees */
/* Innovation gate once converged: reject wildly inconsistent samples and
 * reset when they persist (magnetic environment changed) */
#define MBT_MIN_SAMPLES 150
#define MBT_INNOVATION_MAX_FRAC 0.2f
#define MBT_INNOVATION_RESET_COUNT 50
/* Correction application */
#define MBT_APPLY_INTERVAL_MS 4000
#define MBT_DEADBAND_FRAC 0.005f  /* ignore centers below 0.5% of field norm */
#define MBT_MAX_STEP_FRAC 0.02f   /* per-apply step limit: 2% of field norm */
#define MBT_MAX_CENTER_FRAC 0.35f /* implausible center: reset instead of applying */
/* NVS write policy for folded corrections */
#define MBT_NVS_MIN_DELTA_FRAC 0.01f
#define MBT_NVS_MIN_INTERVAL_MS 120000

static float mbt_theta[4];
static float mbt_P[4][4];
static uint32_t mbt_sample_count;
static uint32_t mbt_reject_count;
static float mbt_last_dir[3];
static bool mbt_have_dir;
static int64_t mbt_last_sample_time;
static int64_t mbt_last_apply_time;

/* Snapshot of magBAinv to detect external calibration changes */
static float mbt_cal_snapshot[4][3];
static bool mbt_initialized;

/* Accumulated folded correction (in raw units) pending NVS write */
static float mbt_unsaved_delta;
static int64_t mbt_last_nvs_write_time;

static void mbt_reset(const float m_cal[3])
{
	memset(mbt_theta, 0, sizeof(mbt_theta));
	memset(mbt_P, 0, sizeof(mbt_P));
	for (int i = 0; i < 4; i++) {
		mbt_P[i][i] = MBT_RLS_P0;
	}
	if (m_cal) {
		// Initialize rho with the current squared norm (d = 0, H = |c|)
		mbt_theta[3] = magneto_norm_sq(m_cal);
	}
	mbt_sample_count = 0;
	mbt_reject_count = 0;
	mbt_have_dir = false;
	memcpy(mbt_cal_snapshot, magBAinv, sizeof(mbt_cal_snapshot));
	mbt_initialized = true;
}

static bool mbt_invert_soft_iron(float a_inv[3][3])
{
	const float (*a)[3] = &magBAinv[1]; /* rows 1..3 form the soft-iron matrix */
	float det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
	          - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
	          + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
	if (fabsf(det) < 1e-9f) {
		return false;
	}
	float inv_det = 1.0f / det;
	a_inv[0][0] = (a[1][1] * a[2][2] - a[1][2] * a[2][1]) * inv_det;
	a_inv[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * inv_det;
	a_inv[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * inv_det;
	a_inv[1][0] = (a[1][2] * a[2][0] - a[1][0] * a[2][2]) * inv_det;
	a_inv[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * inv_det;
	a_inv[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * inv_det;
	a_inv[2][0] = (a[1][0] * a[2][1] - a[1][1] * a[2][0]) * inv_det;
	a_inv[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * inv_det;
	a_inv[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * inv_det;
	return true;
}

static void mbt_rls_update(const float phi[4], float y)
{
	// k = P*phi / (lambda + phi'*P*phi)
	float p_phi[4];
	for (int i = 0; i < 4; i++) {
		p_phi[i] = mbt_P[i][0] * phi[0] + mbt_P[i][1] * phi[1] + mbt_P[i][2] * phi[2] + mbt_P[i][3] * phi[3];
	}
	float denom = MBT_RLS_LAMBDA + phi[0] * p_phi[0] + phi[1] * p_phi[1] + phi[2] * p_phi[2] + phi[3] * p_phi[3];
	if (denom < 1e-9f) {
		return;
	}
	float inv_denom = 1.0f / denom;

	float err = y - (phi[0] * mbt_theta[0] + phi[1] * mbt_theta[1] + phi[2] * mbt_theta[2] + phi[3] * mbt_theta[3]);

	float k[4];
	for (int i = 0; i < 4; i++) {
		k[i] = p_phi[i] * inv_denom;
		mbt_theta[i] += k[i] * err;
	}

	// P = (P - k * (P*phi)') / lambda, symmetrized
	float inv_lambda = 1.0f / MBT_RLS_LAMBDA;
	for (int i = 0; i < 4; i++) {
		for (int j = i; j < 4; j++) {
			float v = (mbt_P[i][j] - k[i] * p_phi[j]) * inv_lambda;
			mbt_P[i][j] = v;
			mbt_P[j][i] = v;
		}
	}
}

/* Fold a calibrated-space center step into magBAinv[0] and shift the
 * estimator state so it keeps tracking the remaining residual. */
static void mbt_apply_step(const float step[3], float field_norm)
{
	float a_inv[3][3];
	if (!mbt_invert_soft_iron(a_inv)) {
		return;
	}

	float step_raw[3];
	for (int i = 0; i < 3; i++) {
		step_raw[i] = a_inv[i][0] * step[0] + a_inv[i][1] * step[1] + a_inv[i][2] * step[2];
	}
	for (int i = 0; i < 3; i++) {
		magBAinv[0][i] += step_raw[i];
	}

	// Shift estimator state: samples now arrive with c' = c - step, so the
	// remaining center is d' = d - step (H unchanged). P is kept as an
	// approximation; steps are small relative to the field norm.
	float d[3] = {0.5f * mbt_theta[0] - step[0], 0.5f * mbt_theta[1] - step[1], 0.5f * mbt_theta[2] - step[2]};
	float h_sq = mbt_theta[3] + 0.25f * (mbt_theta[0] * mbt_theta[0] + mbt_theta[1] * mbt_theta[1]
	                                     + mbt_theta[2] * mbt_theta[2]);
	mbt_theta[0] = 2.0f * d[0];
	mbt_theta[1] = 2.0f * d[1];
	mbt_theta[2] = 2.0f * d[2];
	mbt_theta[3] = h_sq - (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);

	memcpy(mbt_cal_snapshot, magBAinv, sizeof(mbt_cal_snapshot));

	float step_mag = sqrtf(step_raw[0] * step_raw[0] + step_raw[1] * step_raw[1] + step_raw[2] * step_raw[2]);
	mbt_unsaved_delta += step_mag;

	LOG_INF("Hard-iron track: applied step %.5f %.5f %.5f (|d|=%.2f%% of field)",
	        (double)step[0], (double)step[1], (double)step[2],
	        (double)(100.0f * step_mag / MAX(field_norm, 1e-6f)));

	// Persist folded corrections occasionally; the correction is tiny per
	// step, so losing the tail on power loss is inconsequential.
	int64_t now = k_uptime_get();
	if (mbt_unsaved_delta >= MBT_NVS_MIN_DELTA_FRAC * field_norm
	    && (mbt_last_nvs_write_time == 0 || now - mbt_last_nvs_write_time >= MBT_NVS_MIN_INTERVAL_MS)) {
		sys_write(MAIN_MAG_BIAS_ID, &retained->magBAinv, magBAinv, sizeof(magBAinv));
		mbt_unsaved_delta = 0;
		mbt_last_nvs_write_time = now;
		LOG_INF("Hard-iron track: persisted folded corrections");
	}
}

static void mbt_try_apply(void)
{
	int64_t now = k_uptime_get();
	if (mbt_sample_count < MBT_MIN_SAMPLES) {
		return;
	}
	if (mbt_last_apply_time != 0 && now - mbt_last_apply_time < MBT_APPLY_INTERVAL_MS) {
		return;
	}
	mbt_last_apply_time = now;

	float d[3] = {0.5f * mbt_theta[0], 0.5f * mbt_theta[1], 0.5f * mbt_theta[2]};
	float d_mag_sq = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
	float h_sq = mbt_theta[3] + d_mag_sq;
	if (h_sq < 1e-9f) {
		return;
	}
	float h = sqrtf(h_sq);
	float d_mag = sqrtf(d_mag_sq);

	if (d_mag < MBT_DEADBAND_FRAC * h) {
		return;
	}
	if (d_mag > MBT_MAX_CENTER_FRAC * h) {
		// Implausibly large residual center: bad data (undetected
		// disturbance) rather than drift. Start over.
		LOG_WRN("Hard-iron track: residual center %.1f%% of field, resetting",
		        (double)(100.0f * d_mag / h));
		mbt_reset(NULL);
		return;
	}

	float step[3];
	float max_step = MBT_MAX_STEP_FRAC * h;
	float scale = d_mag > max_step ? max_step / d_mag : 1.0f;
	for (int i = 0; i < 3; i++) {
		step[i] = d[i] * scale;
	}
	mbt_apply_step(step, h);
}

void sensor_calibration_mag_bias_track_sample(const float m_cal[3])
{
	// Don't fight the manual figure-8 calibration
	if (magneto_progress & 0x80) {
		return;
	}

	float zero[3] = {0};
	if (v_diff_mag(magBAinv[0], zero) == 0) {
		return; // no calibration yet
	}

	// External calibration change (online/manual Magneto fit, clear):
	// calibrated space moved under us, start fresh
	if (!mbt_initialized || memcmp(mbt_cal_snapshot, magBAinv, sizeof(mbt_cal_snapshot)) != 0) {
		mbt_reset(m_cal);
		return;
	}

	int64_t now = k_uptime_get();
	if (now - mbt_last_sample_time < MBT_MIN_INTERVAL_MS) {
		return;
	}

	// A sphere center is only observable from rotation: require a minimum
	// direction change so resting samples do not drive the estimate.
	float dir[3];
	if (!magneto_normalize_direction(m_cal, dir)) {
		return;
	}
	if (mbt_have_dir) {
		float dot = dir[0] * mbt_last_dir[0] + dir[1] * mbt_last_dir[1] + dir[2] * mbt_last_dir[2];
		if (dot > MBT_MIN_DIR_CHANGE_COS) {
			return;
		}
	}
	memcpy(mbt_last_dir, dir, sizeof(mbt_last_dir));
	mbt_have_dir = true;
	mbt_last_sample_time = now;

	float y = magneto_norm_sq(m_cal);
	float phi[4] = {m_cal[0], m_cal[1], m_cal[2], 1.0f};

	// Innovation gate once converged: a residual far beyond anything offset
	// drift can produce means the environment changed
	if (mbt_sample_count >= MBT_MIN_SAMPLES) {
		float pred = phi[0] * mbt_theta[0] + phi[1] * mbt_theta[1] + phi[2] * mbt_theta[2] + mbt_theta[3];
		float d_mag_sq = 0.25f * (mbt_theta[0] * mbt_theta[0] + mbt_theta[1] * mbt_theta[1]
		                          + mbt_theta[2] * mbt_theta[2]);
		float h_sq = mbt_theta[3] + d_mag_sq;
		if (h_sq > 1e-9f && fabsf(y - pred) > MBT_INNOVATION_MAX_FRAC * h_sq) {
			mbt_reject_count++;
			if (mbt_reject_count >= MBT_INNOVATION_RESET_COUNT) {
				LOG_WRN("Hard-iron track: persistent inconsistency, resetting");
				mbt_reset(m_cal);
			}
			return;
		}
		mbt_reject_count = 0;
	}

	mbt_rls_update(phi, y);
	if (mbt_sample_count < UINT32_MAX) {
		mbt_sample_count++;
	}

	mbt_try_apply();
}

#endif /* CONFIG_SENSOR_MAG_HARD_IRON_TRACKING */
