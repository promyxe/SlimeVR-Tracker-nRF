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
#include "util.h"

#include <math.h>
#if defined(CONFIG_VQF_BENCH)
#include <zephyr/kernel.h>
#include "thread_priority.h"
#endif

#if defined(CONFIG_VQF_BENCH) && defined(CONFIG_CPU_CORTEX_M_HAS_DWT)
#include <cmsis_core.h>
#endif

#include "../src/vqf.h" // conflicting with vqf.h in local path

#include "../vqf/vqf.h" // conflicting with vqf.h in vqf-c

#include "retained.h" // for BUILD_ASSERT on fusion_data size

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.01745329251994329577f /* (float)(M_PI / 180.0) */
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.29577951308232087680f /* (float)(180.0 / M_PI) */
#endif

#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
/* ---------- Adaptive tauAcc configuration ---------- */
/*
 * Three-regime adaptive tauAcc strategy:
 *
 * 1. REST (VQF rest detection active):
 *    tauAcc = TAU_ACC_REST (moderate).
 *    At rest the accelerometer cleanly reflects gravity and the rest
 *    bias estimator handles gyro drift, so a moderate tau provides
 *    stable inclination without needing fast correction.
 *
 * 2. GENTLE MOTION (accel close to gravity, low linear acceleration):
 *    tauAcc = TAU_ACC_GENTLE (low).
 *    Slow orientation changes (rolling, tilting) must be tracked
 *    quickly.  A low tau enables fast inclination correction so the
 *    acc LP filter keeps up with the actual gravity direction,
 *    preventing heading drift from stale inclination estimates.
 *
 * 3. AGGRESSIVE MOTION (significant linear acceleration):
 *    tauAcc = TAU_ACC_AGGRESSIVE (higher).
 *    Centripetal / dynamic accelerations corrupt the gravity estimate.
 *    A higher tau rejects these transients at the cost of slower
 *    inclination tracking.
 *
 * The accel deviation |‖a‖ - g| drives a [0,1] motion_intensity
 * with fast-attack / slow-release dynamics.  During motion, tauAcc
 * is linearly interpolated from GENTLE (intensity=0) to AGGRESSIVE
 * (intensity=1).  When rest is detected, TAU_ACC_REST overrides.
 */
#define ADAPTIVE_TAU_ACC_REST 3.0f       /* tauAcc when at rest (seconds) */
#define ADAPTIVE_TAU_ACC_GENTLE 2.0f     /* tauAcc during gentle motion (seconds) */
#define ADAPTIVE_TAU_ACC_AGGRESSIVE 4.3f /* tauAcc under aggressive motion (seconds) */
#define ADAPTIVE_TAU_ACC_LEVELS 5        /* quantization levels */
#define ADAPTIVE_ACC_DEV_TH 2.0f         /* accel deviation threshold (m/s²) */
#define ADAPTIVE_ATTACK_ALPHA 0.4f       /* attack coefficient: fast increase (per sample) */
#define ADAPTIVE_RELEASE_ALPHA 0.2f      /* release coefficient: slow decrease (per sample) */
#define TAU_SMOOTH_ALPHA_DOWN 0.21f      /* tauAcc decrease smoothing (per sample) */
#define TAU_SMOOTH_ALPHA_UP 0.21f        /* tauAcc increase smoothing (per sample) */
#endif                                   /* CONFIG_VQF_ADAPTIVE_TAU_ACC */

#if IS_ENABLED(CONFIG_VQF_MAG_SLEW_LIMIT)
/* ---------- Magnetometer heading slew-rate limiting ---------- */
/*
 * VQF applies k*disAngle to the heading offset delta on every mag update.
 * With the normal gain (k ≈ Ts/tauMag) a large disagreement — e.g. right
 * after an online calibration update re-establishes the mag reference, or
 * when a wrongly-accepted disturbed field pulls the heading — corrects at
 * up to disAngle/tauMag, which is a visible heading snap of tens of °/s.
 *
 * Clamping the per-update delta change bounds both effects: corrections
 * stay below a rate that is imperceptible in VR, and a bad field can only
 * inject a bounded yaw error before disturbance rejection reacts.
 *
 * Not applied while kMagInit is active (initial convergence must stay fast).
 * At rest the true disagreement drift is tiny, so a much lower ceiling is
 * used; during motion fresh field sampling justifies faster correction.
 */
#define MAG_SLEW_MAX_RATE_REST_DPS 1.0f
#define MAG_SLEW_MAX_RATE_MOTION_DPS 8.0f
#endif /* CONFIG_VQF_MAG_SLEW_LIMIT */

#if IS_ENABLED(CONFIG_VQF_MAG_GYRO_CONSISTENCY)
/* ---------- Gyro–mag direction consistency check ---------- */
/*
 * VQF's built-in disturbance detection compares only field NORM and DIP
 * angle against the reference. A disturbance that rotates the field while
 * roughly preserving norm and dip (common indoors: large steel structures,
 * a nearby tracker's magnet) passes that gate and pulls heading directly.
 *
 * The Earth field is fixed in the world frame, so over a short window the
 * measured field direction must rotate with the INVERSE of the sensor
 * rotation. Each window the mag direction at window start is rotated by
 * the conjugate of the (bias-corrected) gyro-integrated rotation and
 * compared against the current measurement; a direction error beyond a
 * rotation-scaled threshold marks the field as disturbed and heading
 * corrections are rejected for a hold period.
 *
 * The threshold's relative term absorbs gyro scale/misalignment residuals
 * and mag soft-iron residuals that grow with rotation magnitude. Windows
 * with more than MAG_CONSIST_MAX_ROT of rotation are skipped because the
 * quaternion angle folds beyond 180° and the threshold would be wrong.
 */
#define MAG_CONSIST_WINDOW_S 0.3f
#define MAG_CONSIST_BASE_TH_RAD (3.0f * DEG_TO_RAD)
#define MAG_CONSIST_REL_TH 0.10f
#define MAG_CONSIST_HOLD_S 2.0f
#define MAG_CONSIST_MAX_ROT_RAD (160.0f * DEG_TO_RAD)
#endif /* CONFIG_VQF_MAG_GYRO_CONSISTENCY */

static uint8_t imu_id;

static vqf_params_t params;
static vqf_state_t state;
static vqf_coeffs_t coeffs;
#define VQF_MEM_SIZE (sizeof(vqf_state_t) + sizeof(vqf_coeffs_t))

static float last_a[3] = {0};
#if defined(CONFIG_VQF_BENCH)
static volatile float vqf_bench_sink;
static vqf_params_t vqf_bench_params;
static vqf_params_t vqf_bench_warm_params;
static vqf_state_t vqf_bench_state;
static vqf_state_t vqf_bench_warm_state;
static vqf_coeffs_t vqf_bench_coeffs;
static vqf_coeffs_t vqf_bench_warm_coeffs;
static float vqf_bench_quat[4];

static const vqf_real_t vqf_bench_gyr_samples[][3] = {
	{0.12f, -0.34f, 0.56f},
	{-1.10f, 0.45f, 0.08f},
	{0.78f, 0.11f, -0.29f},
	{-0.05f, 0.92f, 0.37f},
};

static const vqf_real_t vqf_bench_acc_samples[][3] = {
	{0.30f, 0.10f, 9.70f},
	{-0.25f, 0.45f, 9.63f},
	{0.60f, -0.15f, 9.55f},
	{-0.40f, -0.20f, 9.81f},
};

static const vqf_real_t vqf_bench_mag_samples[][3] = {
	{0.32f, 0.05f, -0.41f},
	{0.28f, 0.10f, -0.39f},
	{0.35f, -0.02f, -0.43f},
	{0.30f, 0.08f, -0.40f},
};

static const size_t vqf_bench_sample_count = sizeof(vqf_bench_gyr_samples) / sizeof(vqf_bench_gyr_samples[0]);
#endif

#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
/* Adaptive tauAcc state */
static float motion_intensity;       /* low-pass filtered motion intensity [0,1] */
static float current_tau_level = -1; /* current quantized level (-1 = unset) */
static float smoothed_tau;           /* smoothed tauAcc for gradual transitions */
#endif

#if IS_ENABLED(CONFIG_VQF_MAG_GYRO_CONSISTENCY)
/* Gyro–mag consistency state */
static float consist_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f}; /* sensor rotation since window start */
static float consist_mag_ref[3];                         /* normalized mag direction at window start */
static bool consist_have_ref;
static float consist_window_t;
static float consist_hold_t; /* remaining disturbance hold time */
#endif

static float dist_continuous_s; /* persistent disturbance release timer */

/* Gradient-vs-transient field classifier state.
 *
 * The residual r = dm/dt + ω × m is ~0 during any rotation in a uniform
 * field (transport theorem: dm_body/dt = -ω × m_body). Non-zero r means
 * the field itself is changing in the world frame - either from a spatial
 * gradient (sensor moving through steel-distorted field) or a transient
 * (magnet/electronics). The discriminator is gated on VQF's restDetected:
 * large sustained residual at rest → transient; during motion → gradient.
 *
 * The gyro accumulator provides interval-averaged ω (vector sum, no
 * coning correction - coning error is negligible at the classifier's
 * noise floor). The EMA smooths the residual magnitude with τ ~ 0.5 s.
 */
static struct {
	float omega_sum[3];      /* bias-corrected gyro accumulated since last mag sample */
	float omega_dt;          /* accumulated gyro interval time */
	float prev_m[3];         /* previous mag sample for adjacent-sample diff */
	bool have_prev;          /* true once the first mag sample has been stored */
	float ema;               /* EMA of |r| */
	bool init;               /* EMA seeded after first valid dm/dt */
} grad_cls;

static bool grad_cls_gradient;
static bool grad_cls_transient;

/* Rest detection diagnostics */
static uint32_t rest_enter_count;
static uint32_t rest_exit_count;
static float rest_total_s;
static float rest_last_enter_time; /* uptime when last rest started */
static float rest_last_duration_s;
static float uptime_s;
static bool prev_rest_detected;

/* Circular rest event log */
#define REST_EVENT_LOG_SIZE 5
static struct {
	float time_s;
	bool entered;
} rest_event_log[REST_EVENT_LOG_SIZE];
static uint8_t rest_event_idx;   /* next write position */
static uint8_t rest_event_total; /* total events (up to log size) */

static void vqf_consist_reset(void);

void vqf_update_sensor_ids(int imu)
{
	ARG_UNUSED(imu);
}

static void set_params()
{
	init_params(&params);
	params.tauAcc = 4.3f;
	params.biasClip = 2.0f;
	params.biasForgettingTime = 427.0f;
	params.biasSigmaInit = 1.10f;
	params.biasSigmaMotion = 0.048f;
	params.biasSigmaRest = 0.0153f;
	params.biasVerticalForgettingFactor = 0.0001f;
	params.motionBiasEstEnabled = true;
	params.restBiasEstEnabled = true;
	params.restFilterTau = 0.66f;
	params.restMinT = 0.63f;
	params.restThGyr = 0.68f;
	params.restThAcc = 0.21f;
	params.magDistRejectionEnabled = true;
	params.tauMag = 6.0f;
	params.magCurrentTau = 0.50f;
	params.magNormTh = 0.08f;
	params.magDipTh = 6.0f;
	params.magRefTau = 15.0f;
	params.magNewTime = 12.0f;
	params.magNewFirstTime = 5.5f;
	params.magNewMinGyr = 16.0f;
	params.magMinUndisturbedTime = 1.0f;
	params.magMaxRejectionTime = 3200.0f;
	params.magRejectionFactor = 1150.0f;
}

void vqf_init(float g_time, float a_time, float m_time)
{
	set_params();
	initVqf(&params, &state, &coeffs, g_time, a_time, m_time);
#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
	motion_intensity = 0.0f;
	current_tau_level = -1.0f;
	smoothed_tau = params.tauAcc;
#endif
	rest_enter_count = 0;
	rest_exit_count = 0;
	rest_total_s = 0;
	rest_last_enter_time = 0;
	rest_last_duration_s = 0;
	uptime_s = 0;
	prev_rest_detected = false;
	rest_event_idx = 0;
	rest_event_total = 0;
	dist_continuous_s = 0;
	memset(&grad_cls, 0, sizeof(grad_cls));
	grad_cls_gradient = false;
	grad_cls_transient = false;
	vqf_consist_reset();
}

void vqf_load(const void *data)
{
	BUILD_ASSERT(
		VQF_MEM_SIZE <= sizeof(((struct retained_data *)0)->fusion_data),
		"VQF state+coeffs exceeds fusion_data buffer in retained memory"
	);
	set_params();
	memcpy(&state, data, sizeof(state));
	memcpy(&coeffs, (uint8_t *)data + sizeof(state), sizeof(coeffs));
#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
	motion_intensity = 0.0f;
	current_tau_level = -1.0f;
	smoothed_tau = params.tauAcc;
#endif
	rest_enter_count = 0;
	rest_exit_count = 0;
	rest_total_s = 0;
	rest_last_enter_time = 0;
	rest_last_duration_s = 0;
	uptime_s = 0;
	prev_rest_detected = false;
	rest_event_idx = 0;
	rest_event_total = 0;
	dist_continuous_s = 0;
	memset(&grad_cls, 0, sizeof(grad_cls));
	grad_cls_gradient = false;
	grad_cls_transient = false;
	vqf_consist_reset();
}

void vqf_save(void *data)
{
	BUILD_ASSERT(
		VQF_MEM_SIZE <= sizeof(((struct retained_data *)0)->fusion_data),
		"VQF state+coeffs exceeds fusion_data buffer in retained memory"
	);
	memcpy(data, &state, sizeof(state));
	memcpy((uint8_t *)data + sizeof(state), &coeffs, sizeof(coeffs));
}

#if IS_ENABLED(CONFIG_VQF_MAG_GYRO_CONSISTENCY)
static void vqf_consist_reset(void)
{
	consist_quat[0] = 1.0f;
	consist_quat[1] = 0.0f;
	consist_quat[2] = 0.0f;
	consist_quat[3] = 0.0f;
	consist_have_ref = false;
	consist_window_t = 0.0f;
	consist_hold_t = 0.0f;
	dist_continuous_s = 0;
	memset(&grad_cls, 0, sizeof(grad_cls));
	grad_cls_gradient = false;
	grad_cls_transient = false;
}

/**
 * @brief Accumulate the bias-corrected sensor rotation for the current window.
 *
 * Strapdown integration: consist_quat maps vectors from the current sensor
 * frame to the sensor frame at window start.
 */
static void vqf_consist_integrate_gyro(const float g_rad[3], float dt)
{
	if (!consist_have_ref || dt <= 0.0f) {
		return;
	}

	float w[3] = {g_rad[0] - state.bias[0], g_rad[1] - state.bias[1], g_rad[2] - state.bias[2]};
	float w_norm = sqrtf(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
	float angle = w_norm * dt;
	if (angle < 1e-8f) {
		return;
	}

	float half_angle = 0.5f * angle;
	float s = sinf(half_angle) / w_norm;
	float step[4] = {cosf(half_angle), s * w[0], s * w[1], s * w[2]};
	float q_new[4];
	q_multiply(consist_quat, step, q_new);
	q_normalize(q_new, consist_quat);
}

/**
 * @brief Per-mag-sample consistency evaluation.
 *
 * @param m calibrated mag sample (fusion input units)
 * @param dt time step of this mag update in seconds
 * @return true if the field is currently regarded as inconsistent (hold active)
 */
static bool vqf_consist_check_mag(const float m[3], float dt)
{
	if (consist_hold_t > 0.0f && dt > 0.0f) {
		consist_hold_t -= dt;
		if (consist_hold_t < 0.0f) {
			consist_hold_t = 0.0f;
		}
	}

	float m_norm_sq = m[0] * m[0] + m[1] * m[1] + m[2] * m[2];
	if (m_norm_sq < 1e-12f) {
		return consist_hold_t > 0.0f;
	}
	float inv_norm = 1.0f / sqrtf(m_norm_sq);
	float m_dir[3] = {m[0] * inv_norm, m[1] * inv_norm, m[2] * inv_norm};

	if (!consist_have_ref) {
		memcpy(consist_mag_ref, m_dir, sizeof(consist_mag_ref));
		consist_quat[0] = 1.0f;
		consist_quat[1] = 0.0f;
		consist_quat[2] = 0.0f;
		consist_quat[3] = 0.0f;
		consist_window_t = 0.0f;
		consist_have_ref = true;
		return consist_hold_t > 0.0f;
	}

	consist_window_t += dt;
	if (consist_window_t < MAG_CONSIST_WINDOW_S) {
		return consist_hold_t > 0.0f;
	}

	float cos_half = fabsf(consist_quat[0]);
	if (cos_half > 1.0f) {
		cos_half = 1.0f;
	}
	float rot_angle = 2.0f * acosf(cos_half);

	if (rot_angle < MAG_CONSIST_MAX_ROT_RAD) {
		// Predicted current measurement of the window-start field:
		// rotate the start direction into the current sensor frame.
		float q_inv[4];
		q_conj(consist_quat, q_inv);
		float m_pred[3];
		v_rotate(consist_mag_ref, q_inv, m_pred);

		float dot = m_pred[0] * m_dir[0] + m_pred[1] * m_dir[1] + m_pred[2] * m_dir[2];
		if (dot > 1.0f) {
			dot = 1.0f;
		} else if (dot < -1.0f) {
			dot = -1.0f;
		}
		float err = acosf(dot);
		float threshold = MAG_CONSIST_BASE_TH_RAD + MAG_CONSIST_REL_TH * rot_angle;
		if (err > threshold) {
			consist_hold_t = MAG_CONSIST_HOLD_S;
		}
	}

	// Start the next window from the current sample
	memcpy(consist_mag_ref, m_dir, sizeof(consist_mag_ref));
	consist_quat[0] = 1.0f;
	consist_quat[1] = 0.0f;
	consist_quat[2] = 0.0f;
	consist_quat[3] = 0.0f;
	consist_window_t = 0.0f;

	return consist_hold_t > 0.0f;
}
#else
static inline void vqf_consist_reset(void)
{
	dist_continuous_s = 0;
	memset(&grad_cls, 0, sizeof(grad_cls));
	grad_cls_gradient = false;
	grad_cls_transient = false;
}
#endif /* CONFIG_VQF_MAG_GYRO_CONSISTENCY */

void vqf_update_gyro(float *g, float time)
{
	float g_rad[3] = {0};
	// g is in deg/s, convert to rad/s
	for (int i = 0; i < 3; i++) {
		g_rad[i] = g[i] * DEG_TO_RAD;
	}
	updateGyr(&params, &state, &coeffs, g_rad);
	// Feed bias-corrected gyro to gradient classifier vector-sum accumulator
	float w[3] = {g_rad[0] - state.bias[0], g_rad[1] - state.bias[1], g_rad[2] - state.bias[2]};
	float dt = coeffs.gyrTs;
	for (int i = 0; i < 3; i++) {
		grad_cls.omega_sum[i] += w[i] * dt;
	}
	grad_cls.omega_dt += dt;
#if IS_ENABLED(CONFIG_VQF_MAG_GYRO_CONSISTENCY)
	vqf_consist_integrate_gyro(g_rad, dt);
#endif
}

void vqf_update_gyro_ts(float *g, uint64_t timestamp_us)
{
	float g_rad[3] = {0};
	for (int i = 0; i < 3; i++) {
		g_rad[i] = g[i] * DEG_TO_RAD;
	}
#if IS_ENABLED(CONFIG_VQF_MAG_GYRO_CONSISTENCY)
	float dt = coeffs.gyrTs;
	if (state.lastGyrTsUs != 0 && timestamp_us > state.lastGyrTsUs) {
		uint64_t diff = timestamp_us - state.lastGyrTsUs;
		if (diff > 0 && diff <= 10000000ULL) {
			dt = (float)diff / 1e6f;
		}
	}
#endif
	updateGyrTs(&params, &state, &coeffs, g_rad, timestamp_us);
	// Feed bias-corrected gyro to gradient classifier vector-sum accumulator
#if IS_ENABLED(CONFIG_VQF_MAG_GYRO_CONSISTENCY)
#else
	float dt = coeffs.gyrTs;
#endif
	{
		float w[3] = {g_rad[0] - state.bias[0], g_rad[1] - state.bias[1], g_rad[2] - state.bias[2]};
		for (int i = 0; i < 3; i++) {
			grad_cls.omega_sum[i] += w[i] * dt;
		}
		grad_cls.omega_dt += dt;
	}
#if IS_ENABLED(CONFIG_VQF_MAG_GYRO_CONSISTENCY)
#else
	float dt = coeffs.gyrTs;
#endif
	{
		float w[3] = {g_rad[0] - state.bias[0], g_rad[1] - state.bias[1], g_rad[2] - state.bias[2]};
		for (int i = 0; i < 3; i++) {
			grad_cls.omega_sum[i] += w[i] * dt;
		}
		grad_cls.omega_dt += dt;
	}
#if IS_ENABLED(CONFIG_VQF_MAG_GYRO_CONSISTENCY)
	vqf_consist_integrate_gyro(g_rad, dt);
#endif
}

#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
/**
 * @brief Pre-accel-update processing: adaptive tauAcc (experimental).
 *
 * Called before each accelerometer update to adjust VQF parameters based on
 * current motion intensity and temperature dynamics.
 *
 * @param a_m_s2 accelerometer reading in m/s²
 */
static void vqf_pre_accel_update(const float a_m_s2[3])
{
	/* --- Adaptive tauAcc based on motion intensity --- */
	float a_norm = sqrtf(a_m_s2[0] * a_m_s2[0] + a_m_s2[1] * a_m_s2[1] + a_m_s2[2] * a_m_s2[2]);
	float a_dev = fabsf(a_norm - CONST_EARTH_GRAVITY);
	float alpha_inst = fminf(a_dev / ADAPTIVE_ACC_DEV_TH, 1.0f);

	/* Attack-release envelope: fast increase (detect motion quickly),
	 * slow decrease (sustain elevated tau after motion stops) */
	if (alpha_inst > motion_intensity) {
		motion_intensity += ADAPTIVE_ATTACK_ALPHA * (alpha_inst - motion_intensity);
	} else {
		motion_intensity += ADAPTIVE_RELEASE_ALPHA * (alpha_inst - motion_intensity);
	}

	/*
	 * Compute target tauAcc:
	 * - Rest detected: use rest tauAcc
	 * - Motion: linearly interpolate from GENTLE (intensity=0) to
	 *   AGGRESSIVE (intensity=1).  This supports both increasing and
	 *   decreasing curves depending on how the defines are set.
	 */
	float target_tau;
	if (state.restDetected) {
		target_tau = ADAPTIVE_TAU_ACC_REST;
	} else {
		target_tau
			= ADAPTIVE_TAU_ACC_GENTLE + (ADAPTIVE_TAU_ACC_AGGRESSIVE - ADAPTIVE_TAU_ACC_GENTLE) * motion_intensity;
	}

	/*
	 * Smooth tauAcc transitions to avoid Butterworth LP filter transients.
	 * Decrease (rest→motion) is slow to prevent heading coupling from the
	 * sudden acc correction gain change.  Increase (motion→rest) is fast
	 * so the filter settles quickly at rest.
	 */
	float tau_alpha = (target_tau < smoothed_tau) ? TAU_SMOOTH_ALPHA_DOWN : TAU_SMOOTH_ALPHA_UP;
	smoothed_tau += tau_alpha * (target_tau - smoothed_tau);

	/*
	 * Quantize target_tau directly to avoid unnecessary setTauAcc() calls.
	 * The step size is derived from the full possible range of tauAcc values.
	 */
	float tau_min = fminf(fminf(ADAPTIVE_TAU_ACC_REST, ADAPTIVE_TAU_ACC_GENTLE), ADAPTIVE_TAU_ACC_AGGRESSIVE);
	float tau_max = fmaxf(fmaxf(ADAPTIVE_TAU_ACC_REST, ADAPTIVE_TAU_ACC_GENTLE), ADAPTIVE_TAU_ACC_AGGRESSIVE);
	float tau_step = (tau_max - tau_min) / ADAPTIVE_TAU_ACC_LEVELS;
	float quantized_tau;
	if (tau_step > 0.001f) {
		quantized_tau = tau_min + roundf((smoothed_tau - tau_min) / tau_step) * tau_step;
		quantized_tau = fmaxf(tau_min, fminf(quantized_tau, tau_max));
	} else {
		quantized_tau = smoothed_tau;
	}

	if (quantized_tau != current_tau_level) {
		current_tau_level = quantized_tau;
		setTauAcc(&params, &state, &coeffs, quantized_tau);
	}
}
#endif /* CONFIG_VQF_ADAPTIVE_TAU_ACC */

/**
 * @brief Track rest detection transitions and accumulate diagnostics.
 * Called after each accelerometer update (which runs rest detection).
 */
static void vqf_track_rest_diag(void)
{
	float dt = coeffs.accTs;
	uptime_s += dt;

	bool cur = state.restDetected;
	if (cur && !prev_rest_detected) {
		/* entering rest */
		rest_enter_count++;
		rest_last_enter_time = uptime_s;
		rest_event_log[rest_event_idx].time_s = uptime_s;
		rest_event_log[rest_event_idx].entered = true;
		rest_event_idx = (rest_event_idx + 1) % REST_EVENT_LOG_SIZE;
		if (rest_event_total < REST_EVENT_LOG_SIZE) {
			rest_event_total++;
		}
	} else if (!cur && prev_rest_detected) {
		/* leaving rest */
		rest_exit_count++;
		rest_last_duration_s = uptime_s - rest_last_enter_time;
		rest_event_log[rest_event_idx].time_s = uptime_s;
		rest_event_log[rest_event_idx].entered = false;
		rest_event_idx = (rest_event_idx + 1) % REST_EVENT_LOG_SIZE;
		if (rest_event_total < REST_EVENT_LOG_SIZE) {
			rest_event_total++;
		}
	}
	if (cur) {
		rest_total_s += dt;
	}
	prev_rest_detected = cur;
}

void vqf_update_accel(float *a, float time)
{
	float a_m_s2[3] = {0};
	// a is in g, convert to m/s^2
	for (int i = 0; i < 3; i++) {
		a_m_s2[i] = a[i] * CONST_EARTH_GRAVITY;
	}
	if (a_m_s2[0] != 0 || a_m_s2[1] != 0 || a_m_s2[2] != 0) {
		memcpy(last_a, a_m_s2, sizeof(a_m_s2));
	}
#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
	vqf_pre_accel_update(a_m_s2);
#endif
	if (time > 0.0f && time < 10.0f) {
		uint64_t synth_ts = state.lastAccTsUs + (uint64_t)(time * 1e6f);
		if (synth_ts == 0) {
			synth_ts = 1;
		}
		updateAccTs(&params, &state, &coeffs, a_m_s2, synth_ts);
	} else {
		updateAcc(&params, &state, &coeffs, a_m_s2);
	}
	vqf_track_rest_diag();
}

#if IS_ENABLED(CONFIG_VQF_MAG_SLEW_LIMIT)
static float vqf_wrap_pi(float angle)
{
	if (angle > (float)M_PI) {
		angle -= 2.0f * (float)M_PI;
	} else if (angle < -(float)M_PI) {
		angle += 2.0f * (float)M_PI;
	}
	return angle;
}

/**
 * @brief Clamp the heading change applied by the last mag update.
 *
 * Called after updateMag with the pre-update delta and the update time step.
 * Skipped during initial convergence (kMagInit active).
 */
static void vqf_apply_mag_slew_limit(float delta_before, float dt)
{
	if (state.kMagInit != 0.0f || dt <= 0.0f) {
		return;
	}

	float change = vqf_wrap_pi(state.delta - delta_before);
	float max_rate_dps = state.restDetected ? MAG_SLEW_MAX_RATE_REST_DPS : MAG_SLEW_MAX_RATE_MOTION_DPS;
	float max_change = max_rate_dps * DEG_TO_RAD * dt;

	if (change > max_change) {
		change = max_change;
	} else if (change < -max_change) {
		change = -max_change;
	} else {
		return; // within limit, keep the library result
	}

	state.delta = vqf_wrap_pi(delta_before + change);
	state.lastMagCorrAngularRate = change / dt;
}
#endif /* CONFIG_VQF_MAG_SLEW_LIMIT */

/* Pre/post-update processing shared by both mag entry points. */
/* Huber M-estimation tuning constants for continuous heading trust.
 * NORM_SIGMA and DIP_SIGMA are the expected clean-condition standard
 * deviations of calibrated field norm and dip angle - they are physical
 * quantities, not curve-shape constants.
 * Kconfig units: NORM_SIGMA and DIP_SIGMA in millis (150 = 0.15),
 * HUBER_DELTA in millisigma (2000 = 2.0), GRADIENT_TRUST in percent (80). */

static void vqf_post_mag_update(float delta_before, float dt, bool consist_disturbed)
{
	// Mahalanobis-like distance from expected clean-condition sigma
	float ref_norm = getMagRefNorm(&state);
	float ref_dip = getMagRefDip(&state);
	float norm_dev = state.magNormDip[0] - ref_norm;
	float dip_dev = state.magNormDip[1] - ref_dip;
	float nz = norm_dev / (CONFIG_VQF_MAG_NORM_SIGMA * 0.001f);  /* Kconfig: millis */
	float dz = dip_dev  / (CONFIG_VQF_MAG_DIP_SIGMA * 0.001f);
	float d = sqrtf(nz * nz + dz * dz);

	// Huber weight: quadratic (full trust) within delta, linear beyond
	float delta = CONFIG_VQF_MAG_HUBER_DELTA * 0.001f;  /* Kconfig: millisigma */
	float w_huber = (d <= delta) ? 1.0f : delta / d;

	// Gradient classifier override: cap even when d is small — a sample
	// can momentarily cross zero deviation by coincidence in a room the
	// classifier (via rotation-residual history) knows is bad.
	float trust = w_huber;
	if (grad_cls_gradient) {
		trust = fminf(w_huber, CONFIG_VQF_MAG_GRADIENT_TRUST * 0.01f);  /* Kconfig: percent */
	}

	// Transient override: full reject regardless of Huber weight
	if (grad_cls_transient) {
		trust = 0.0f;
	}

	// Gyro-mag consistency: at rest a direction mismatch is likely genuine
	// disturbance (full reject). In motion it is expected from spatial field
	// gradients — reduce trust but keep heading correction active.
#if IS_ENABLED(CONFIG_VQF_MAG_GYRO_CONSISTENCY)
	if (consist_disturbed && trust > 0.0f) {
		if (state.restDetected) {
			trust = 0.0f;
		} else {
			trust *= 0.5f;
		}
	}
#else
	ARG_UNUSED(consist_disturbed);
#endif

	float change = state.delta - delta_before;
	state.delta = delta_before + change * trust;
	state.lastMagCorrAngularRate = trust > 0.0f ? change * trust / dt : 0.0f;

#if IS_ENABLED(CONFIG_VQF_MAG_SLEW_LIMIT)
	vqf_apply_mag_slew_limit(delta_before, dt);
#else
	ARG_UNUSED(delta_before);
	ARG_UNUSED(dt);
#endif
}

static bool vqf_pre_mag_update(const float *m, float dt)
{
#if IS_ENABLED(CONFIG_VQF_MAG_GYRO_CONSISTENCY)
	return vqf_consist_check_mag(m, dt);
#else
	ARG_UNUSED(m);
	ARG_UNUSED(dt);
	return false;
#endif
}

void vqf_update_mag(float *m, float time)
{
	float delta_before = state.delta;

	// Use the caller-supplied time step when valid so that VQF time accumulators
	// (magCandidateT, magRejectT, etc.) and gain k run at the correct real-time
	// rate even when the sensor loop runs faster or slower than the mag ODR.
	//
	// Build a synthetic cumulative microsecond timestamp from the elapsed time so
	// updateMagTs can derive the correct dt internally (avoids calling the static
	// updateMag_internal directly).
	if (time > 0.0f && time < 10.0f) {
		bool consist_disturbed = vqf_pre_mag_update(m, time);
		uint64_t synth_ts = state.lastMagTsUs + (uint64_t)(time * 1e6f);
		if (synth_ts == 0) {
			synth_ts = 1; // avoid the "uninitialized" sentinel value
		}
		updateMagTs(&params, &state, &coeffs, m, synth_ts);
		vqf_post_mag_update(delta_before, time, consist_disturbed);
	} else {
		bool consist_disturbed = vqf_pre_mag_update(m, coeffs.magTs);
		updateMag(&params, &state, &coeffs, m);
		vqf_post_mag_update(delta_before, coeffs.magTs, consist_disturbed);
	}
}

void vqf_update_mag_ts(float *m, uint64_t timestamp_us)
{
	float delta_before = state.delta;
	// Mirror the library's dt derivation so pre/post-processing uses the same step.
	float dt = coeffs.magTs;
	if (state.lastMagTsUs != 0 && timestamp_us > state.lastMagTsUs) {
		uint64_t diff = timestamp_us - state.lastMagTsUs;
		if (diff > 0 && diff <= 10000000ULL) {
			dt = (float)diff / 1e6f;
		}
	}
	bool consist_disturbed = vqf_pre_mag_update(m, dt);
	updateMagTs(&params, &state, &coeffs, m, timestamp_us);
	vqf_post_mag_update(delta_before, dt, consist_disturbed);
}
void vqf_update(float *g, float *a, float *m, float time)
{
	// TODO: time unused?
	// TODO: gyro is a different rate to the others, should they be separated
	if (g[0] != 0 || g[1] != 0 || g[2] != 0) { // ignore zeroed gyro
		vqf_update_gyro(g, time);
	}
	vqf_update_accel(a, time);
	vqf_update_mag(m, time);
}

void vqf_get_gyro_bias(float *g_off)
{
	getBiasEstimate(&state, &coeffs, g_off);
	// VQF internal unit is rad/s, fusion interface expects deg/s
	for (int i = 0; i < 3; i++) {
		g_off[i] *= RAD_TO_DEG;
	}
}

void vqf_set_gyro_bias(float *g_off)
{
	float g_off_rad[3];
	// fusion interface receives values in deg/s, VQF requires rad/s
	for (int i = 0; i < 3; i++) {
		g_off_rad[i] = g_off[i] * DEG_TO_RAD;
	}
	setBiasEstimate(&state, g_off_rad, -1);
}

void vqf_update_gyro_sanity(float *g, float *m)
{
	// TODO: does vqf tell us a "recovery state"
}

int vqf_get_gyro_sanity(void)
{
	// TODO: does vqf tell us a "recovery state"
	return 0;
}

void vqf_get_lin_a(float *lin_a)
{
	float q[4] = {0};
	vqf_get_quat(q);

	float vec_gravity[3] = {0};
	vec_gravity[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
	vec_gravity[1] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
	vec_gravity[2] = 2.0f * (q[0] * q[0] - 0.5f + q[3] * q[3]);

	//	float *a = state.lastAccLp; // not usable, rotated by inertial frame
	float *a = last_a;
	for (int i = 0; i < 3; i++) {
		lin_a[i] = a[i] - vec_gravity[i] * CONST_EARTH_GRAVITY; // gravity vector to m/s^2 before subtracting
	}
}

void vqf_get_quat(float *q)
{
	getQuat9D(&state, q);
}

bool vqf_get_rest_detected(void)
{
	return getRestDetected(&state);
}

/* ---- Gradient-vs-transient classifier ---- */
/* Kconfig values: TH_LOW in millis (20 = 0.02), TAU in ms (500 = 0.5 s) */

void vqf_grad_classifier_sample(const float m[3], float mag_dt)
{
	float eff_dt = grad_cls.omega_dt > 0.0f ? grad_cls.omega_dt : mag_dt;
	float inv_eff_dt = eff_dt > 1e-9f ? 1.0f / eff_dt : 0.0f;
	float omega_avg[3] = {
		grad_cls.omega_sum[0] * inv_eff_dt,
		grad_cls.omega_sum[1] * inv_eff_dt,
		grad_cls.omega_sum[2] * inv_eff_dt,
	};
	memset(grad_cls.omega_sum, 0, sizeof(grad_cls.omega_sum));
	grad_cls.omega_dt = 0;

	if (grad_cls.have_prev) {
		float dmdt[3] = {
			(m[0] - grad_cls.prev_m[0]) * inv_eff_dt,
			(m[1] - grad_cls.prev_m[1]) * inv_eff_dt,
			(m[2] - grad_cls.prev_m[2]) * inv_eff_dt,
		};
		float r[3];
		r[0] = dmdt[0] + omega_avg[1] * m[2] - omega_avg[2] * m[1];
		r[1] = dmdt[1] + omega_avg[2] * m[0] - omega_avg[0] * m[2];
		r[2] = dmdt[2] + omega_avg[0] * m[1] - omega_avg[1] * m[0];
		float r_norm = sqrtf(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);

		float tau = CONFIG_VQF_MAG_GRAD_CLS_TAU * 0.001f; /* Kconfig: ms -> s */
		float alpha = (tau > 0.0f && eff_dt > 0.0f) ? eff_dt / (eff_dt + tau) : 1.0f;
		if (grad_cls.init) {
			grad_cls.ema += alpha * (r_norm - grad_cls.ema);
		} else {
			grad_cls.ema = r_norm;
			grad_cls.init = true;
		}

		if (grad_cls.ema < CONFIG_VQF_MAG_GRAD_CLS_TH_LOW * 0.001f) { /* Kconfig: millis -> float */
			grad_cls_gradient = false;
			grad_cls_transient = false;
		} else if (state.restDetected) {
			grad_cls_transient = true;
			grad_cls_gradient = false;
		} else {
			grad_cls_gradient = true;
			grad_cls_transient = false;
		}
	} else {
		grad_cls_gradient = false;
		grad_cls_transient = false;
	}

	memcpy(grad_cls.prev_m, m, sizeof(grad_cls.prev_m));
	grad_cls.have_prev = true;
}

bool vqf_get_grad_cls_gradient(void)
{
	return grad_cls_gradient;
}

bool vqf_get_grad_cls_transient(void)
{
	return grad_cls_transient;
}

bool vqf_get_mag_dist_detected(void)
{
	bool dist = getMagDistDetected(&state);
#if IS_ENABLED(CONFIG_VQF_MAG_GYRO_CONSISTENCY)
	dist = dist || consist_hold_t > 0.0f;
#endif

	if (dist) {
		dist_continuous_s += coeffs.magTs;
		if (dist_continuous_s > CONFIG_VQF_MAG_CONT_DIST_THRESH_S) {
			return false;
		}
	} else {
		dist_continuous_s = 0;
	}
	return dist;
}

void vqf_reset_mag_ref(void)
{
	setMagRef(&state, 0, 0);
	// Calibration changed: window-start mag direction is no longer comparable
	vqf_consist_reset();
}

void vqf_set_mag_ref(float norm, float dip)
{
	setMagRef(&state, norm, dip);
	vqf_consist_reset();
}

float vqf_get_mag_ref_norm(void)
{
	return getMagRefNorm(&state);
}

void vqf_get_mag_ref(float *norm, float *dip)
{
	*norm = getMagRefNorm(&state);
	*dip = getMagRefDip(&state);
}

float vqf_get_delta(void)
{
	return getDelta(&state);
}

void vqf_set_delta(float delta)
{
	state.delta = delta;
}

void vqf_get_relative_rest_deviations(float *out)
{
	getRelativeRestDeviations(&params, &state, out);
}

void vqf_get_debug_info(vqf_debug_info_t *info)
{
	if (!info) {
		return;
	}

	info->rest_detected = getRestDetected(&state);
	getRelativeRestDeviations(&params, &state, info->rest_deviations);
	info->bias_sigma = getBiasEstimate(&state, &coeffs, info->bias);

	// Heading correction state
	info->delta = getDelta(&state);

	// Magnetic disturbance / reference (includes gyro-mag consistency check)
	info->mag_dist_detected = vqf_get_mag_dist_detected();
	info->mag_ref_norm = getMagRefNorm(&state);
	info->mag_ref_dip = getMagRefDip(&state);

	// Current magnetic field (after optional magCurrentTau LPF)
	info->mag_norm = state.magNormDip[0];
	info->mag_dip = state.magNormDip[1];

	// Heading correction diagnostics (from last magnetometer update)
	info->mag_dis_angle = state.lastMagDisAngle;
	info->mag_corr_rate = state.lastMagCorrAngularRate;

	// Disturbance rejection timers
	info->mag_undisturbed_t = state.magUndisturbedT;
	info->mag_reject_t = state.magRejectT;

	// Candidate field tracking
	info->mag_candidate_norm = state.magCandidateNorm;
	info->mag_candidate_dip = state.magCandidateDip;
	info->mag_candidate_t = state.magCandidateT;

	// Filter gains
	info->mag_k = coeffs.kMag;
	info->mag_k_init = state.kMagInit;

	// Convert bias from rad/s to °/s
	for (int i = 0; i < 3; i++) {
		info->bias[i] *= RAD_TO_DEG;
	}
	info->bias_sigma *= RAD_TO_DEG;

	// Convert rad-based angles to degrees
	info->delta *= RAD_TO_DEG;
	info->mag_ref_dip *= RAD_TO_DEG;
	info->mag_dip *= RAD_TO_DEG;
	info->mag_dis_angle *= RAD_TO_DEG;

	// Convert angular rates from rad/s to °/s
	info->mag_corr_rate *= RAD_TO_DEG;

	// Convert candidate dip from rad to degrees
	info->mag_candidate_dip *= RAD_TO_DEG;

#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
	// Adaptive tauAcc state
	info->tau_acc = params.tauAcc;
	info->motion_intensity = motion_intensity;
#endif

	// Rest detection diagnostics
	info->rest_enter_count = rest_enter_count;
	info->rest_exit_count = rest_exit_count;
	info->rest_total_s = rest_total_s;
	info->rest_last_duration_s = rest_last_duration_s;
	info->uptime_s = uptime_s;

	// Copy rest event log (oldest first)
	info->rest_event_count = rest_event_total;
	for (uint8_t i = 0; i < REST_EVENT_LOG_SIZE; i++) {
		uint8_t src;
		if (rest_event_total >= REST_EVENT_LOG_SIZE) {
			src = (rest_event_idx + i) % REST_EVENT_LOG_SIZE;
		} else {
			src = i;
		}
		info->rest_events[i].time_s = rest_event_log[src].time_s;
		info->rest_events[i].entered = rest_event_log[src].entered;
	}

	// Kalman filter P diagonal
	info->biasP[0] = state.biasP[0];
	info->biasP[1] = state.biasP[4];
	info->biasP[2] = state.biasP[8];
}

#if defined(CONFIG_VQF_BENCH)
static ALWAYS_INLINE void vqf_bench_timer_prepare(void)
{
#if defined(CONFIG_CPU_CORTEX_M_HAS_DWT)
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
}

static ALWAYS_INLINE uint32_t vqf_bench_timer_begin(unsigned int *irq_key)
{
	*irq_key = irq_lock();
#if defined(CONFIG_CPU_CORTEX_M_HAS_DWT)
	DWT->CYCCNT = 0;
	return 0;
#else
	return k_cycle_get_32();
#endif
}

static ALWAYS_INLINE uint32_t vqf_bench_timer_end(unsigned int irq_key, uint32_t start_cycles)
{
#if defined(CONFIG_CPU_CORTEX_M_HAS_DWT)
	uint32_t elapsed_cycles = DWT->CYCCNT;
	irq_unlock(irq_key);
	return elapsed_cycles;
#else
	uint32_t elapsed_cycles = k_cycle_get_32() - start_cycles;
	irq_unlock(irq_key);
	return elapsed_cycles;
#endif
}

static ALWAYS_INLINE uint32_t vqf_bench_timer_hz(void)
{
#if defined(CONFIG_CPU_CORTEX_M_HAS_DWT)
	return SystemCoreClock ? SystemCoreClock : CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;
#else
	return CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;
#endif
}

static void vqf_bench_print_stats(const char *name, uint32_t iterations, uint32_t total_cycles, uint32_t timer_hz)
{
	uint32_t avg_cycles_int = 0;
	uint32_t avg_cycles_frac = 0;
	uint32_t total_us_int = 0;
	uint32_t total_us_frac = 0;
	uint32_t avg_us_int = 0;
	uint32_t avg_us_frac = 0;

	if (iterations) {
		uint64_t avg_cycles_x1000 = (total_cycles * 1000ULL) / iterations;
		avg_cycles_int = (uint32_t)(avg_cycles_x1000 / 1000ULL);
		avg_cycles_frac = (uint32_t)(avg_cycles_x1000 % 1000ULL);
	}

	if (timer_hz) {
		uint64_t total_us_x1000 = (total_cycles * 1000000000ULL) / timer_hz;
		total_us_int = (uint32_t)(total_us_x1000 / 1000ULL);
		total_us_frac = (uint32_t)(total_us_x1000 % 1000ULL);

		if (iterations) {
			uint64_t avg_us_x1000 = total_us_x1000 / iterations;
			avg_us_int = (uint32_t)(avg_us_x1000 / 1000ULL);
			avg_us_frac = (uint32_t)(avg_us_x1000 % 1000ULL);
		}
	}

	printk(
		"  %-10s t_c=%10u avg_c=%4u.%03u t_us=%8u.%03u avg_us=%4u.%03u\n",
		name,
		total_cycles,
		avg_cycles_int,
		avg_cycles_frac,
		total_us_int,
		total_us_frac,
		avg_us_int,
		avg_us_frac
	);
}

#define VQF_BENCH_BATCH_SIZE 16U

typedef enum {
	VQF_BENCH_UPDATE_GYR,
	VQF_BENCH_UPDATE_ACC,
	VQF_BENCH_UPDATE_MAG,
	VQF_BENCH_GET_QUAT9D,
} vqf_bench_op_t;

static uint32_t vqf_bench_measure(
	vqf_bench_op_t op,
	uint32_t iterations,
	const vqf_real_t gyr_samples[][3],
	const vqf_real_t acc_samples[][3],
	const vqf_real_t mag_samples[][3],
	size_t sample_count,
	vqf_params_t *bench_params,
	vqf_state_t *bench_state,
	vqf_coeffs_t *bench_coeffs,
	float quat[4]
)
{
	uint32_t total_cycles = 0;
	uint32_t remaining = iterations;
	uint32_t sample_offset = 0;

	while (remaining > 0) {
		uint32_t batch_len = remaining > VQF_BENCH_BATCH_SIZE ? VQF_BENCH_BATCH_SIZE : remaining;
		unsigned int irq_key;
		uint32_t start_cycles = vqf_bench_timer_begin(&irq_key);

		for (uint32_t i = 0; i < batch_len; i++, sample_offset++) {
			size_t sample_idx = sample_offset % sample_count;

			switch (op) {
			case VQF_BENCH_UPDATE_GYR:
				updateGyr(bench_params, bench_state, bench_coeffs, gyr_samples[sample_idx]);
				break;
			case VQF_BENCH_UPDATE_ACC:
				updateAcc(bench_params, bench_state, bench_coeffs, acc_samples[sample_idx]);
				break;
			case VQF_BENCH_UPDATE_MAG:
				updateMag(bench_params, bench_state, bench_coeffs, mag_samples[sample_idx]);
				break;
			case VQF_BENCH_GET_QUAT9D:
				getQuat9D(bench_state, quat);
				vqf_bench_sink += quat[sample_offset & 3U];
				break;
			}
		}

		total_cycles += vqf_bench_timer_end(irq_key, start_cycles);
		remaining -= batch_len;
	}

	return total_cycles;
}

void vqf_run_benchmark(uint32_t iterations)
{
	vqf_bench_sink = 0.0f;
	uint32_t timer_hz;
	uint32_t elapsed_cycles;
	k_tid_t bench_thread = k_current_get();
	int bench_thread_prio = k_thread_priority_get(bench_thread);
	bool bench_prio_changed = false;

	if (iterations == 0) {
		iterations = 1000;
	}

	if (bench_thread_prio < VQF_BENCH_THREAD_PRIORITY) {
		k_thread_priority_set(bench_thread, VQF_BENCH_THREAD_PRIORITY);
		bench_prio_changed = true;
	}

	set_params();
	vqf_bench_params = params;
	initVqf(&vqf_bench_params, &vqf_bench_state, &vqf_bench_coeffs, 0.001f, 0.001f, 0.01f);

	for (size_t i = 0; i < vqf_bench_sample_count * 8; i++) {
		const size_t idx = i % vqf_bench_sample_count;
		updateGyr(&vqf_bench_params, &vqf_bench_state, &vqf_bench_coeffs, vqf_bench_gyr_samples[idx]);
		updateAcc(&vqf_bench_params, &vqf_bench_state, &vqf_bench_coeffs, vqf_bench_acc_samples[idx]);
		updateMag(&vqf_bench_params, &vqf_bench_state, &vqf_bench_coeffs, vqf_bench_mag_samples[idx]);
		getQuat9D(&vqf_bench_state, vqf_bench_quat);
		vqf_bench_sink += vqf_bench_quat[0];
	}

	vqf_bench_warm_params = vqf_bench_params;
	vqf_bench_warm_state = vqf_bench_state;
	vqf_bench_warm_coeffs = vqf_bench_coeffs;
	vqf_bench_timer_prepare();
	timer_hz = vqf_bench_timer_hz();

	printk(
		"VQF benchmark (%u iterations, CMSIS-DSP=%s, timer=%s)\n",
		iterations,
#ifdef CONFIG_CMSIS_DSP
		"on"
#else
		"off"
#endif
		,
#if defined(CONFIG_CPU_CORTEX_M_HAS_DWT)
		"DWT CYCCNT"
#else
		"system timer"
#endif
	);

	vqf_bench_params = vqf_bench_warm_params;
	vqf_bench_state = vqf_bench_warm_state;
	vqf_bench_coeffs = vqf_bench_warm_coeffs;
	elapsed_cycles = vqf_bench_measure(
		VQF_BENCH_UPDATE_GYR,
		iterations,
		vqf_bench_gyr_samples,
		vqf_bench_acc_samples,
		vqf_bench_mag_samples,
		vqf_bench_sample_count,
		&vqf_bench_params,
		&vqf_bench_state,
		&vqf_bench_coeffs,
		vqf_bench_quat
	);
	vqf_bench_print_stats("updateGyr", iterations, elapsed_cycles, timer_hz);

	vqf_bench_params = vqf_bench_warm_params;
	vqf_bench_state = vqf_bench_warm_state;
	vqf_bench_coeffs = vqf_bench_warm_coeffs;
	elapsed_cycles = vqf_bench_measure(
		VQF_BENCH_UPDATE_ACC,
		iterations,
		vqf_bench_gyr_samples,
		vqf_bench_acc_samples,
		vqf_bench_mag_samples,
		vqf_bench_sample_count,
		&vqf_bench_params,
		&vqf_bench_state,
		&vqf_bench_coeffs,
		vqf_bench_quat
	);
	vqf_bench_print_stats("updateAcc", iterations, elapsed_cycles, timer_hz);

	vqf_bench_params = vqf_bench_warm_params;
	vqf_bench_state = vqf_bench_warm_state;
	vqf_bench_coeffs = vqf_bench_warm_coeffs;
	elapsed_cycles = vqf_bench_measure(
		VQF_BENCH_UPDATE_MAG,
		iterations,
		vqf_bench_gyr_samples,
		vqf_bench_acc_samples,
		vqf_bench_mag_samples,
		vqf_bench_sample_count,
		&vqf_bench_params,
		&vqf_bench_state,
		&vqf_bench_coeffs,
		vqf_bench_quat
	);
	vqf_bench_print_stats("updateMag", iterations, elapsed_cycles, timer_hz);

	vqf_bench_params = vqf_bench_warm_params;
	vqf_bench_state = vqf_bench_warm_state;
	vqf_bench_coeffs = vqf_bench_warm_coeffs;
	elapsed_cycles = vqf_bench_measure(
		VQF_BENCH_GET_QUAT9D,
		iterations,
		vqf_bench_gyr_samples,
		vqf_bench_acc_samples,
		vqf_bench_mag_samples,
		vqf_bench_sample_count,
		&vqf_bench_params,
		&vqf_bench_state,
		&vqf_bench_coeffs,
		vqf_bench_quat
	);
	vqf_bench_print_stats("getQuat9D", iterations, elapsed_cycles, timer_hz);

	printk("  checksum: %.6f\n", (double)vqf_bench_sink);

	if (bench_prio_changed) {
		k_thread_priority_set(bench_thread, bench_thread_prio);
	}
}
#endif

const sensor_fusion_t sensor_fusion_vqf = {
	.init = vqf_init,
	.load = vqf_load,
	.save = vqf_save,

	.update_gyro = vqf_update_gyro,
	.update_accel = vqf_update_accel,
	.update_mag = vqf_update_mag,
	.update = vqf_update,

	.get_gyro_bias = vqf_get_gyro_bias,
	.set_gyro_bias = vqf_set_gyro_bias,

	.update_gyro_sanity = vqf_update_gyro_sanity,
	.get_gyro_sanity = vqf_get_gyro_sanity,

	.get_lin_a = vqf_get_lin_a,
	.get_quat = vqf_get_quat,

	.get_rest_detected = vqf_get_rest_detected,
	.get_relative_rest_deviations = vqf_get_relative_rest_deviations,
	.get_mag_dist_detected = vqf_get_mag_dist_detected,
	.reset_mag_ref = vqf_reset_mag_ref,
	.set_mag_ref = vqf_set_mag_ref,
	.get_mag_ref = vqf_get_mag_ref,
};
