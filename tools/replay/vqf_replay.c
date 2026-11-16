/*
 * Offline VQF replay for raw collections (receiver scripts/data_collect_hid.py).
 *
 * Runs the vqf-c core with the firmware's parameter set on a recorded
 * <name>.csv + <name>.meta.txt pair and prints drift / mag-rejection metrics,
 * so fusion parameters can be compared on identical data.
 *
 * Build:  make (see Makefile)
 * Usage:  vqf_replay [--param name=value]... [--no-mag] [--skip s] [--window s] [--out trace.csv] rec.csv [rec.meta.txt]
 *         Yaw change is measured between the window starting at --skip and the final window.
 *
 * Differences from firmware (deliberate, keep in mind when reading results):
 * - core vqf-c only: the firmware wrapper's extra rest/bias policies in
 *   src/sensor/fusion/vqf/vqf.c are not applied;
 * - static calibration from the .meta.txt snapshot (gyro bias + sensitivity,
 *   accel/mag BAinv); T-Cal and online mag recalibration are not replayed;
 * - mag axes are used as recorded (SENSOR_MAGNETOMETER_AXES_ALIGNMENT default).
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vqf.h"

#define DEG2RAD 0.017453292519943295
#define RAD2DEG 57.29577951308232
#define GRAVITY 9.80665

static vqf_params_t params;
static vqf_state_t state;
static vqf_coeffs_t coeffs;

/* Keep in sync with set_params() in src/sensor/fusion/vqf/vqf.c. */
static void firmware_params(void)
{
	init_params(&params);
	params.tauAcc = 3.294f;
	params.biasClip = 5.0f;
	params.biasForgettingTime = 235.58f;
	params.biasSigmaInit = 0.6585f;
	params.biasSigmaMotion = 0.1118f;
	params.biasSigmaRest = 0.0028f;
	params.biasVerticalForgettingFactor = 0.0003f;
	params.motionBiasEstEnabled = true;
	params.restBiasEstEnabled = true;
	params.restFilterTau = 2.278f;
	params.restMinT = 0.118f;
	params.restThGyr = 0.486f;
	params.restThAcc = 0.186f;
	params.magDistRejectionEnabled = true;
	params.tauMag = 9.0f;
	params.magCurrentTau = 0.20f;
	params.magNormTh = 0.10f;
	params.magDipTh = 5.0f;
	params.magRefTau = 10.0f;
	params.magNewTime = 15.0f;
	params.magNewFirstTime = 5.0f;
	params.magNewMinGyr = 10.0f;
	params.magMinUndisturbedTime = 0.5f;
	params.magMaxRejectionTime = 60.0f;
	params.magRejectionFactor = 1000.0f;
}

static int set_param(const char *name, double v)
{
	static const struct {
		const char *name;
		float *f;
		bool *b;
	} table[] = {
#define F(n) {#n, &params.n, NULL}
#define B(n) {#n, NULL, &params.n}
		F(tauAcc), F(biasClip), F(biasForgettingTime), F(biasSigmaInit), F(biasSigmaMotion),
		F(biasSigmaRest), F(biasVerticalForgettingFactor), B(motionBiasEstEnabled),
		B(restBiasEstEnabled), F(restFilterTau), F(restMinT), F(restThGyr), F(restThAcc),
		B(magDistRejectionEnabled), F(tauMag), F(magCurrentTau), F(magNormTh), F(magDipTh),
		F(magRefTau), F(magNewTime), F(magNewFirstTime), F(magNewMinGyr),
		F(magMinUndisturbedTime), F(magMaxRejectionTime), F(magRejectionFactor),
#undef F
#undef B
	};
	for (size_t i = 0; i < sizeof(table) / sizeof(table[0]); i++) {
		if (strcmp(name, table[i].name) == 0) {
			if (table[i].f) {
				*table[i].f = (float)v;
			} else {
				*table[i].b = v != 0;
			}
			return 0;
		}
	}
	return -1;
}

struct meta {
	double gyro_hz, mag_hz;
	double gyro_bias[3], gyro_scale[3];
	double acc_bainv[12], mag_bainv[12];
	bool has_acc, has_mag;
};

static int parse_list(const char *s, double *out, int n)
{
	for (int i = 0; i < n; i++) {
		char *end;
		out[i] = strtod(s, &end);
		if (end == s) {
			return -1;
		}
		s = (*end == ',') ? end + 1 : end;
	}
	return 0;
}

static void read_meta(const char *path, struct meta *m)
{
	*m = (struct meta){.gyro_scale = {1, 1, 1}};
	FILE *f = fopen(path, "r");
	if (!f) {
		fprintf(stderr, "warning: no metadata %s, assuming 400 Hz gyro, 50 Hz mag, no calibration\n", path);
		m->gyro_hz = 400;
		m->mag_hz = 50;
		return;
	}
	char line[4096];
	while (fgets(line, sizeof(line), f)) {
		char *eq = strchr(line, '=');
		if (!eq || line[0] == '#') {
			continue;
		}
		*eq = 0;
		const char *k = line, *v = eq + 1;
		if (!strcmp(k, "gyro_odr_hz")) m->gyro_hz = atof(v);
		else if (!strcmp(k, "mag_odr_hz")) m->mag_hz = atof(v);
		else if (!strcmp(k, "gyro_bias")) parse_list(v, m->gyro_bias, 3);
		else if (!strcmp(k, "gyro_sens_scale")) parse_list(v, m->gyro_scale, 3);
		else if (!strcmp(k, "acc_cal_bias")) m->has_acc = !parse_list(v, m->acc_bainv, 3);
		else if (!strcmp(k, "acc_cal_matrix")) m->has_acc &= !parse_list(v, m->acc_bainv + 3, 9);
		else if (!strcmp(k, "mag_cal_bias")) m->has_mag = !parse_list(v, m->mag_bainv, 3);
		else if (!strcmp(k, "mag_cal_matrix")) m->has_mag &= !parse_list(v, m->mag_bainv + 3, 9);
	}
	fclose(f);
	if (m->gyro_hz <= 0) {
		fprintf(stderr, "error: gyro_odr_hz missing in %s\n", path);
		exit(2);
	}
	if (m->mag_hz <= 0) {
		m->mag_hz = 50;
	}
}

/* Same convention as apply_BAinv() in src/util.c: out = A * (v - b). */
static void apply_bainv(const double *bainv, const double v[3], vqf_real_t out[3])
{
	double t[3] = {v[0] - bainv[0], v[1] - bainv[1], v[2] - bainv[2]};
	for (int r = 0; r < 3; r++) {
		const double *row = bainv + 3 + 3 * r;
		out[r] = (vqf_real_t)(row[0] * t[0] + row[1] * t[1] + row[2] * t[2]);
	}
}

static double yaw_of(const vqf_real_t q[4])
{
	return atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
}

static double wrap(double a)
{
	while (a > M_PI) a -= 2 * M_PI;
	while (a < -M_PI) a += 2 * M_PI;
	return a;
}

/* Circular mean accumulators for the start / end windows. */
struct circ {
	double s, c;
};

static void circ_add(struct circ *a, double ang)
{
	a->s += sin(ang);
	a->c += cos(ang);
}

static double circ_mean(const struct circ *a)
{
	return atan2(a->s, a->c);
}

int main(int argc, char **argv)
{
	firmware_params();
	const char *csv_path = NULL, *meta_path = NULL, *out_path = NULL;
	bool use_mag = true;
	double window_s = 2.0, skip_s = 0.0;

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--param") && i + 1 < argc) {
			char *eq = strchr(argv[++i], '=');
			if (!eq) {
				fprintf(stderr, "error: --param expects name=value\n");
				return 2;
			}
			*eq = 0;
			if (set_param(argv[i], atof(eq + 1))) {
				fprintf(stderr, "error: unknown parameter %s\n", argv[i]);
				return 2;
			}
		} else if (!strcmp(argv[i], "--no-mag")) {
			use_mag = false;
		} else if (!strcmp(argv[i], "--skip") && i + 1 < argc) {
			skip_s = atof(argv[++i]);
		} else if (!strcmp(argv[i], "--window") && i + 1 < argc) {
			window_s = atof(argv[++i]);
		} else if (!strcmp(argv[i], "--out") && i + 1 < argc) {
			out_path = argv[++i];
		} else if (!csv_path) {
			csv_path = argv[i];
		} else {
			meta_path = argv[i];
		}
	}
	if (!csv_path) {
		fprintf(stderr, "usage: %s [--param name=value]... [--no-mag] [--skip s] [--window s] [--out trace.csv] rec.csv [rec.meta.txt]\n", argv[0]);
		return 2;
	}

	char derived_meta[4096];
	if (!meta_path) {
		snprintf(derived_meta, sizeof(derived_meta), "%s", csv_path);
		char *dot = strrchr(derived_meta, '.');
		if (dot) {
			*dot = 0;
		}
		strncat(derived_meta, ".meta.txt", sizeof(derived_meta) - strlen(derived_meta) - 1);
		meta_path = derived_meta;
	}
	struct meta m;
	read_meta(meta_path, &m);

	FILE *in = fopen(csv_path, "r");
	if (!in) {
		perror(csv_path);
		return 2;
	}
	FILE *out = out_path ? fopen(out_path, "w") : NULL;
	if (out) {
		fprintf(out, "t,qw,qx,qy,qz,yaw6_deg,yaw9_deg,delta_deg,mag_dist,rest\n");
	}

	double ts = 1.0 / m.gyro_hz;
	initVqf(&params, &state, &coeffs, (vqf_real_t)ts, (vqf_real_t)ts, (vqf_real_t)(1.0 / m.mag_hz));

	char line[1024];
	if (!fgets(line, sizeof(line), in) || strncmp(line, "seq,gx", 6) != 0) {
		fprintf(stderr, "error: expected raw-mode CSV header 'seq,gx,...' (gyr_quat captures are not supported)\n");
		return 2;
	}

	long n = 0, mag_n = 0, mag_dist_n = 0, rest_n = 0, discont_n = 0;
	double dist_run = 0, dist_run_max = 0;
	double t_end;
	/* Two passes would need the duration up front; instead keep a ring of end-window yaws. */
	long win_n = (long)(window_s * m.gyro_hz);
	if (win_n < 1) win_n = 1;
	long skip_n = (long)(skip_s * m.gyro_hz);
	double *ring6 = calloc((size_t)win_n, sizeof(double));
	double *ring9 = calloc((size_t)win_n, sizeof(double));
	struct circ start6 = {0}, start9 = {0};

	while (fgets(line, sizeof(line), in)) {
		double v[12];
		if (parse_list(line, v, 12)) {
			continue;
		}
		double g[3] = {v[1], v[2], v[3]}, a[3] = {v[4], v[5], v[6]}, mg[3] = {v[7], v[8], v[9]};
		if (v[11] != 0) {
			discont_n++;
		}

		vqf_real_t gr[3], ar[3], mr[3];
		for (int i = 0; i < 3; i++) {
			gr[i] = (vqf_real_t)((g[i] - m.gyro_bias[i]) * m.gyro_scale[i] * DEG2RAD);
		}
		if (m.has_acc) {
			apply_bainv(m.acc_bainv, a, ar);
		} else {
			for (int i = 0; i < 3; i++) ar[i] = (vqf_real_t)a[i];
		}
		for (int i = 0; i < 3; i++) ar[i] = (vqf_real_t)(ar[i] * GRAVITY);

		updateGyr(&params, &state, &coeffs, gr);
		updateAcc(&params, &state, &coeffs, ar);
		vqf_real_t q6[4], q9[4];
		getQuat6D(&state, q6);

		bool has_mag = mg[0] != 0 || mg[1] != 0 || mg[2] != 0;
		if (use_mag && has_mag) {
			if (m.has_mag) {
				apply_bainv(m.mag_bainv, mg, mr);
			} else {
				for (int i = 0; i < 3; i++) mr[i] = (vqf_real_t)mg[i];
			}
			updateMag(&params, &state, &coeffs, mr);
			mag_n++;
			if (getMagDistDetected(&state)) {
				mag_dist_n++;
				dist_run += 1.0 / m.mag_hz;
				if (dist_run > dist_run_max) dist_run_max = dist_run;
			} else {
				dist_run = 0;
			}
		}
		bool rest = getRestDetected(&state);
		rest_n += rest;

		getQuat9D(&state, q9);
		double y6 = yaw_of(q6), y9 = yaw_of(q9);
		if (n >= skip_n && n < skip_n + win_n) {
			circ_add(&start6, y6);
			circ_add(&start9, y9);
		}
		ring6[n % win_n] = y6;
		ring9[n % win_n] = y9;

		double t = (double)n * ts;
		if (out && n % (long)fmax(1, m.gyro_hz / 50) == 0) {
			fprintf(out, "%.4f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f,%d,%d\n", t, q9[0], q9[1], q9[2], q9[3],
				y6 * RAD2DEG, y9 * RAD2DEG, getDelta(&state) * RAD2DEG, getMagDistDetected(&state), rest);
		}
		n++;
	}
	fclose(in);
	if (out) fclose(out);
	if (n < skip_n + 2 * win_n) {
		fprintf(stderr, "error: recording shorter than skip + two windows (%ld samples)\n", n);
		return 2;
	}
	t_end = (double)n * ts;

	struct circ end6 = {0}, end9 = {0};
	for (long i = 0; i < win_n; i++) {
		circ_add(&end6, ring6[i]);
		circ_add(&end9, ring9[i]);
	}
	vqf_real_t bias[3];
	getBiasEstimate(&state, &coeffs, bias);

	printf("duration_s=%.1f\n", t_end);
	printf("samples=%ld discontinuities=%ld\n", n, discont_n);
	printf("mag_updates=%ld mag_disturbed_pct=%.1f longest_disturbed_s=%.1f\n", mag_n,
	       mag_n ? 100.0 * (double)mag_dist_n / (double)mag_n : 0.0, dist_run_max);
	printf("rest_pct=%.1f\n", 100.0 * (double)rest_n / (double)n);
	printf("yaw6_change_deg=%.2f\n", wrap(circ_mean(&end6) - circ_mean(&start6)) * RAD2DEG);
	printf("yaw9_change_deg=%.2f\n", wrap(circ_mean(&end9) - circ_mean(&start9)) * RAD2DEG);
	printf("bias_final_dps=%.4f,%.4f,%.4f\n", bias[0] * RAD2DEG, bias[1] * RAD2DEG, bias[2] * RAD2DEG);
	free(ring6);
	free(ring9);
	return 0;
}
