/* Host side (native simulator runner, host libc) of slime_sim.h.
 * Env: SLIME_SIM_CSV  recording to feed (tools/replay/sim.py output)
 *      SLIME_SIM_OUT  quaternion log (default slime_sim_quat.csv) */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct row {
	float g[3], a[3], m[3];
	int has_mag;
};

static struct row *rows;
static uint32_t nrows;
static float rate_hz = 400.0f;
static FILE *out;
static int loaded;

static void load(void)
{
	loaded = 1;
	const char *path = getenv("SLIME_SIM_CSV");
	if (!path) {
		fprintf(stderr, "slime_sim: SLIME_SIM_CSV not set, IMU emulator returns no data\n");
		return;
	}
	FILE *f = fopen(path, "r");
	if (!f) {
		perror(path);
		return;
	}
	char meta[4096];
	snprintf(meta, sizeof(meta), "%s", path);
	char *dot = strrchr(meta, '.');
	if (dot) {
		strcpy(dot, ".meta.txt");
		FILE *mf = fopen(meta, "r");
		char line[256];
		while (mf && fgets(line, sizeof(line), mf)) {
			if (!strncmp(line, "gyro_odr_hz=", 12)) {
				rate_hz = (float)atof(line + 12);
			}
		}
		if (mf) {
			fclose(mf);
		}
	}
	char line[512];
	uint32_t cap = 0;
	if (!fgets(line, sizeof(line), f)) { /* header */
		fclose(f);
		return;
	}
	while (fgets(line, sizeof(line), f)) {
		double v[12];
		if (sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &v[0], &v[1], &v[2], &v[3], &v[4],
			   &v[5], &v[6], &v[7], &v[8], &v[9], &v[10], &v[11]) != 12) {
			continue;
		}
		if (nrows == cap) {
			cap = cap ? cap * 2 : 65536;
			rows = realloc(rows, cap * sizeof(*rows));
		}
		struct row *r = &rows[nrows++];
		for (int i = 0; i < 3; i++) {
			r->g[i] = (float)v[1 + i];
			r->a[i] = (float)v[4 + i];
			r->m[i] = (float)v[7 + i];
		}
		r->has_mag = v[7] != 0 || v[8] != 0 || v[9] != 0;
	}
	fclose(f);
	fprintf(stderr, "slime_sim: %u samples at %.0f Hz from %s\n", nrows, (double)rate_hz, path);
}

int slime_sim_sample(uint32_t idx, float g[3], float a[3], float m[3], int *has_mag)
{
	if (!loaded) {
		load();
	}
	if (idx >= nrows) {
		return -1;
	}
	memcpy(g, rows[idx].g, sizeof(rows[idx].g));
	memcpy(a, rows[idx].a, sizeof(rows[idx].a));
	memcpy(m, rows[idx].m, sizeof(rows[idx].m));
	*has_mag = rows[idx].has_mag;
	return 0;
}

float slime_sim_rate_hz(void)
{
	if (!loaded) {
		load();
	}
	return rate_hz;
}

int slime_sim_mag_requested(void)
{
	const char *v = getenv("SLIME_SIM_MAG");
	return !(v && v[0] == '0');
}

int slime_sim_magcal_requested(void)
{
	const char *v = getenv("SLIME_SIM_MAGCAL");
	return !(v && v[0] == '0');
}

/* Firmware runs in zero simulated time; charge this per FIFO sample read (fusion cost). */
int slime_sim_cpu_us_per_sample(void)
{
	const char *v = getenv("SLIME_SIM_CPU_US");
	return v ? atoi(v) : 50;
}

void slime_sim_log_quat(uint64_t t_us, const float q[4])
{
	if (!out) {
		const char *path = getenv("SLIME_SIM_OUT");
		out = fopen(path ? path : "slime_sim_quat.csv", "w");
		if (!out) {
			return;
		}
		fprintf(out, "t_us,qw,qx,qy,qz\n");
	}
	fprintf(out, "%llu,%.6f,%.6f,%.6f,%.6f\n", (unsigned long long)t_us, (double)q[0], (double)q[1], (double)q[2],
		(double)q[3]);
}
