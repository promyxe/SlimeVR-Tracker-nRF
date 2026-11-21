#include "mag_offset_track.h"

#include <math.h>
#include <string.h>

/* Tuned in simulation (tools/replay, tools/bsim). */
#ifndef MOT_TAU_S
#define MOT_TAU_S 20.0 /* forgetting time constant */
#endif
#ifndef MOT_MIN_WEIGHT
#define MOT_MIN_WEIGHT 300.0 /* ~6 s of 50 Hz samples */
#endif
#ifndef MOT_FIT_EVERY
#define MOT_FIT_EVERY 25 /* samples between fits */
#endif
#ifndef MOT_MAX_CV_PCT
#define MOT_MAX_CV_PCT 1.5f /* max field strength variation */
#endif
#ifndef MOT_MIN_SPREAD
#define MOT_MIN_SPREAD 0.15 /* min per-axis direction std (0.577 = full sphere) */
#endif
#ifndef MOT_MAX_OFFSET
#define MOT_MAX_OFFSET 0.5f /* relative to field strength */
#endif
#ifndef MOT_DEADBAND
#define MOT_DEADBAND 0.03 /* ignore smaller offsets (relative) */
#endif
#ifndef MOT_STEP_TAU_S
#define MOT_STEP_TAU_S 1.0f /* norm error averaging for removal detection */
#endif
#ifndef MOT_STEP_DEV
#define MOT_STEP_DEV 0.025f /* rms relative norm error with the offset applied */
#endif

void mag_offset_track_reset(struct mag_offset_track *t)
{
	memset(t, 0, sizeof(*t));
}

bool mag_sphere_solve(double a_in[4][4], const double b_in[4], double x[4])
{
	double a[4][5];
	for (int i = 0; i < 4; i++) {
		memcpy(a[i], a_in[i], sizeof(a_in[i]));
		a[i][4] = b_in[i];
	}
	for (int col = 0; col < 4; col++) {
		int p = col;
		for (int r = col + 1; r < 4; r++) {
			if (fabs(a[r][col]) > fabs(a[p][col])) {
				p = r;
			}
		}
		if (fabs(a[p][col]) < 1e-15) {
			return false;
		}
		if (p != col) {
			double tmp[5];
			memcpy(tmp, a[col], sizeof(tmp));
			memcpy(a[col], a[p], sizeof(tmp));
			memcpy(a[p], tmp, sizeof(tmp));
		}
		for (int r = 0; r < 4; r++) {
			if (r != col) {
				double f = a[r][col] / a[col][col];
				for (int c = col; c < 5; c++) {
					a[r][c] -= f * a[col][c];
				}
			}
		}
	}
	for (int i = 0; i < 4; i++) {
		x[i] = a[i][4] / a[i][i];
	}
	return true;
}

static void fit(struct mag_offset_track *t)
{
	double w = t->weight;
	if (w < MOT_MIN_WEIGHT) {
		return;
	}
	/* Direction spread per axis (ata[i][i] = sum 4 m_i^2, ata[i][3] = sum 2 m_i). */
	double mean_sq = t->atb[3] / w; /* mean |m|^2 */
	if (mean_sq <= 0) {
		return;
	}
	for (int i = 0; i < 3; i++) {
		double mean = t->ata[i][3] / (2.0 * w);
		double var = t->ata[i][i] / (4.0 * w) - mean * mean;
		if (!(var / mean_sq > MOT_MIN_SPREAD * MOT_MIN_SPREAD)) {
			return; /* not enough rotation: offset is not observable */
		}
	}
	double x[4];
	if (!mag_sphere_solve(t->ata, t->atb, x)) {
		return;
	}
	double r2 = x[3] + x[0] * x[0] + x[1] * x[1] + x[2] * x[2]; /* radius^2 */
	if (!(r2 > 0)) {
		return;
	}
	/* Residual sum of squares of |m|^2 = sum t^2 - x.atb (normal equations). */
	double rss = t->tt - (x[0] * t->atb[0] + x[1] * t->atb[1] + x[2] * t->atb[2] + x[3] * t->atb[3]);
	double rms_t = sqrt(fmax(rss, 0.0) / w);
	t->cv = (float)(100.0 * rms_t / (2.0 * r2)); /* d|m|^2 = 2|m| d|m| -> relative d|m| */
	double off = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
	if (t->cv > MOT_MAX_CV_PCT || off > MOT_MAX_OFFSET * sqrt(r2)) {
		return;
	}
	t->ref_norm = (float)sqrt(r2);
	/* Accumulators hold uncorrected samples: x is the absolute offset. */
	bool significant = off > MOT_DEADBAND * sqrt(r2);
	for (int i = 0; i < 3; i++) {
		t->offset[i] = significant ? (float)x[i] : 0.0f;
	}
}

/* Magnet removed: the raw field matches the last fit's strength much better than
 * the corrected one. Drop the offset at once and refit from fresh data only. */
static void detect_removal(struct mag_offset_track *t, const float m[3], float dt)
{
	bool offset_on = t->offset[0] != 0 || t->offset[1] != 0 || t->offset[2] != 0;
	if (!offset_on || !(t->ref_norm > 0) || !(dt > 0)) {
		t->dev_off = t->dev_raw = 0;
		return;
	}
	float raw = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]) / t->ref_norm - 1.0f;
	float c[3] = {m[0] - t->offset[0], m[1] - t->offset[1], m[2] - t->offset[2]};
	float cor = sqrtf(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]) / t->ref_norm - 1.0f;
	float a = fminf(dt / MOT_STEP_TAU_S, 1.0f);
	t->dev_raw += a * (raw * raw - t->dev_raw);
	t->dev_off += a * (cor * cor - t->dev_off);
	if (t->dev_off > MOT_STEP_DEV * MOT_STEP_DEV && t->dev_raw < t->dev_off / 4) {
		memset(t->offset, 0, sizeof(t->offset));
		memset(t->ata, 0, sizeof(t->ata));
		memset(t->atb, 0, sizeof(t->atb));
		t->tt = t->weight = 0;
		t->since_fit = 0;
	}
}

void mag_offset_track_apply(struct mag_offset_track *t, float m[3], float dt)
{
	detect_removal(t, m, dt);
	double lambda = (dt > 0 && dt < 1) ? exp(-dt / MOT_TAU_S) : 1.0;
	double row[4] = {2.0 * m[0], 2.0 * m[1], 2.0 * m[2], 1.0};
	double target = (double)m[0] * m[0] + (double)m[1] * m[1] + (double)m[2] * m[2];
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			t->ata[i][j] = lambda * t->ata[i][j] + row[i] * row[j];
		}
		t->atb[i] = lambda * t->atb[i] + row[i] * target;
	}
	t->tt = lambda * t->tt + target * target;
	t->weight = lambda * t->weight + 1.0;
	if (++t->since_fit >= MOT_FIT_EVERY) {
		t->since_fit = 0;
		fit(t);
	}
	for (int i = 0; i < 3; i++) {
		m[i] -= t->offset[i];
	}
}
