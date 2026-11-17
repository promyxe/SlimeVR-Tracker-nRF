/*
 * Rolling hard-iron offset tracker: forgetting sphere fit over calibrated mag
 * samples; reports the residual offset (e.g. a magnet moving with the tracker)
 * when the fit is consistent. Plain C, also built by tools/replay.
 */
#ifndef SLIMENRF_MAG_OFFSET_TRACK_H
#define SLIMENRF_MAG_OFFSET_TRACK_H

#include <stdbool.h>
#include <stdint.h>

struct mag_offset_track {
	double ata[4][4];
	double atb[4];
	double tt;        /* sum of squared targets, for the fit residual */
	double weight;    /* forgotten sample count */
	float offset[3];  /* currently applied offset (calibrated units) */
	float cv;         /* last fit's field-strength variation, % */
	uint32_t since_fit;
};

/* Sphere fit normal equations |m|^2 = 2 c.m + k: solve for x = (cx, cy, cz, k).
 * Gaussian elimination with partial pivoting; false if singular. */
bool mag_sphere_solve(double ata[4][4], const double atb[4], double x[4]);

void mag_offset_track_reset(struct mag_offset_track *t);
/* Feed one calibrated sample (dt = time since the previous one, s); m is then
 * corrected in place by the tracked offset (zero unless significant). */
void mag_offset_track_apply(struct mag_offset_track *t, float m[3], float dt);

#endif
