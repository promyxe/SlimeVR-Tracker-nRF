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
#ifndef SLIMENRF_CAL_SENS_AUTO_H
#define SLIMENRF_CAL_SENS_AUTO_H

/*
 * Opportunistic gyro sensitivity self-calibration: whenever a large rotation
 * happens between two quasi-static, mag-anchored moments, the gyro-integrated
 * rotation angle is compared against the accel+mag (TRIAD) orientation delta
 * and the dominant axis' sensitivity scale is refined.
 *
 * All feed functions are called from the sensor loop thread with data in the
 * fusion input frame (calibrated, sensitivity-scaled gyro; calibrated accel;
 * calibrated axis-aligned mag).
 */

void sensor_sens_auto_feed_gyro(const float g[3], float dt);
void sensor_sens_auto_feed_accel(const float a[3]);
void sensor_sens_auto_feed_mag(const float m[3], bool mag_disturbed);

#endif /* SLIMENRF_CAL_SENS_AUTO_H */
