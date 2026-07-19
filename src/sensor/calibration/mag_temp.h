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
#ifndef SLIMENRF_CAL_MAG_TEMP_H
#define SLIMENRF_CAL_MAG_TEMP_H

/*
 * Magnetometer offset vs temperature compensation using the IMU temperature
 * sensor as proxy (the QMC6309 has none). Learned at rest when the field is
 * undisturbed; applied as a relative correction in calibrated space.
 */

/* Apply temperature compensation to a calibrated mag sample (process_mag). */
void sensor_calibration_mag_temp_apply(float m[3]);

/* Feed a compensated, calibrated mag sample for curve learning. Caller must
 * already gate on mag_calibrated and no detected disturbance; rest detection
 * is checked internally. Called from the sensor loop thread. */
void sensor_calibration_mag_temp_sample(const float m[3]);

/* Clear the learned curve (called when mag calibration is cleared). */
void sensor_calibration_mag_temp_clear(void);

#endif /* SLIMENRF_CAL_MAG_TEMP_H */
