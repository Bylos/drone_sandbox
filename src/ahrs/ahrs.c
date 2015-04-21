/*
ahrs.c
Implements orientation filters based on Madgwick's quaternion algorithm

Copyright (C) 2015  Bylos & Korky
Thanks to Seb Madgwick, Jim Lindblom, Kris Winer

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "ahrs.h"

static float beta = BETA_MAX;
static float q1 = 1.0f, q2 = 0.0f, q3 = 0.0f, q4 = 0.0f;

void ahrs_BetaUpdate(vector_t gyro) {
	float betaTemp = BETA_MIN;
	float alphaFilter = ALPHA;
	float angularVelocity = sqrt(gyro.x*gyro.x+gyro.y*gyro.y+gyro.z*gyro.z);
	float minAngularVelocity = 1.0f; // deg/s
	float maxAngularVelocity = 600.0f; // deg/s

	if(angularVelocity<minAngularVelocity) betaTemp = BETA_MIN;
	else if(angularVelocity>=maxAngularVelocity) betaTemp = BETA_MAX;
	else betaTemp = BETA_MIN + (BETA_MAX - BETA_MIN)/(maxAngularVelocity - minAngularVelocity) * (angularVelocity - minAngularVelocity);

	if(betaTemp>0.0f) beta=alphaFilter*beta+(1.0f-alphaFilter)*betaTemp;
}


/*
 * ahrs_Madgwick2015
 * Updated implementation of Madgwick's orientation filter
 * Thanks to Jeroen van de Mortel <http://diydrones.com/profile/JeroenvandeMortel>
 */
void ahrs_Madgwick2015(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float deltat = AHRS_UPDATE_PERIOD;
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _8bx;
  float _8bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;
  _8bx = 2.0f * _4bx;
  _8bz = 2.0f * _4bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * (q2q4 - q1q3) - ax) + _2q2 * (2.0f * (q1q2 + q3q4) - ay) - _4bz * q3 * (_4bx * (0.5f - q3q3 - q4q4) + _4bz * (q2q4 - q1q3) - mx) + (-_4bx * q4 + _4bz * q2) * (_4bx * (q2q3 - q1q4) + _4bz * (q1q2 + q3q4) - my) + _4bx * q3 * (_4bx * (q1q3 + q2q4) + _4bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * (q2q4 - q1q3) - ax) + _2q1 * (2.0f * (q1q2 + q3q4) - ay) - 4.0f * q2 * (2.0f * (0.5 - q2q2 - q3q3) - az) + _4bz * q4 * (_4bx * (0.5f - q3q3 - q4q4) + _4bz * (q2q4 - q1q3) - mx) + (_4bx * q3 + _4bz * q1) * (_4bx * (q2q3 - q1q4) + _4bz * (q1q2 + q3q4) - my) + (_4bx * q4 - _8bz * q2) * (_4bx * (q1q3 + q2q4) + _4bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * (q2q4 - q1q3) - ax) + _2q4 * (2.0f * (q1q2 + q3q4) - ay) + (-4.0f * q3) * (2.0f * (0.5f - q2q2 - q3q3) - az) + (-_8bx * q3 - _4bz * q1) * (_4bx * (0.5f - q3q3 - q4q4) + _4bz * (q2q4 - q1q3) - mx) + (_4bx * q2 + _4bz * q4) * (_4bx * (q2q3 - q1q4) + _4bz * (q1q2 + q3q4) - my) + (_4bx * q1 - _8bz * q3) * (_4bx * (q1q3 + q2q4) + _4bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * (q2q4 - q1q3) - ax) + _2q3 * (2.0f * (q1q2 + q3q4) - ay) + (-_8bx * q4 + _4bz * q2) * (_4bx * (0.5f - q3q3 - q4q4) + _4bz * (q2q4 - q1q3) - mx) + (-_4bx * q1 + _4bz * q3) * (_4bx * (q2q3 - q1q4) + _4bz * (q1q2 + q3q4) - my) + (_4bx * q2) * (_4bx * (q1q3 + q2q4) + _4bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f/norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f/norm;
  q1 *= norm;
  q2 *= norm;
  q3 *= norm;
  q4 *= norm;
}

/*
 * ahrs_MadgwickIMU
 * 6dof version of Madgwick orientation filter
 */
void ahrs_MadgwickIMU(float ax, float ay, float az, float gx, float gy, float gz) {
	float deltat = AHRS_UPDATE_PERIOD;
	float norm;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q1, _2q2, _2q3, _2q4, _4q1, _4q2, _4q3 ,_8q2, _8q3, q1q1, q2q2, q3q3, q4q4;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz);
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy);
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx);
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx);

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Auxiliary variables to avoid repeated arithmetic
	_2q1 = 2.0f * q1;
	_2q2 = 2.0f * q2;
	_2q3 = 2.0f * q3;
	_2q4 = 2.0f * q4;
	_4q1 = 4.0f * q1;
	_4q2 = 4.0f * q2;
	_4q3 = 4.0f * q3;
	_8q2 = 8.0f * q2;
	_8q3 = 8.0f * q3;
	q1q1 = q1 * q1;
	q2q2 = q2 * q2;
	q3q3 = q3 * q3;
	q4q4 = q4 * q4;

	// Gradient decent algorithm corrective step
	s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
	s2 = _4q2 * q4q4 - _2q4 * ax + 4.0f * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
	s3 = 4.0f * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
	s4 = 4.0f * q2q2 * q4 - _2q2 * ax + 4.0f * q3q3 * q4 - _2q3 * ay;
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Apply feedback step
	qDot1 -= beta * s1;
	qDot2 -= beta * s2;
	qDot3 -= beta * s3;
	qDot4 -= beta * s4;

	// Integrate rate of change of quaternion to yield quaternion
	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;

	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	q1 *= norm;
	q2 *= norm;
	q3 *= norm;
	q4 *= norm;
}

/*
 * ahrs_Quaternion2Euler
 * Converts quaternion to Trait-Brian angles (z-y-x)
 * Yaw, Pitch and Roll values are updated internally
 */
euler_angles_t ahrs_Quaternion2Euler(void) {
	euler_angles_t angles;
	angles.yaw   = atan2(2.0f * (q2 * q3 + q1 * q4), q1 * q1 + q2 * q2 - q3 * q3 - q4 * q4);
	angles.pitch = -asin(2.0f * (q2 * q4 - q1 * q3));
	angles.roll  = atan2(2.0f * (q1 * q2 + q3 * q4), q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4);
	angles.pitch *= 180.0f / 3.14159265359f;
	angles.yaw   *= 180.0f / 3.14159265359f;
	angles.roll  *= 180.0f / 3.14159265359f;
	return angles;
}

void ahrs_init(void) {
	q1 = 1.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	q4 = 0.0f;
	beta = BETA_MAX;
}

euler_angles_t ahrs_orientation_update(inertial_data_t data, ahrs_filter_t filter) {
	ahrs_BetaUpdate(data.gyro);
	switch(filter) {
	case AHRS_MADGWICK_2015:
		ahrs_Madgwick2015(
				data.accel.x, data.accel.y, data.accel.z,
				data.gyro.x*3.14159265359f/180.0f, data.gyro.y*3.14159265359f/180.0f, data.gyro.z*3.14159265359f/180.0f,
				data.magn.x, data.magn.y, data.magn.z);
		break;
	case AHRS_MADGWICK_IMU:
		ahrs_MadgwickIMU(
				data.accel.x, data.accel.y, data.accel.z,
				data.gyro.x*3.14159265359f/180.0f, data.gyro.y*3.14159265359f/180.0f, data.gyro.z*3.14159265359f/180.0f);
		break;
	default :
		break;
	}
	return ahrs_Quaternion2Euler();
}
