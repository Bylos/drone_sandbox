/*
ahrs.h
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

#ifndef AHRS_H_
#define AHRS_H_

#include <math.h>

#define BETA_MIN	0.1f
#define BETA_MAX	10.0f
#define BETA_STP	0.1f

#define AHRS_UPDATE_PERIOD	0.01f

extern volatile float beta;
extern volatile float q1, q2, q3, q4;
extern volatile float yaw, pitch, roll;

void ahrs_BetaUpdate(void);
void ahrs_Madgwick2014(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void ahrs_Madgwick2015(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void ahrs_MadgwickIMU (float ax, float ay, float az, float gx, float gy, float gz);

void ahrs_Quaternion2Euler(void);

#endif /* AHRS_H_ */
