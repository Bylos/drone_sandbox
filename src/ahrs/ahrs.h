/*
 * ahrs.h
 * Implements orientation filters based on Madgwick's quaternion
 * algorithm
 *
 * Copyright (C) 2015  Bylos & Korky
 * Thanks to Seb Madgwick, Jim Lindblom, Kris Winer
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _AHRS_H_
#define _AHRS_H_

#include <math.h>

// Defines integration time in orientation filters
// Calls to filters should be synchronized using this value
#define AHRS_UPDATE_PERIOD	0.01f

// Defines start (max) and final (min) feedback gain for fast convergence
// High values gives fast convergence but poor stability
// Low values gives good stability but induces drifts during fast changes
#define BETA_MIN	0.02f
#define BETA_MAX	30.0f
#define BETA_STEP	0.1f

// Madgwick orientation filter's beta gain declaration
extern volatile float beta;
// Quaternions declaration
extern volatile float q1, q2, q3, q4;
// Euler's angle declaration
extern volatile float yaw, pitch, roll;

/* ahrs_BetaUpdate
 * Update dynamically beta feedback gain based on current angular velocity
 * The update is smoothed with an exponential filter
 */
void ahrs_BetaUpdate(float gx, float gy, float gz);

/*
 * ahrs_Madgwick2014
 * Original implementation of Madgwick's orientation filter
 * The function update quaternion values internally
 */
void ahrs_Madgwick2014(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

/*
 * ahrs_Madgwick2014
 * Original implementation of Madgwick's orientation filter
 * The function update quaternion values internally
 */
void ahrs_Madgwick2015(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

/*
 * ahrs_Madgwick2015
 * Updated implementation of Madgwick's orientation filter
 * Thanks to Jeroen van de Mortel <http://diydrones.com/profile/JeroenvandeMortel>
 */
void ahrs_MadgwickIMU (float ax, float ay, float az, float gx, float gy, float gz);

/*
 * ahrs_Quaternion2Euler
 * Converts quaternion to Trait-Brian angles (z-y-x)
 * Yaw, Pitch and Roll values are updated internally
 */
void ahrs_Quaternion2Euler(void);

#endif /* _AHRS_H_ */
