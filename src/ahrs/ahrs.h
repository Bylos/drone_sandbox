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
#include "../types/types.h"

// Defines integration time in orientation filters
// Calls to filters should be synchronized using this value
#define AHRS_UPDATE_PERIOD	0.01f

// Defines start (max) and final (min) feedback gain for fast convergence
// High values gives fast convergence but poor stability
// Low values gives good stability but induces drifts during fast changes
#define BETA_MIN	0.02f
#define BETA_MAX	30.0f
#define ALPHA		0.5f

// Enumerates implemented orientation filters
typedef enum {
	AHRS_MADGWICK_2015,
	AHRS_MADGWICK_IMU
} ahrs_filter_t;

/* ahrs_init
 * initialize internal beta and quaternion values
 */
void ahrs_init(void);

/* ahrs_orientation_update
 * update quaternion and euler angle from new interial sensors values using a chosen filter
 */
euler_angles_t ahrs_orientation_update(inertial_data_t data, ahrs_filter_t filter);

#endif /* _AHRS_H_ */
