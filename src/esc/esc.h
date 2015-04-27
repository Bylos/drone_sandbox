/*
 * esc.h
 * Provides convenient functions to control ESCs and servos controlled
 * with PCA9685
 *
 * Copyright (C) 2015  Bylos & Korky
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

#ifndef _ESC_H_
#define _ESC_H_

#include <math.h>
#include <mraa.h>
#include "PCA9685.h"

// Note : Using PCA9685 does not enable a full implementation of OneShot125, but at least enable its rate (up to 2kHz PWM control)
#define ESC_FULL_RATE
//#define ESC_HIGH_RATE
//#define ESC_LOW_RATE

#ifdef ESC_FULL_RATE
// Defines PWM frequency for ESC refresh rate
// HI-rate ESCs can go up to 480Hz
#define ESC_PWM_FREQUENCY	2000.0f
// Defines minimum and maximum PWM up-time
#define MOTOR_OFF	0.000125f
#define MOTOR_FULL	0.000250f
#endif

#ifdef ESC_HIGH_RATE
// Defines PWM frequency for ESC refresh rate
// HI-rate ESCs can go up to 488Hz
#define ESC_PWM_FREQUENCY	488.0f
// Defines minimum and maximum PWM up-time
#define MOTOR_OFF	0.001f
#define MOTOR_FULL	0.002f
#endif

#ifdef ESC_LOW_RATE
// Defines PWM frequency for ESC refresh rate
// Standard ESCs uses 50Hz PWM
#define ESC_PWM_FREQUENCY	50.0f
// Defines minimum and maximum PWM up-time
#define MOTOR_OFF	0.001f
#define MOTOR_FULL	0.002f
#endif


// Assign ESC positions to PCA9685 PWM channels
typedef enum {
	ESC_FRONT_LEFT = 0,
	ESC_FRONT_RIGHT = 1,
	ESC_BACK_RIGHT = 2,
	ESC_BACK_LEFT = 3,
} esc_position_t;

/* esc_init
 * Pass i2c bus context to PCA9685 for low-level communication
 * This function must be called before any other of the library
 */
int esc_init(mraa_i2c_context i2c);

/* esc_disable
 * Disable a single PWM channel corresponding
 * to an ESC position
 */
void esc_disable(esc_position_t esc_position);

/* esc_disable_all
 * Disable all PWM channels without regarding
 * esc positions
 */
void esc_disable_all();

/* esc_set_power
 * Adjust PWM value corresponding to an ESC position
 * the up-time is based on the percentage provided and
 * MOTOR_OFF to MOTOR_FULL range
 */
int esc_set_power(esc_position_t esc_position, float power_percent);

/* esc_set_fast_power_0_3
 * Adjust PWM value corresponding to an ESC position
 * the up-time is based on the percentage provided and
 * MOTOR_OFF to MOTOR_FULL range
 * Set values for pwm channels 0 to 3
 */
// TODO assign esc position to channels
void esc_set_fast_power_0_3(float power_percent0, float power_percent1, float power_percent2, float power_percent3);

/* esc_deinit
 * Disable all PWM channels without regarding
 * esc_positions
 */
void esc_deinit(void);

#endif /* _ESC_H_ */
