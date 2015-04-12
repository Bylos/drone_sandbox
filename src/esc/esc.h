/*
esc.h
Provides convenient functions to control ESCs and servos controlled
with PCA9685

Copyright (C) 2015  Bylos & Korky

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

#ifndef ESC_H_
#define ESC_H_

#include <math.h>
#include <mraa.h>
#include "PCA9685.h"

#define ESC_PWM_FREQUENCY	50.0f

#define MOTOR_OFF	0.001f
#define MOTOR_FULL	0.002f

typedef enum {
	ESC_FRONT_LEFT = 0,
	ESC_FRONT_RIGHT = 1,
	ESC_BACK_RIGHT = 2,
	ESC_BACK_LEFT = 3,
} esc_position_t;

int esc_init(mraa_i2c_context i2c);
void esc_disable(esc_position_t esc_position);
void esc_disable_all();
int esc_set_uptime(esc_position_t esc_position, float uptime);
int esc_set_power(esc_position_t esc_position, float power_percent);
void esc_deinit(void);

#endif /* ESC_H_ */
