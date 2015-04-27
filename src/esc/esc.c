/*
esc.c
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

#include "esc.h"

int esc_init(mraa_i2c_context i2c) {
	if (pca_init(i2c, ESC_PWM_FREQUENCY) != 0) {
		return -1;
	}
	pca_setAllAlwaysOff();
	return 0;
}

void esc_disable(esc_position_t esc_position) {
	pca_setAlwaysOff(esc_position);
}

void esc_disable_all() {
	pca_setAllAlwaysOff();
}

int esc_set_uptime(esc_position_t esc_position, float uptime) {
	if(uptime < MOTOR_OFF || uptime > MOTOR_FULL) {
		return -1;
	}

	int value = uptime * 4096.0f * ESC_PWM_FREQUENCY;
	pca_setPWMValue(esc_position, 0, value);
	return 0;
}

int esc_set_power(esc_position_t esc_position, float power_percent) {
	return esc_set_uptime(esc_position, power_percent*(MOTOR_FULL-MOTOR_OFF)/100.0f + MOTOR_OFF);
}

void esc_set_fast_power_0_3(float power_percent0, float power_percent1, float power_percent2, float power_percent3){
	int value0 = (power_percent0*(MOTOR_FULL-MOTOR_OFF)/100.0f + MOTOR_OFF) * 4096.0f * ESC_PWM_FREQUENCY;
	int value1 = (power_percent1*(MOTOR_FULL-MOTOR_OFF)/100.0f + MOTOR_OFF) * 4096.0f * ESC_PWM_FREQUENCY;
	int value2 = (power_percent2*(MOTOR_FULL-MOTOR_OFF)/100.0f + MOTOR_OFF) * 4096.0f * ESC_PWM_FREQUENCY;
	int value3 = (power_percent3*(MOTOR_FULL-MOTOR_OFF)/100.0f + MOTOR_OFF) * 4096.0f * ESC_PWM_FREQUENCY;
	pca_set_fast_0_3_PWM_OnValue(value0, value1, value2, value3);
}

void esc_deinit(void) {
	pca_setAllAlwaysOff();
}
