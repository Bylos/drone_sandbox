/*
 * esc.c
 *
 *  Created on: 23 févr. 2015
 *      Author: Bylos
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

void esc_deinit(void) {
	pca_setAllAlwaysOff();
}
