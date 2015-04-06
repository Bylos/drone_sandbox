/*
 * esc.h
 *
 *  Created on: 23 févr. 2015
 *      Author: Bylos
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
