/*
 * PCA9685.h
 * Implements low-level communication and provides convenient functions to
 * control PCA9685 on the Intel Edison Platform with I2C bus
 *
 * Copyright (C) 2015  Bylos & Korky
 * Thanks to Georgi Todorov
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

#ifndef _PCA9685_H_
#define _PCA9685_H_

#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <syslog.h>
#include <inttypes.h>
#include <errno.h>
#include <math.h>
#include <inttypes.h>
#include <mraa.h>

/////////////////////////
// PCA9685 I2C Address //
/////////////////////////
#define PCA_ADDRESS	0x40

/////////////////////////////////////
// PCA9685 Registers And Constants //
/////////////////////////////////////
#define PCA_MODE1 0x00				// Mode  register  1
#define PCA_MODE2 0x01				// Mode  register  2
#define PCA_SUBADR1 0x02			// I2C-bus subaddress 1
#define PCA_SUBADR2 0x03			// I2C-bus subaddress 2
#define PCA_SUBADR3 0x04			// I2C-bus subaddress 3
#define PCA_ALLCALLADR 0x05     	// LED All Call I2C-bus address
#define PCA_LED0 0x6				// LED0 start register
#define PCA_LED0_ON_L 0x6			// LED0 output and brightness control byte 0
#define PCA_LED0_ON_H 0x7			// LED0 output and brightness control byte 1
#define PCA_LED0_OFF_L 0x8			// LED0 output and brightness control byte 2
#define PCA_LED0_OFF_H 0x9			// LED0 output and brightness control byte 3
#define PCA_LED_MULTIPLYER 4		// For the other 15 channels
#define PCA_ALLLED_ON_L 0xFA    	// load all the LEDn_ON registers, byte 0 (turn 0-7 channels on)
#define PCA_ALLLED_ON_H 0xFB		// load all the LEDn_ON registers, byte 1 (turn 8-15 channels on)
#define PCA_ALLLED_OFF_L 0xFC		// load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off)
#define PCA_ALLLED_OFF_H 0xFD		// load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off)
#define PCA_PRE_SCALE 0xFE			// prescaler for output frequency
#define PCA_CLOCK_FREQ 28200000.0f	// 25MHz default osc clock

/* pca_init
 * Initialize PCA9685 operating mode and PWM frequency
 * This function must be called before any other of the library
 */
int	pca_init(mraa_i2c_context i2c_context, float frequency);

/* pca_setPWMFreq
 * Set PCA9685 PWM frequency
 * This value is common to all channels of PCA9685
 */
void pca_setPWMFreq(float freq);

/* pca_setPWMValue
 * Set PWM up-time for the desired channel
 * Up-time goes from on-value to off-value if on-value is lower
 */
void pca_setPWMValue(uint8_t ch, int on_value, int off_value);

/* pca_set_fast_0_3_PWM_OnValue
 * Set PWM up-time for the 4 first channels
 * Only set off_values, on_values are 0 by default
 * Use i2c bulk write and auto-increment for faster operation
 */
void pca_set_fast_0_3_PWM_OnValue(int on_value0, int on_value1, int on_value2, int on_value3);

/* pca_getPWMValue
 * Return the off-value of desired channel
 */
int pca_getPWMValue(uint8_t ch);

/* pca_setAlwaysOff
 * Disable the desired channel
 */
void pca_setAlwaysOff(uint8_t ch);

/* pca_setAllAlwaysOff
 * Disable all channel of PCA9685
 */
void pca_setAllAlwaysOff(void);

#endif /* _PCA9685_H_ */
