/*
PCA9685.h
Implements low-level communication and provides convenient functions to
control PCA9685 on the Intel Edison Platform with I2C bus

Copyright (C) 2015  Bylos & Korky
Thanks to Georgi Todorov

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

#ifndef _PCA9685_H
#define _PCA9685_H

#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>      /* Standard I/O functions */
#include <fcntl.h>
#include <syslog.h>		/* Syslog functionallity */
#include <inttypes.h>
#include <errno.h>
#include <math.h>
#include <inttypes.h>
#include <mraa.h>

// Device I2C Address
#define PCA_ADDRESS	0x40

// Register Definitions

#define PCA_MODE1 0x00				//Mode  register  1
#define PCA_MODE2 0x01				//Mode  register  2
#define PCA_SUBADR1 0x02			//I2C-bus subaddress 1
#define PCA_SUBADR2 0x03			//I2C-bus subaddress 2
#define PCA_SUBADR3 0x04			//I2C-bus subaddress 3
#define PCA_ALLCALLADR 0x05     	//LED All Call I2C-bus address
#define PCA_LED0 0x6				//LED0 start register
#define PCA_LED0_ON_L 0x6			//LED0 output and brightness control byte 0
#define PCA_LED0_ON_H 0x7			//LED0 output and brightness control byte 1
#define PCA_LED0_OFF_L 0x8			//LED0 output and brightness control byte 2
#define PCA_LED0_OFF_H 0x9			//LED0 output and brightness control byte 3
#define PCA_LED_MULTIPLYER 4		// For the other 15 channels
#define PCA_ALLLED_ON_L 0xFA    	//load all the LEDn_ON registers, byte 0 (turn 0-7 channels on)
#define PCA_ALLLED_ON_H 0xFB		//load all the LEDn_ON registers, byte 1 (turn 8-15 channels on)
#define PCA_ALLLED_OFF_L 0xFC		//load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off)
#define PCA_ALLLED_OFF_H 0xFD		//load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off)
#define PCA_PRE_SCALE 0xFE			//prescaler for output frequency
#define PCA_CLOCK_FREQ 28200000.0f	//25MHz default osc clock


int	pca_init(mraa_i2c_context i2c_context, float frequency);
void pca_setPWMFreq(float freq);
void pca_setPWMValue(uint8_t ch, int on_value, int off_value);
int pca_getPWMValue(uint8_t ch);
void pca_setAlwaysOff(uint8_t ch);
void pca_setAllAlwaysOff(void);

#endif /* PCA9685_H_ */
