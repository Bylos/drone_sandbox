/*
PCA9685.c
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

#include "PCA9685.h"


static mraa_i2c_context i2c = NULL;

mraa_result_t pca_write_byte_data(const uint8_t data, const uint8_t command) {
	if(i2c == NULL) return MRAA_ERROR_INVALID_RESOURCE;
	mraa_i2c_address(i2c, PCA_ADDRESS);
	return mraa_i2c_write_byte_data(i2c, data, command);
}

uint8_t pca_read_byte_data(const uint8_t command) {
	if(i2c == NULL) return -1;
	mraa_i2c_address(i2c, PCA_ADDRESS);
	return mraa_i2c_read_byte_data(i2c, command);
}

int pca_init(mraa_i2c_context i2c_context, float frequency) {
	if((i2c = i2c_context) == NULL) {
		printf("LMS9D0 Init : Wrong i2c context");
		return -1;
	}

	pca_write_byte_data(0x00, PCA_MODE1);
	pca_write_byte_data(0x00, PCA_MODE2);

	pca_setPWMFreq(frequency);

	return 0;
}

void pca_setPWMFreq(float freq) {

		uint8_t prescale_val = (PCA_CLOCK_FREQ / 4096.0f / freq)  - 1.0f;
		pca_write_byte_data(0x10, PCA_MODE1);
		pca_write_byte_data(prescale_val, PCA_PRE_SCALE);
		pca_write_byte_data(0x80, PCA_MODE1);
		pca_write_byte_data(0x00, PCA_MODE2);
}

void pca_setPWMValue(uint8_t ch, int on_value, int off_value) {
		if(on_value > 4095) on_value = 4095;
		if(on_value < 0) on_value = 0;
		if(off_value > 4095) off_value = 4095;
		if(off_value < 0) off_value = 0;

		pca_write_byte_data(on_value & 0xFF,	PCA_LED0_ON_L + PCA_LED_MULTIPLYER * ch);
		pca_write_byte_data(on_value >> 8,		PCA_LED0_ON_H + PCA_LED_MULTIPLYER * ch);
		pca_write_byte_data(off_value & 0xFF,	PCA_LED0_OFF_L + PCA_LED_MULTIPLYER * ch);
		pca_write_byte_data(off_value >> 8,		PCA_LED0_OFF_H + PCA_LED_MULTIPLYER * ch);
}

void pca_setAlwaysOff(uint8_t ch) {
	pca_write_byte_data(0x00,		PCA_LED0_ON_L + PCA_LED_MULTIPLYER * ch);
	pca_write_byte_data(0x00,		PCA_LED0_ON_H + PCA_LED_MULTIPLYER * ch);
	pca_write_byte_data(0x00,		PCA_LED0_OFF_L + PCA_LED_MULTIPLYER * ch);
	pca_write_byte_data(0x10,		PCA_LED0_OFF_H + PCA_LED_MULTIPLYER * ch);
}

void pca_setAllAlwaysOff(void) {
	pca_write_byte_data(0x00,		PCA_ALLLED_ON_L);
	pca_write_byte_data(0x00,		PCA_ALLLED_ON_H);
	pca_write_byte_data(0x00,		PCA_ALLLED_OFF_L);
	pca_write_byte_data(0x10,		PCA_ALLLED_OFF_H);
}

int pca_getPWMValue(uint8_t ch){
	int ledval = 0;
	ledval = pca_read_byte_data(PCA_LED0_OFF_H + PCA_LED_MULTIPLYER * ch);
	ledval = ledval & 0xf;
	ledval <<= 8;
	ledval += pca_read_byte_data(PCA_LED0_OFF_L + PCA_LED_MULTIPLYER * ch);
	return ledval;
}
