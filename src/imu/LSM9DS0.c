/*
LSM9DS0.c
Implements low-level communication and provides convenient functions to
control LSM9DS0 on the Intel Edison Platform with I2C bus

Copyright (C) 2015  Bylos & Korky
Thanks to Jim Lindblom, Taylor Andrews

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

#include "LSM9DS0.h"

// global acceleration, magnetic and gyroscopic variables
volatile float ax, ay, az, mx, my, mz, gx, gy, gz;

// local bias variables
// accel and gyro bias are evaluated at start
int16_t axb=0, ayb=0, azb=0, gxb=0, gyb=0, gzb=0;

// global and static
static mraa_i2c_context i2c = NULL;
static float board_side = (float)B_UP;
static float accel_res = 1;
static float magn_res = 1;
static float gyro_res = 1;

///////////////////////////////////////////////
// I2C Bus Low Level Communication Functions //
///////////////////////////////////////////////

mraa_result_t lsm_write_byte_data(const uint8_t address, const uint8_t data, const uint8_t command) {
	if(i2c == NULL) return MRAA_ERROR_INVALID_RESOURCE;
	mraa_i2c_address(i2c, address);
	return mraa_i2c_write_byte_data(i2c, data, command);
}

uint8_t lsm_read_byte_data(const uint8_t address, const uint8_t command) {
	if(i2c == NULL) return -1;
	mraa_i2c_address(i2c, address);
	return mraa_i2c_read_byte_data(i2c, command);
}

////////////////////////////////////////
// Accelero and Gyro Bias Calculation //
////////////////////////////////////////

void lsm_gyro_bias(void) {
	int32_t tempx = 0, tempy = 0, tempz = 0;

	// Setup FIFO mode
	uint8_t ctrl_reg5_g = lsm_read_byte_data(LSM_ADDRESS_G, LSM_CTRL_REG5_G);
	lsm_write_byte_data(LSM_ADDRESS_G, ctrl_reg5_g | 0x40, LSM_CTRL_REG5_G);
	usleep(20000);
	lsm_write_byte_data(LSM_ADDRESS_G, 0x20|0x1F, LSM_FIFO_CTRL_REG_G);
	usleep(320000);

	// Get samples and average bias
	int samples = (lsm_read_byte_data(LSM_ADDRESS_G, LSM_FIFO_SRC_REG_G) & 0x1F);
	uint16_t u_value;
	int16_t *s_value = (int16_t*)(&u_value);
	int ii;
	for (ii = 0; ii < samples; ii++) {
		u_value = lsm_read_byte_data(LSM_ADDRESS_G, LSM_OUT_X_H_G);
		u_value = u_value << 8;
		u_value += lsm_read_byte_data(LSM_ADDRESS_G, LSM_OUT_X_L_G);
		tempx += *s_value;

		u_value = lsm_read_byte_data(LSM_ADDRESS_G, LSM_OUT_Y_H_G);
		u_value = u_value << 8;
		u_value += lsm_read_byte_data(LSM_ADDRESS_G, LSM_OUT_Y_L_G);
		tempy += *s_value;

		u_value = lsm_read_byte_data(LSM_ADDRESS_G, LSM_OUT_Z_H_G);
		u_value = u_value << 8;
		u_value += lsm_read_byte_data(LSM_ADDRESS_G, LSM_OUT_Z_L_G);
		tempz += *s_value;
	}
	gxb = (int16_t)(tempx/samples);
	gyb = (int16_t)(tempy/samples);
	gzb = (int16_t)(tempz/samples);

	// Log Bias value for future evaluation of bias drift
	FILE *fd = fopen("/home/root/applications/gyro_bias.txt", "a");
	fprintf(fd, "%d\t %d\t %d\n", gxb, gyb, gzb);
	fclose(fd);

	// Setup bypass (normal) mode
	ctrl_reg5_g = lsm_read_byte_data(LSM_ADDRESS_G, LSM_CTRL_REG5_G);
	lsm_write_byte_data(LSM_ADDRESS_G, ctrl_reg5_g & ~0x40, LSM_CTRL_REG5_G);
	usleep(20000);
	lsm_write_byte_data(LSM_ADDRESS_G, 0x00, LSM_FIFO_CTRL_REG_G);
}

void lsm_accel_bias(void) {
	int32_t tempx = 0, tempy = 0, tempz = 0;

	// Setup FIFO mode
	uint8_t ctrl_reg0_xm = lsm_read_byte_data(LSM_ADDRESS_XM, LSM_CTRL_REG0_XM);
	lsm_write_byte_data(LSM_ADDRESS_XM, ctrl_reg0_xm | 0x40, LSM_CTRL_REG0_XM);
	usleep(20000);
	lsm_write_byte_data(LSM_ADDRESS_XM, 0x20|0x1F, LSM_FIFO_CTRL_REG);
	usleep(320000);

	// Get samples and average bias
	int samples = (lsm_read_byte_data(LSM_ADDRESS_XM, LSM_FIFO_SRC_REG) & 0x1F);
	uint16_t u_value;
	int16_t *s_value = (int16_t*)(&u_value);
	int ii;
	for (ii = 0; ii < samples; ii++) {
		u_value = lsm_read_byte_data(LSM_ADDRESS_XM, LSM_OUT_X_H_A);
		u_value = u_value << 8;
		u_value += lsm_read_byte_data(LSM_ADDRESS_XM, LSM_OUT_X_L_A);
		tempx += *s_value;

		u_value = lsm_read_byte_data(LSM_ADDRESS_XM, LSM_OUT_Y_H_A);
		u_value = u_value << 8;
		u_value += lsm_read_byte_data(LSM_ADDRESS_XM, LSM_OUT_Y_L_A);
		tempy += *s_value;

		u_value = lsm_read_byte_data(LSM_ADDRESS_XM, LSM_OUT_Z_H_A);
		u_value = u_value << 8;
		u_value += lsm_read_byte_data(LSM_ADDRESS_XM, LSM_OUT_Z_L_A);
		tempz += *s_value + (int16_t)(1/accel_res);
	}
	axb = (int16_t)(tempx/samples);
	ayb = (int16_t)(tempy/samples);
	azb = (int16_t)(tempz/samples);

	// Log Bias value for future evaluation of bias drift
	FILE *fd = fopen("/home/root/applications/accel_bias.txt", "a");
	fprintf(fd, "%d\t %d\t %d\n", axb, ayb, azb);
	fclose(fd);

	// Setup bypass (normal) mode
	ctrl_reg0_xm = lsm_read_byte_data(LSM_ADDRESS_XM, LSM_CTRL_REG0_XM);
	lsm_write_byte_data(LSM_ADDRESS_XM, ctrl_reg0_xm & ~0x40, LSM_CTRL_REG0_XM);
	usleep(20000);
	lsm_write_byte_data(LSM_ADDRESS_XM, 0x00, LSM_FIFO_CTRL_REG);
}

//////////////////////////////
// Initialization Functions //
//////////////////////////////

int lsm_init(mraa_i2c_context i2c_context, board_side_t side) {
	if((i2c = i2c_context) == NULL) {
		printf("LSM9D0 Init : Wrong i2c context");
		return -1;
	}
	board_side = (float)side;

	return 0;
}

void lsm_accel_start(accel_scale_t scale, accel_odr_t odr, accel_abw_t abw) {
	lsm_write_byte_data(LSM_ADDRESS_XM, 0b00000000, LSM_CTRL_REG0_XM);
	lsm_write_byte_data(LSM_ADDRESS_XM, 0b00000111 | (odr << 4) , LSM_CTRL_REG1_XM);
	lsm_write_byte_data(LSM_ADDRESS_XM, (abw << 6) | (scale << 3), LSM_CTRL_REG2_XM);
	lsm_write_byte_data(LSM_ADDRESS_XM, 0b00000000, LSM_CTRL_REG3_XM);

    accel_res = scale == A_SCALE_16G ? 16.0f / 32768.0f :
           (((float) scale + 1.0f) * 2.0f) / 32768.0f;

    lsm_accel_bias();
}

void lsm_magn_start(magn_scale_t scale, magn_odr_t odr) {
	lsm_write_byte_data(LSM_ADDRESS_XM, odr << 2, LSM_CTRL_REG5_XM);
	lsm_write_byte_data(LSM_ADDRESS_XM, scale << 5, LSM_CTRL_REG6_XM);
	lsm_write_byte_data(LSM_ADDRESS_XM, 0b00000000, LSM_CTRL_REG7_XM);

	magn_res = scale == M_SCALE_2GS ? 2.0f / 32768.0f :
	           (float) (scale << 2) / 32768.0f;
}

void lsm_gyro_start(gyro_scale_t scale, gyro_odr_t odr) {
	lsm_write_byte_data(LSM_ADDRESS_G, 0b00001111 | (odr << 4), LSM_CTRL_REG1_G);
	lsm_write_byte_data(LSM_ADDRESS_G, 0b00000000, LSM_CTRL_REG2_G);
	lsm_write_byte_data(LSM_ADDRESS_G, 0b00000000, LSM_CTRL_REG3_G);
	lsm_write_byte_data(LSM_ADDRESS_G, scale << 4, LSM_CTRL_REG4_G);
	lsm_write_byte_data(LSM_ADDRESS_G, 0b00000000, LSM_CTRL_REG5_G);

    switch (scale)
    {
    case G_SCALE_245DPS:
        gyro_res = 245.0f / 32768.0f;
        break;
    case G_SCALE_500DPS:
    	gyro_res = 500.0f / 32768.0f;
        break;
    case G_SCALE_2000DPS:
    	gyro_res = 2000.0f / 32768.0f;
        break;
    }

    lsm_gyro_bias();
}

////////////////////////////
// Sensors Read Functions //
////////////////////////////

void lsm_accel_read(void) {
	uint16_t u_value;
	int16_t *s_value = (int16_t *)(&u_value);

	u_value = lsm_read_byte_data(LSM_ADDRESS_XM, LSM_OUT_X_H_A);
	u_value = u_value << 8;
	u_value += lsm_read_byte_data(LSM_ADDRESS_XM, LSM_OUT_X_L_A);
	ax = accel_res * (float)(*s_value - axb);

	u_value = lsm_read_byte_data(LSM_ADDRESS_XM, LSM_OUT_Y_H_A);
	u_value = u_value << 8;
	u_value += lsm_read_byte_data(LSM_ADDRESS_XM, LSM_OUT_Y_L_A);
	ay = -accel_res * (float)(*s_value - ayb);

	u_value = lsm_read_byte_data(LSM_ADDRESS_XM, LSM_OUT_Z_H_A);
	u_value = u_value << 8;
	u_value += lsm_read_byte_data(LSM_ADDRESS_XM, LSM_OUT_Z_L_A);
	az = -accel_res * (float)(*s_value - azb);
}

// magn bias are predefined values
const float mxb = -1302.185116f, myb = -545.239796f, mzb = 130.141928;

void lsm_magn_read(void) {
	uint16_t u_value;
	int16_t *s_value = (int16_t *)(&u_value);
	float tempx, tempy, tempz;

//	FILE *fd = fopen("/home/root/applications/magn_log.txt", "a");

	u_value = lsm_read_byte_data(LSM_ADDRESS_XM, LSM_OUT_X_H_M);
	u_value = u_value << 8;
	u_value += lsm_read_byte_data(LSM_ADDRESS_XM, LSM_OUT_X_L_M);
	tempx = (float)(*s_value) - mxb;

//	fprintf(fd, "%d\t", *s_value);

	u_value = lsm_read_byte_data(LSM_ADDRESS_XM, LSM_OUT_Y_H_M);
	u_value = u_value << 8;
	u_value += lsm_read_byte_data(LSM_ADDRESS_XM, LSM_OUT_Y_L_M);
	tempy =  (float)(*s_value) - myb;

//	fprintf(fd, "%d\t", *s_value);

	u_value = lsm_read_byte_data(LSM_ADDRESS_XM, LSM_OUT_Z_H_M);
	u_value = u_value << 8;
	u_value += lsm_read_byte_data(LSM_ADDRESS_XM, LSM_OUT_Z_L_M);
	tempz =  (float)(*s_value) - mzb;

//	fprintf(fd, "%d\n", *s_value);
//	fclose(fd);

	mx = magn_res * (tempx * 1.195299f + tempy * 0.003829f - tempz * 0.089238f);
	my = -magn_res * (tempx * 0.003829f + tempy * 1.283451 + tempz * 0.058980);
	mz = -magn_res * (- tempx * 0.089238 + tempy * 0.058980 + tempz * 1.233952);
}

void lsm_gyro_read(void) {
	uint16_t u_value;
	int16_t *s_value = (int16_t *)(&u_value);

	u_value = lsm_read_byte_data(LSM_ADDRESS_G, LSM_OUT_X_H_G);
	u_value = u_value << 8;
	u_value += lsm_read_byte_data(LSM_ADDRESS_G, LSM_OUT_X_L_G);
	gx = gyro_res * (float)(*s_value - gxb);

	u_value = lsm_read_byte_data(LSM_ADDRESS_G, LSM_OUT_Y_H_G);
	u_value = u_value << 8;
	u_value += lsm_read_byte_data(LSM_ADDRESS_G, LSM_OUT_Y_L_G);
	gy = -gyro_res * (float)(*s_value - gyb);

	u_value = lsm_read_byte_data(LSM_ADDRESS_G, LSM_OUT_Z_H_G);
	u_value = u_value << 8;
	u_value += lsm_read_byte_data(LSM_ADDRESS_G, LSM_OUT_Z_L_G);
	gz = -gyro_res * (float)(*s_value - gzb);
}
