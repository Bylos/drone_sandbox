/*
LSM9DS0.h
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

#ifndef LMS9D0_H_
#define LMS9D0_H_

#include <mraa.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

extern volatile float ax, ay, az, mx, my, mz, gx, gy, gz;

///////////////////////////
// LSM9DS0 I2C Addresses //
///////////////////////////
#define LMS_ADDRESS_G			0x6B
#define LMS_ADDRESS_XM			0x1D

////////////////////////////////
// LSM9DS0 Gyro (G) Registers //
////////////////////////////////
#define LMS_WHO_AM_I_G          0x0F
#define LMS_CTRL_REG1_G         0x20
#define LMS_CTRL_REG2_G         0x21
#define LMS_CTRL_REG3_G         0x22
#define LMS_CTRL_REG4_G         0x23
#define LMS_CTRL_REG5_G         0x24
#define LMS_REFERENCE_G         0x25
#define LMS_STATUS_REG_G        0x27
#define LMS_OUT_X_L_G           0x28
#define LMS_OUT_X_H_G           0x29
#define LMS_OUT_Y_L_G           0x2A
#define LMS_OUT_Y_H_G           0x2B
#define LMS_OUT_Z_L_G           0x2C
#define LMS_OUT_Z_H_G           0x2D
#define LMS_FIFO_CTRL_REG_G     0x2E
#define LMS_FIFO_SRC_REG_G      0x2F
#define LMS_INT1_CFG_G          0x30
#define LMS_INT1_SRC_G          0x31
#define LMS_INT1_THS_XH_G       0x32
#define LMS_INT1_THS_XL_G       0x33
#define LMS_INT1_THS_YH_G       0x34
#define LMS_INT1_THS_YL_G       0x35
#define LMS_INT1_THS_ZH_G       0x36
#define LMS_INT1_THS_ZL_G       0x37
#define LMS_INT1_DURATION_G     0x38

//////////////////////////////////////////
// LSM9DS0 Accel/Magneto (XM) Registers //
//////////////////////////////////////////
#define LMS_OUT_TEMP_L_XM       0x05
#define LMS_OUT_TEMP_H_XM       0x06
#define LMS_STATUS_REG_M        0x07
#define LMS_OUT_X_L_M           0x08
#define LMS_OUT_X_H_M           0x09
#define LMS_OUT_Y_L_M           0x0A
#define LMS_OUT_Y_H_M           0x0B
#define LMS_OUT_Z_L_M           0x0C
#define LMS_OUT_Z_H_M           0x0D
#define LMS_WHO_AM_I_XM         0x0F
#define LMS_INT_CTRL_REG_M      0x12
#define LMS_INT_SRC_REG_M       0x13
#define LMS_INT_THS_L_M         0x14
#define LMS_INT_THS_H_M         0x15
#define LMS_OFFSET_X_L_M        0x16
#define LMS_OFFSET_X_H_M        0x17
#define LMS_OFFSET_Y_L_M        0x18
#define LMS_OFFSET_Y_H_M        0x19
#define LMS_OFFSET_Z_L_M        0x1A
#define LMS_OFFSET_Z_H_M        0x1B
#define LMS_REFERENCE_X         0x1C
#define LMS_REFERENCE_Y         0x1D
#define LMS_REFERENCE_Z         0x1E
#define LMS_CTRL_REG0_XM        0x1F
#define LMS_CTRL_REG1_XM        0x20
#define LMS_CTRL_REG2_XM        0x21
#define LMS_CTRL_REG3_XM        0x22
#define LMS_CTRL_REG4_XM        0x23
#define LMS_CTRL_REG5_XM        0x24
#define LMS_CTRL_REG6_XM        0x25
#define LMS_CTRL_REG7_XM        0x26
#define LMS_STATUS_REG_A        0x27
#define LMS_OUT_X_L_A           0x28
#define LMS_OUT_X_H_A           0x29
#define LMS_OUT_Y_L_A           0x2A
#define LMS_OUT_Y_H_A           0x2B
#define LMS_OUT_Z_L_A           0x2C
#define LMS_OUT_Z_H_A           0x2D
#define LMS_FIFO_CTRL_REG       0x2E
#define LMS_FIFO_SRC_REG        0x2F
#define LMS_INT_GEN_1_REG       0x30
#define LMS_INT_GEN_1_SRC       0x31
#define LMS_INT_GEN_1_THS       0x32
#define LMS_INT_GEN_1_DURATION  0x33
#define LMS_INT_GEN_2_REG       0x34
#define LMS_INT_GEN_2_SRC       0x35
#define LMS_INT_GEN_2_THS       0x36
#define LMS_INT_GEN_2_DURATION  0x37
#define LMS_CLICK_CFG           0x38
#define LMS_CLICK_SRC           0x39
#define LMS_CLICK_THS           0x3A
#define LMS_TIME_LIMIT          0x3B
#define LMS_TIME_LATENCY        0x3C
#define LMS_TIME_WINDOW         0x3D
#define LMS_ACT_THS             0x3E
#define LMS_ACT_DUR             0x3F

// edison board orientation
typedef enum
{
	B_UP = 1,
	B_DOWN = -1,
} board_side_t;

// accel_scale defines all possible FSR's of the accelerometer:
typedef enum
{
	A_SCALE_2G, // 000: +/- 2g
	A_SCALE_4G, // 001: +/- 4g
	A_SCALE_6G, // 010: +/- 6g
	A_SCALE_8G, // 011: +/- 8g
	A_SCALE_16G // 100: +/- 16g
} accel_scale_t;

// accel_oder defines all possible output data rates of the accelerometer:
typedef enum
{
	A_POWER_DOWN,   // Power-down mode (0x0)
	A_ODR_3125,     // 3.125 Hz (0x1)
	A_ODR_625,      // 6.25 Hz (0x2)
	A_ODR_125,      // 12.5 Hz (0x3)
	A_ODR_25,       // 25 Hz (0x4)
	A_ODR_50,       // 50 Hz (0x5)
	A_ODR_100,      // 100 Hz (0x6)
	A_ODR_200,      // 200 Hz (0x7)
	A_ODR_400,      // 400 Hz (0x8)
	A_ODR_800,      // 800 Hz (9)
	A_ODR_1600      // 1600 Hz (0xA)
} accel_odr_t;

// accel_abw defines all possible anti-aliasing filter rates of the accelerometer:
typedef enum
{
	A_ABW_773,		// 773 Hz (0x0)
	A_ABW_194,		// 194 Hz (0x1)
	A_ABW_362,		// 362 Hz (0x2)
	A_ABW_50,		//  50 Hz (0x3)
} accel_abw_t;

// gyro_scale defines the possible full-scale ranges of the gyroscope:
typedef enum
{
	G_SCALE_245DPS,     // 00: +/- 245 degrees per second
	G_SCALE_500DPS,     // 01: +/- 500 dps
	G_SCALE_2000DPS,    // 10: +/- 2000 dps
} gyro_scale_t;

// gyro_odr defines all possible data rate/bandwidth combos of the gyro:
typedef enum
{                           // ODR (Hz) --- Cutoff
	G_ODR_95_BW_125  = 0x0, //   95         12.5
	G_ODR_95_BW_25   = 0x1, //   95          25
	// 0x2 and 0x3 define the same data rate and bandwidth
	G_ODR_190_BW_125 = 0x4, //   190        12.5
	G_ODR_190_BW_25  = 0x5, //   190         25
	G_ODR_190_BW_50  = 0x6, //   190         50
	G_ODR_190_BW_70  = 0x7, //   190         70
	G_ODR_380_BW_20  = 0x8, //   380         20
	G_ODR_380_BW_25  = 0x9, //   380         25
	G_ODR_380_BW_50  = 0xA, //   380         50
	G_ODR_380_BW_100 = 0xB, //   380         100
	G_ODR_760_BW_30  = 0xC, //   760         30
	G_ODR_760_BW_35  = 0xD, //   760         35
	G_ODR_760_BW_50  = 0xE, //   760         50
	G_ODR_760_BW_100 = 0xF, //   760         100
} gyro_odr_t;

// magn_scale defines all possible FSR's of the magnetometer:
typedef enum
{
	M_SCALE_2GS,    // 00: +/- 2Gs
	M_SCALE_4GS,    // 01: +/- 4Gs
	M_SCALE_8GS,    // 10: +/- 8Gs
	M_SCALE_12GS,   // 11: +/- 12Gs
} magn_scale_t;

// magn_odr defines all possible output data rates of the magnetometer:
typedef enum
{
	M_ODR_3125, // 3.125 Hz (0x00)
	M_ODR_625,  // 6.25 Hz (0x01)
	M_ODR_125,  // 12.5 Hz (0x02)
	M_ODR_25,   // 25 Hz (0x03)
	M_ODR_50,   // 50 (0x04)
	M_ODR_100,  // 100 Hz (0x05)
} magn_odr_t;

/*
 * lms_init get i2c context for future communications
 * Return 0 if i2c bus is valid
 */
int lms_init(mraa_i2c_context i2c_context, board_side_t side);

/*
 * lms_accel_start set up and start acceleration sampling
 */
void lms_accel_start(accel_scale_t scale, accel_odr_t odr, accel_abw_t abw);

/*
 * lms_gyro_start set up and start gyroscope sampling
 */
void lms_gyro_start(gyro_scale_t scale, gyro_odr_t odr);

/*
 * lms_magn_start set up and start magnetometer sampling
 */
void lms_magn_start(magn_scale_t scale, magn_odr_t odr);

/*
 * lms_accel_read store acceleration values in ax, ay, az
 */
void lms_accel_read(void);

/*
 * lms_gyro_read store gyroscopic values in gx, gy, gz
 */
void lms_gyro_read(void);

/*
 * lms_magn_read store magnetometer values in mx, my, mz
 */
void lms_magn_read(void);

#endif /* LMS9D0_H_ */
