/*
 * c_sandbox.c
 *
 *  Created on: 25 janv. 2015
 *      Author: Bylos
 */


#include <math.h>
#include <mraa.h>
#include <stdio.h>

#include "tools/timeout.h"
#include "tools/utils.h"

#include "ahrs/ahrs.h"
#include "imu/LSM9DS0.h"
#include "rflink/uart.h"
#include "esc/esc.h"
#include "ctrl_loop/PID.h"

void update_IMU(void);
void send_Euler(void);
void read_esc_power(void);

double main_loop_timeout = 2.0;
double imu_update_period = AHRS_UPDATE_PERIOD;
double rf_update_period = 0.1;

char command;

int main(void) {

	mraa_result_print(mraa_init());
	printf("\nMRAA library version %s\n", mraa_get_version());
	printf("on platform %s\n", mraa_get_platform_name());
	printf("Platform has %d pins available\n", mraa_get_pin_count());

	mraa_i2c_context i2c;

	if ((i2c = mraa_i2c_init_raw(1)) == NULL) {
		printf("Unable to initialize i2c\n");
		return MRAA_ERROR_INVALID_RESOURCE;
	}

	uart_init();

	lsm_init(i2c, B_UP);
	lsm_accel_start(A_SCALE_4G, A_ODR_100, A_ABW_50);
	lsm_magn_start(M_SCALE_2GS, M_ODR_125);
	lsm_gyro_start(G_SCALE_500DPS, G_ODR_190_BW_125);

	esc_init(i2c);

	timeout_init();

	int quit = 0;
	int imu_refresh_timer = timeout_set(imu_update_period);
	int rf_update_timer = timeout_set(rf_update_period);
	int main_loop_timer = timeout_set(main_loop_timeout);

	while(!(timeout_passed(main_loop_timer)) && !quit) {

		if(timeout_passed(imu_refresh_timer)) {
			timeout_unset(imu_refresh_timer);
			imu_refresh_timer = timeout_set(imu_update_period);
			update_IMU();
		}

		if(timeout_passed(rf_update_timer)) {
			timeout_unset(rf_update_timer);
			rf_update_timer = timeout_set(rf_update_period);
			send_Euler();
		}

		command = 0;
		while (uart_read(&command, 1) > 0) {
			switch(command) {
			case 'E' :
				read_esc_power();
				break;
			case 'S' :
				esc_disable_all();
				break;
			case 'Q' :
				quit = 1;
				break;
			default :
				break;
			}
			timeout_unset(main_loop_timer);
			main_loop_timer = timeout_set(main_loop_timeout);
		}

		utils_sleep(1.0);
	}

	printf("Deinit Timer\n");
	timeout_unset(imu_refresh_timer);
	timeout_unset(rf_update_timer);
	timeout_unset(main_loop_timer);
	timeout_deinit();

	esc_deinit();
	uart_deinit();

	mraa_result_print(mraa_i2c_stop(i2c));
	mraa_deinit();

	printf("Exit application \n");
	return MRAA_SUCCESS;
}


void update_IMU(void) {
	lsm_accel_read();
	lsm_magn_read();
	lsm_gyro_read();

	ahrs_BetaUpdate(gx, gy, gz);
//	ahrs_Madgwick2014(ax, ay, az, gx*3.14159265359f/180.0f, gy*3.14159265359f/180.0f, gz*3.14159265359f/180.0f, mx, my, -mz);
	ahrs_Madgwick2015(ax, ay, az, gx*3.14159265359f/180.0f, gy*3.14159265359f/180.0f, gz*3.14159265359f/180.0f, mx, my, -mz);
//	ahrs_MadgwickIMU(ax, ay, az, gx*3.14159265359f/180.0f, gy*3.14159265359f/180.0f, gz*3.14159265359f/180.0f);
	ahrs_Quaternion2Euler();
}

void send_Euler(void) {
	char s_buffer[8];
	sprintf(s_buffer, "Y%+06.1f", yaw);
	uart_write(s_buffer, 7);
	sprintf(s_buffer, "P%+06.1f", pitch);
	uart_write(s_buffer, 7);
	sprintf(s_buffer, "R%+06.1f", roll);
	uart_write(s_buffer, 7);
	uart_write("\n",1);
}

void read_esc_power(void) {
	esc_position_t esc;
	char s_power[6];
	char s_esc;
	float power_percent;

	while(uart_bytesAvailable() < 1);
	uart_read(&s_esc, 1);
	esc = strtoul(&s_esc, NULL, 10);
	while(uart_bytesAvailable() < 5);
	uart_read(s_power, 5);
	power_percent = strtof(s_power, NULL);
	if(power_percent < 0 || power_percent > 100) {
		esc_disable(esc);
	}
	else {
		esc_set_power(esc, power_percent);
	}
}
