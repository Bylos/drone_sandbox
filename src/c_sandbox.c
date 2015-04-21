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
#include "rflink/rflink.h"
#include "esc/esc.h"
#include "ctrl_loop/PID.h"
#include "types/types.h"

#define MAIN_LOOP_TIMEOUT	10.0f

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

	lsm_init(i2c);
	esc_init(i2c);
	rflink_init();
	timeout_init();

	inertial_data_t sensors_data;
	euler_angles_t orientation_angles;
	rf_command_t command;

	int quit = 0;
	int ahrs_update_timer = timeout_set(AHRS_UPDATE_PERIOD);
	int rf_update_timer = timeout_set(RF_UPDATE_PERIOD);
	int main_loop_timer = timeout_set(MAIN_LOOP_TIMEOUT);

	while(!(timeout_passed(main_loop_timer)) && !quit) {

		if(timeout_passed(ahrs_update_timer)) {
			timeout_unset(AHRS_UPDATE_PERIOD);
			ahrs_update_timer = timeout_set(AHRS_UPDATE_PERIOD);
			sensors_data = lsm_inertial_read();
			orientation_angles = ahrs_orientation_update(sensors_data, AHRS_MADGWICK_2015);
		}

		if(timeout_passed(rf_update_timer)) {
			timeout_unset(rf_update_timer);
			rf_update_timer = timeout_set(RF_UPDATE_PERIOD);
			rflink_orientation_send(orientation_angles);
		}

		while((command = rflink_command_check()) != RF_CMD_NONE) {
			timeout_unset(main_loop_timer);
			main_loop_timer = timeout_set(MAIN_LOOP_TIMEOUT);

			rflink_cmd_esc_msg_t cmd_esc;
			switch(command) {
			case RF_CMD_QUIT:
				quit = 1;
				break;
			case RF_CMD_STOP:
				esc_disable_all();
				break;
			case RF_CMD_ESC:
				cmd_esc = rflink_read_esc();
				if (cmd_esc.percent_value < 0 || cmd_esc.percent_value > 100) {
					esc_disable(cmd_esc.esc_position);
				}
				else {
					esc_set_power(cmd_esc.esc_position, cmd_esc.percent_value);
				}
				break;

			case RF_CMD_THROTTLE:
				//TODO: Implement ESC common mode (throttle)
				break;

			case RF_CMD_PITCH:
				//TODO: Implement Pitch control loop
				break;

			case RF_CMD_ROLL:
				//TODO: Implement Roll control loop
				break;

			default:
				break;
			}
		}

		utils_sleep(1.0);
	}

	timeout_unset(ahrs_update_timer);
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
