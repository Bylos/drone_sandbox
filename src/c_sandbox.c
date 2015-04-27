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
	timeout_init();

	inertial_data_t sensors_data;
	euler_angles_t orientation_angles;
	rf_command_t command;



	//Set esc to 0
	esc_set_power(ESC_BACK_LEFT, 0.0f);
	esc_set_power(ESC_BACK_RIGHT, 0.0f);
	esc_set_power(ESC_FRONT_LEFT, 0.0f);
	esc_set_power(ESC_FRONT_RIGHT, 0.0f);
	utils_sleep(1.0);

	//PID variables
	float commonPower = 00.0f ;
	float differentialPower = 0.0f ;
	float maxDifferentialPower = 100.0f ;
	float targetRollRate = 0.0f ;
	float targetRollPos = 0.00f ;
	int rollRatePID = PID_set(0.08f, 0.20f, 0.0f, 0.0f, 50.0f) ;
	int rollPosPID = PID_set(3.0f, 0.00f, 0.00f, 0.0f, 5000.0f) ;

	printf("Start initialization\n");
	int ahrs_update_timer = timeout_set(AHRS_UPDATE_PERIOD);
	while(ahrs_init(lsm_inertial_read(), AHRS_MADGWICK_2015) == 0) {
		while(timeout_passed(ahrs_update_timer) == 0);
		timeout_unset(ahrs_update_timer);
		ahrs_update_timer = timeout_set(AHRS_UPDATE_PERIOD);
	}
	printf("Stop initialization\n");
	//initialization of ahrs


	rflink_init();

	//timers
	int quit = 0;
	int rf_update_timer = timeout_set(RF_UPDATE_PERIOD);
	int main_loop_timer = timeout_set(MAIN_LOOP_TIMEOUT);

	while(!(timeout_passed(main_loop_timer)) && !quit) {

		if(timeout_passed(ahrs_update_timer)) {
			timeout_unset(AHRS_UPDATE_PERIOD);
			ahrs_update_timer = timeout_set(AHRS_UPDATE_PERIOD);

			//Calculate new orientation
			sensors_data = lsm_inertial_read();
			orientation_angles = ahrs_orientation_update(sensors_data, AHRS_MADGWICK_2015);

			//Update PID
			if(commonPower>=10.0f) {
				targetRollRate = updatePID(rollPosPID, targetRollPos, orientation_angles.roll);
				differentialPower = updatePID(rollRatePID,targetRollRate,sensors_data.gyro.x);//orientation_angles.roll) ;
				if (differentialPower>=0) {
					 differentialPower = min(differentialPower, maxDifferentialPower) ;
				} else {
					 differentialPower = max(differentialPower, -maxDifferentialPower) ;
				}
				//esc_set_power(0,min(max(commonPower-differentialPower,0.0f),100.0f));
				//esc_set_power(1,min(max(commonPower+differentialPower,0.0f),100.0f));
				esc_set_fast_power_0_3(
						min(max(commonPower-differentialPower,0.0f),100.0f),
						min(max(commonPower+differentialPower,0.0f),100.0f),
						0.0f,
						0.0f);
			} else {
				//esc_set_power(0,0.0f);
				//esc_set_power(1,0.0f);
				esc_set_fast_power_0_3(0.0f, 0.0f, 0.0f, 0.0f);
			}
		}

		if(timeout_passed(rf_update_timer)) {
			timeout_unset(rf_update_timer);
			rf_update_timer = timeout_set(RF_UPDATE_PERIOD);
			rflink_orientation_send(orientation_angles);
		}

		if((command = rflink_command_check()) != RF_CMD_NONE) {
			timeout_unset(main_loop_timer);
			main_loop_timer = timeout_set(MAIN_LOOP_TIMEOUT);

			rflink_cmd_esc_msg_t cmd_esc;
			switch(command) {
			case RF_CMD_QUIT:
				quit = 1;
				break;
			case RF_CMD_STOP:
				esc_set_fast_power_0_3(0.0f, 0.0f, 0.0f, 0.0f);
				break;
			case RF_CMD_ESC:
				cmd_esc = rflink_read_esc();
				if (cmd_esc.percent_value < 0 || cmd_esc.percent_value > 100) {
					esc_set_power(cmd_esc.esc_position, 0.0f);
				}
				else {
					esc_set_power(cmd_esc.esc_position, cmd_esc.percent_value);
				}
				break;

			case RF_CMD_THROTTLE:
				commonPower = rflink_read_throttle();
				break;

			case RF_CMD_PITCH:
				break;

			case RF_CMD_ROLL:
				targetRollPos = rflink_read_roll();
				break;

			default:
				break;
			}
		}
	}

	timeout_unset(ahrs_update_timer);
	timeout_unset(rf_update_timer);
	timeout_unset(main_loop_timer);
	timeout_deinit();

	//les esc ne sont pas desactives mais simplement mis a 0 en oneshot 125
	//esc_deinit();
	esc_set_power(ESC_BACK_LEFT, 0.0f);
	esc_set_power(ESC_BACK_RIGHT, 0.0f);
	esc_set_power(ESC_FRONT_LEFT, 0.0f);
	esc_set_power(ESC_FRONT_RIGHT, 0.0f);


	printf("deinit uart\n");
	uart_deinit();

	mraa_result_print(mraa_i2c_stop(i2c));
	mraa_deinit();

	printf("Exit application \n");
	return MRAA_SUCCESS;
}
