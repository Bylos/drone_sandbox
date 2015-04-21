/*
 * rflink.c
 * Implements high level receive and send commands through rf link
 *
 * Copyright (C) 2015  Bylos & Korky
 * Thanks to Wallyk <http://stackoverflow.com/users/198536/wallyk>.
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

#include "rflink.h"

/* rflink_init
 * Initialize uart and rfcomm
 */
void rflink_init(void) {
	uart_init();
}

/* rflink_orientation_send
 * Send orientation over rf link
 */
void rflink_orientation_send(euler_angles_t angles) {
	char s_buffer[8];
	sprintf(s_buffer, "Y%+06.1f", angles.yaw);
	uart_write(s_buffer, 7);
	sprintf(s_buffer, "P%+06.1f", angles.pitch);
	uart_write(s_buffer, 7);
	sprintf(s_buffer, "R%+06.1f", angles.roll);
	uart_write(s_buffer, 7);
	uart_write("\n",1);
}

rf_command_t rflink_command_check(void) {
	char command = RF_CMD_NONE;
	while (uart_read(&command, 1) > 0) {
		if (command >= 'A' && command <='Z') {
			return command;
		}
	}
	return RF_CMD_NONE;
}

/*void read_esc_power(void) {
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
}*/
