/*
 * rflink.h
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

#ifndef RFLINK_H_
#define RFLINK_H_

#include "uart.h"
#include "../types/types.h"

#define RF_UPDATE_PERIOD 0.1f

typedef enum {
	RF_CMD_NONE = 0,
	RF_CMD_ESC = 'E',
	RF_CMD_PITCH = 'P',
	RF_CMD_QUIT = 'Q',
	RF_CMD_ROLL = 'R',
	RF_CMD_STOP = 'S',
	RF_CMD_THROTTLE = 'T',
} rf_command_t;

/* rflink_init
 * Initialize uart and rfcomm
 */
void rflink_init(void);

/* rflink_orientation_send
 * Send orientation over rf link
 */
void rflink_orientation_send(euler_angles_t);

/* rflink_command_check
 * Check for a new command from rf comm
 */
rf_command_t rflink_command_check(void);

/* rflink_read_pitch
 * Read rf comm to get a new pitch target value if RF_CMD_PITCH was received
 */
float rflink_read_pitch(void);

/* rflink_read_roll
 * Read rf comm to get a new roll target value if RF_CMD_PITCH was received
 */
float rflink_read_roll(void);

/* rflink_read_throttle
 * Read rf comm to get a new throttle target value if RF_CMD_PITCH was received
 */
float rflink_read_throttle(void);

/* rflink_get_cmd_esc
 * Read rf comm to get a new esc value if RF_CMD_ESC was received
 */
typedef struct {
	uint8_t esc_position;
	float percent_value;
} rflink_cmd_esc_msg_t;
rflink_cmd_esc_msg_t rflink_read_esc(void);


#endif /* RFLINK_H_ */
