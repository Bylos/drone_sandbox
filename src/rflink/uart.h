/*
 * uart.h
 * Implements UART1 (/dev/tty/MFD1) management for raw peripheral usage
 * on the Intel Edison platform.
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

#ifndef _UART_H_
#define _UART_H_

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <linux/serial.h>
#include <mraa.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

/* uart_init
 * Initialize and open UART communication
 * Return uart handle if successful
 * baudrate is set to 115200
 */
int uart_init(void);

/* uart_deinit
 * Close UART communication
 * Return 0 if successful
 */
int uart_deinit(void);

/* uart_write
 * Write byte buffer on uart
 * Return number of bytes written
 */
int uart_write(char *buffer, uint16_t length);

/* uart_read
 * Read byte buffer from uart
 * Return number of bytes read
 */
int uart_read(char *buffer, uint16_t length);

/* uart_bytesAvailable
 * Get number of bytes available on port
 * Return number of bytes available
 */
int uart_bytesAvailable(void);

#endif /* _UART_H_ */
