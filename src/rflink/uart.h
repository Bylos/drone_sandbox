/*
 * uart.h
 *
 *  Created on: 26 janv. 2015
 *      Author: Bylos
 */

#ifndef UART_H_
#define UART_H_

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

/* Initialize and open UART communication
 * Return uart handle if successful
 * baudrate is set to 115200
 */
int uart_init(void);

/* Close UART communication
 * Return 0 if successful
 */
int uart_deinit(void);

/* Write byte buffer on uart
* Return Number of bytes written
*/
int uart_write(char *buffer, uint16_t length);

int uart_read(char *buffer, uint16_t length);

/* Get number of bytes available on port
* Return number of bytes available
*/
int uart_bytesAvailable(void);

#endif /* UART_H_ */
