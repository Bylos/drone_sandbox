/*
uart.c
Implements UART1 (/dev/tty/MFD1) management for raw peripheral usage
on the Intel Edison platform.

Copyright (C) 2015  Bylos & Korky
Thanks to Wallyk <http://stackoverflow.com/users/198536/wallyk>.

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

#include "uart.h"

static int uart = 0;

int set_interface_attribs (int fd, int speed, int parity)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		printf ("error %d from tcgetattr\n", errno);
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
								// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
								// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag |= CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		printf ("error %d from tcsetattr\n", errno);
		return -1;
	}
	return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tggetattr\n", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes\n", errno);
}

/* Initialize and open UART communication
 * Return uart handle if successful
 * baudrate is set to 115200
 */
int uart_init(void) {
	mraa_uart_context uart_dev = NULL;
	char *uart_dev_path = NULL;

	if ((uart_dev = mraa_uart_init(0)) == NULL) {
		printf("ERROR : UART\t mraa uart init failed\n");
		return -1;
	}
	if ((uart_dev_path = mraa_uart_get_dev_path(uart_dev)) == NULL) {
		printf("ERROR : UART\t mraa uart path does not exist");
		return -1;
	}
	if((uart = open(uart_dev_path, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
		printf("ERROR : UART\t opening serial device");
		return -1;
	}
	set_interface_attribs(uart, B115200, 0);
	set_blocking(uart, 0);

	printf("UART successfully initialized on %s\n", uart_dev_path);

	tcflush(uart, TCIOFLUSH);
	return 0;
}

/* Close UART communication
 * Return 0 if successful
 */
int uart_deinit(void) {
	return close(uart);
}

/* Write byte buffer on uart
 * Return Number of bytes written
 */
 int uart_write(char *buffer, uint16_t length) {
	 if (uart == 0 || length == 0) {
		 return -1;
	 }
	 return write(uart, buffer, length);
 }

 int uart_read(char *buffer, uint16_t length) {
	 if (uart == 0 || length == 0) {
		 return -1;
	 }
	 return read(uart, buffer, length);
 }

/* Get number of bytes available on port
* Return number of bytes available
*/
int uart_bytesAvailable(void) {
	if(uart == 0) {
		return -1;
	}
	int bytes;
	ioctl(uart, FIONREAD, &bytes);
	return bytes;
}
