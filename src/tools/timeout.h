/*
timeout.h
Implements 'easy to handle' timeouts for real-time management
within applications using Linux signals, in the way of MCU timers.

Copyright (C) 2015  Bylos & Korky
Thanks to Nominal Animal <http://stackoverflow.com/users/1475978/nominal-animal>.

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

#ifndef TIMEOUT_H_
#define TIMEOUT_H_

#define _POSIX_C_SOURCE 200809L
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include "timeout.h"

/* Initialize the timeout library
 */
int timeout_init(void);

/* Set a timeout
 * Returns the timeout number if set successfully
 */
int timeout_set(const double seconds);

/* Release the timeout.
 * Returns 0 if the timeout had not fired yet, 1 if it had.
*/
int timeout_unset(const int timeout);

/* Return nonzero if the timeout has occurred.
*/
int timeout_passed(const int timeout);

/* Release the timeout library
 */
int timeout_deinit(void);

/* Convenient function to convert double to time_spec_t
 */
struct timespec double2timespec(double time);

#endif /* TIMEOUT_H_ */
