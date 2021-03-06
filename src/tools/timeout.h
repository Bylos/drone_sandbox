/*
 * timeout.h
 * Implements 'easy to handle' timeouts for real-time management
 * within applications using Linux signals, in the way of MCU timers.
 *
 * Copyright (C) 2015  Bylos & Korky
 * Thanks to Nominal Animal <http://stackoverflow.com/users/1475978/nominal-animal>.
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

#ifndef _TIMEOUT_H_
#define _TIMEOUT_H_

#define _POSIX_C_SOURCE 200809L
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <errno.h>

/* double2timespec
 * Convenient function to convert double to timespec struct
 */
struct timespec double2timespec(double time);

/* timeout_init
 * Initialize the timeout library
 */
int timeout_init(void);

/* timeout_set
 * Set a timeout
 * Returns the timeout number if set successfully
 */
int timeout_set(const double seconds);

/* timeout_unset
 * Release the timeout.
 * Returns 0 if the timeout had not fired yet, 1 if it had.
 */
int timeout_unset(const int timeout);

/* timeout_passed
 * Return nonzero if the timeout has occurred.
 */
int timeout_passed(const int timeout);

/* timeout_deinit
 * Release the timeout library
 */
int timeout_deinit(void);

#endif /* _TIMEOUT_H_ */
