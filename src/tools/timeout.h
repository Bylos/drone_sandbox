/*
 * timeout.h
 *
 *  Created on: 25 janv. 2015
 *      Author: Bylos
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
