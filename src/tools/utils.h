/*
 * utils.h
 *
 *  Created on: 26 janv. 2015
 *      Author: Bylos
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <time.h>

/* Sleep until seconds or signal interrupt
 * Return 0 if wake up after seconds,
 * Return 1 if wake up on signal interrupt,
 * Return -1 if sleep is not entered
 */
int utils_sleep(double seconds);

#endif /* UTILS_H_ */
