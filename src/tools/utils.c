/*
 * sleep.c
 *
 *  Created on: 26 janv. 2015
 *      Author: Bylos
 */


#include "utils.h"

/* Sleep until seconds or signal interrupt
 * Return 0 if wake up after seconds,
 * Return 1 if wake up on signal interrupt,
 * Return -1 if sleep is not entered
 */
int utils_sleep(double seconds) {
	struct timespec sleep_time;
	if (seconds > 0.0) {
		const long  s = (long)seconds;
		long       ns = (long)(0.5 + 1000000000.0 * (seconds - (double)s));

		if (ns < 0L) {
			ns = 0L;
		}
		else {
			if (ns > 999999999L)
				ns = 999999999L;
		}
		sleep_time.tv_sec = (time_t)s;
		sleep_time.tv_nsec = ns;
	}
	else {
		return -1;
	}
	return clock_nanosleep(CLOCK_REALTIME, 0, &sleep_time, NULL);
}
