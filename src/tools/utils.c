/*
utils.c
Provides a convenient sleep function based on Linux "nanosleep", which can be
interrupted by signals (eg. for timers)

Copyright (C) 2015  Bylos & Korky

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
