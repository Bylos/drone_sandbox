/*
utils.h
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
