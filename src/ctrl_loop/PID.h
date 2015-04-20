/*
 * PID.h
 * Implements assignable PID control loops
 *
 * Copyright (C) 2015  Bylos & Korky
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

#ifndef _PID_H_
#define _PID_H_


#include <stdio.h>

#define max(a,b) (a>=b?a:b)
#define min(a,b) (a<=b?a:b)


#define PID_UPDATE_PERIOD	0.01f

// PIDs

/* PID_set
 * Set a PID ;
 * returns -1 if there are no more slots available
 * returns -2, -3 and -4 if the Kp, Ki or Kd value is wrong, respectively
 * returns and int containing the ID of the set pid
 */
int PID_set(float Kp, float Ki, float Kd, float initVal);

/* PID_unset
 * Unset a PID
 * The parameter is an int containing the ID of the PID to be unset
 * Returns -1 if the ID is invalid
 * Returns -1 if the corresponding PID slot is not used
 */
int PID_unset(const int pid) ;

/* PID_update
 * Update PID value with target and actual measured value
 */
float updatePID(int pid, float targetValue, float currentValue) ;

#endif /* _PID_H_ */
