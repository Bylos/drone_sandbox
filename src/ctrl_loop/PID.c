/*
 * PID.c
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

#include "PID.h"

#define   PIDS       16
#define   PID_USED   1
#define   PID_UNUSED 0

static volatile int pid_state[PIDS] = { 0 };
static float PID_Kp[PIDS];
static float PID_Ki[PIDS];
static float PID_Kd[PIDS];
static float PID_Err[PIDS];
static float PID_Int[PIDS];
static float PID_MaxInt[PIDS];
static float PID_Der[PIDS];
static float PID_Val[PIDS];


/* Set a PID ;
 * returns -1 if there are no more slots available
 * returns -2, -3 and -4 if the Kp, Ki or Kd value is wrong, respectively
 * returns and int containing the ID of the set pid
 */
int PID_set(float Kp, float Ki, float Kd, float initVal, float maxInt) {

	 int pid ;

	 /* Find an unused PID slot and assign it. */
	 for (pid = 0; pid < PIDS; pid++)
		 if (!(__sync_fetch_and_or(&pid_state[pid], PID_USED) & PID_USED))
			 break;

	 /* No unused PIDS? */
	 if (pid >= PIDS)
		 return -1;

	 if (Kp<0) return -2;
	 PID_Kp[pid] = Kp ;

	 if (Ki<0) return -3;
	 PID_Ki[pid] = Ki ;
	 PID_MaxInt[pid] = maxInt ;

	 if (Kd<0) return -4;
	 PID_Kd[pid] = Kd ;

	 PID_Int[pid] = PID_Der[pid] = PID_Err[pid] = 0.0 ;
	 PID_Val[pid] = initVal;

	 return pid ;
}

/*
 * Unset a PID
 * The parameter is an int containing the ID of the PID to be unset
 * Returns -1 if the ID is invalid
 * Returns -1 if the corresponding PID slot is not used
 */
int PID_unset(const int pid) {
	    if (pid >= 0 && pid < PIDS) {
	        /* Only keep TIMEOUT_PASSED for the specified timer. */
	        const int state = __sync_fetch_and_and(&pid_state[pid], PID_UNUSED);

	        /* Reset the pid parameters */
	        PID_Kp[pid] = PID_Ki[pid] = PID_Kd[pid] = PID_Int[pid]= PID_Val[pid] = PID_Der[pid] = PID_Err[pid] = PID_MaxInt[pid] = 0.0 ;

	        /* Returns -1 if pid was already not used and 1 if it has been deactivated */
	        if (state)
	            return 1;
	        else
	        	return -1 ;

	    } else {
	        /* Invalid timeout number. */
	        return -1;
	    }
}


float updatePID(int pid, float targetValue, float currentValue) {
	 float error = targetValue -currentValue ;

	 PID_Int[pid] = min(PID_Int[pid] + error*PID_UPDATE_PERIOD,PID_MaxInt[pid]) ;
	 PID_Der[pid] = (error - PID_Err[pid])/PID_UPDATE_PERIOD ;

	 PID_Val[pid] = PID_Kp[pid]*error+PID_Ki[pid]*PID_Int[pid]+PID_Kd[pid]*PID_Der[pid] ;
	 PID_Err[pid] = error ;
	 return PID_Val[pid] ;
}
