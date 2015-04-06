/*
 * ahrs.h
 *
 *  Created on: 23 févr. 2015
 *      Author: Bylos
 */

#ifndef AHRS_H_
#define AHRS_H_

#include <math.h>

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError 3.14159265359f * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift 3.14159265359f * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
//#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define beta_min 0.1f
#define beta_max 10.0f
#define beta_rate 0.1f

//#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

#define AHRS_UPDATE_PERIOD	0.01f

extern volatile float beta;
extern volatile float q1, q2, q3, q4;
extern volatile float yaw, pitch, roll;

void ahrs_BetaUpdate(void);
void ahrs_Madgwick2014(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void ahrs_Madgwick2015(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void ahrs_MadgwickIMU (float ax, float ay, float az, float gx, float gy, float gz);

void ahrs_Quaternion2Euler(void);

#endif /* AHRS_H_ */
