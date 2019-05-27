/*
 * quaternionFilters.h
 *
 *  Created on: Mar 15, 2019
 *      Author: a-h
 */

#ifndef _QUATERNIONFILTERS_H_
#define _QUATERNIONFILTERS_H_

// parameters for 6 DoF sensor fusion calculations
#define CONST_PI		3.14159265358979323846f
#define CONST_GME		CONST_PI * (60.0f / 180.0f)
#define CONST_beta		sqrt(3.0f / 4.0f) * CONST_GME
#define CONST_GMD		CONST_PI * (1.0f / 180.0f)
#define CONST_zeta		sqrt(3.0f / 4.0f) * CONST_GMD

#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f
//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	512.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions


void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                              float gz, float mx, float my, float mz,
                              float deltat);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                            float gz, float mx, float my, float mz,
                            float deltat);
void  FreeIMUAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
float invSqrt(float number);
//const float * getQ();

#endif // _QUATERNIONFILTERS_H_
