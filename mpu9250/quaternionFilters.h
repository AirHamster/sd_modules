/*
 * quaternionFilters.h
 *
 *  Created on: Mar 15, 2019
 *      Author: a-h
 */

#ifndef _QUATERNIONFILTERS_H_
#define _QUATERNIONFILTERS_H_

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                              float gz, float mx, float my, float mz,
                              float deltat);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                            float gz, float mx, float my, float mz,
                            float deltat);
const float * getQ();

#endif // _QUATERNIONFILTERS_H_
