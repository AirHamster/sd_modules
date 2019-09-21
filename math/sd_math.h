/*
 * sd_math.h
 *
 *  Created on: Sep 19, 2019
 *      Author: a-h
 */

#include "sailDataMath.h"
#ifndef SD_MODULES_MATH_SD_MATH_H_
#define SD_MODULES_MATH_SD_MATH_H_
void math_copy_sensor_values(float *lastSensorValues);
void math_init_calibration_params(CalibrationParmDef *param);
void start_math_module(void);
#endif /* SD_MODULES_MATH_SD_MATH_H_ */
