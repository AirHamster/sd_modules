/*
 * lag.h
 *
 *  Created on: Sep 9, 2019
 *      Author: a-h
 */

/**
 * @file    lag.h
 * @brief   Log Driver structs.
 *
 * @addtogroup LOG
 * @{
 */

#ifndef SD_MODULES_LAG_LAG_H_
#define SD_MODULES_LAG_LAG_H_

#define LAG_MAGIC_CONST		0.1f

/**
 * @struct lag_t
 * @brief Common log struct
 * @var lag_t::calib_num
 * Calibration coefficient
 * @var lag_t::hz
 * Measurement in herts
 * @var lag_t::meters
 * Measurements in meters/sec
 * @var lag_t::millis
 * Measurements in millis period
 */
typedef struct{
	float calib_num;
	float hz;
	float meters;
	uint16_t millis;
	uint8_t update_flag;
	uint8_t rtc_cnt;
	int8_t temperature;
}lag_t;

void start_lag_module(void);

#endif /* SD_MODULES_LAG_LAG_H_ */
