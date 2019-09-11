/*
 * lag.h
 *
 *  Created on: Sep 9, 2019
 *      Author: a-h
 */

#ifndef SD_MODULES_LAG_LAG_H_
#define SD_MODULES_LAG_LAG_H_

typedef struct{
	uint16_t hz;
	uint16_t meters;
	uint16_t millis;
	uint8_t update_flag;
	uint8_t rtc_cnt;
	int8_t temperature;
}lag_t;

void start_lag_module(void);

#endif /* SD_MODULES_LAG_LAG_H_ */
