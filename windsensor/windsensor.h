/*
 * windsensor.h
 *
 *  Created on: Aug 10, 2019
 *      Author: a-h
 */

#include <stdint.h>
#include "ch.h"
#include "hal.h"
#ifndef SD_MODULES_WINDSENSOR_WINDSENSOR_H_
#define SD_MODULES_WINDSENSOR_WINDSENSOR_H_

#define UART_CHAR_RECEIVED		1
#define UART_GENERIC_NOTIFY		2
#define UART_ERROR				3

/**
 * Start windsensor threads
 */
void start_windsensor_module(void);

typedef struct{
	float speed;
	int16_t direction;
}windsensor_t;

#endif /* SD_MODULES_WINDSENSOR_WINDSENSOR_H_ */
