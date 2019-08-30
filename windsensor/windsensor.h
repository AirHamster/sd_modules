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

void start_windsensor_module(void);
static void windstation_char_recieved_async(UARTDriver *uartp, uint16_t c);
static void windstation_rx_buff_filled(UARTDriver *uartp);
static void windstation_rx_error(UARTDriver *uartp, uartflags_t e);
//static THD_FUNCTION(wind_thread, p);

typedef struct{
	float speed;
	uint16_t direction;
}windsensor_t;

#endif /* SD_MODULES_WINDSENSOR_WINDSENSOR_H_ */
