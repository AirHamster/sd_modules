/*
 * mcu-mcu_i2c.h.h
 *
 *  Created on: Nov 26, 2019
 *      Author: a-h
 */

#ifndef SD_MODULES_MCU-MCU_H_
#define SD_MODULES_MCU-MCU_H_

#include "config.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"


#define MCU_MCU_SLAVE_ADDR	0x10

typedef enum{
	STARTUP_NORMAL = 0,
	STARTUP_CHARGE,
	STARTUP_BAT_LOW,
	SHUTDOWN,
	DATA,
	KEY_PRESS
}msg_reason_enum;

typedef struct{
	int16_t voltage;
	int16_t current;
	int16_t soc;
	int16_t capacity;
}mcu_mcu_data_t;

#endif	//SD_MODULES_MCU-MCU_H_
