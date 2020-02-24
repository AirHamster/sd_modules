/*
 * mcu-mcu_i2c.h.h
 *
 *  Created on: Nov 26, 2019
 *      Author: a-h
 */

/**
 * @file    mcu-mcu_i2c.h
 * @brief   Multi MCU communication enums and structs.
 *
 * @addtogroup MCU-MCU
 * @{
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

void start_mcu_mcu_module(void);

/**
 * @enum msg_reason_enum
 * @brief Messages to slave MCU
 */
typedef enum{
	STARTUP_NORMAL = 0,//!< STARTUP_NORMAL
	STARTUP_CHARGE,    //!< STARTUP_CHARGE
	STARTUP_BAT_LOW,   //!< STARTUP_BAT_LOW
	SHUTDOWN,          //!< SHUTDOWN
	DATA,              //!< DATA
	KEY_PRESS          //!< KEY_PRESS
}msg_reason_enum;

/**
 * @struct mcu_mcu_data_t
 * @brief Battery data
 */
typedef struct{
	int16_t voltage;
	int16_t current;
	int16_t soc;
	int16_t capacity;
}mcu_mcu_data_t;

/**
 * @struct power_data_t
 * @brief Power data
 */
typedef struct{
	int16_t current;
	int16_t voltage;
	int8_t soc;
}power_data_t;

#endif	//SD_MODULES_MCU-MCU_H_
