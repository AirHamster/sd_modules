/*
 * bq2560x.c
 *
 *  Created on: Sep 12, 2019
 *      Author: a-h
 */
#include "config.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"
#include "bq2560x.h"
#ifdef USE_UBLOX_GPS_MODULE
#include "neo-m8.h"
extern ubx_nav_pvt_t *pvt_box;
#endif
#ifdef USE_BNO055_MODULE
#include "bno055.h"
#include "bno055_i2c.h"
extern bno055_t *bno055;
#endif
#ifdef USE_WINDSENSOR_MODULE
#include "windsensor.h"
extern windsensor_t *wind;
#endif

extern struct ch_semaphore usart1_semaph;

#ifdef SD_MODULE_TRAINER
const I2CConfig charger_if_cfg = {
  0xD0D43C4C,
  0,
  0
};
#endif

#ifdef SENSOR_BOX
const I2CConfig charger_if_cfg = {
  0x10E37AFF,
  0,
  0
};
#endif

static THD_WORKING_AREA(charger_thread_wa, 256);
static THD_FUNCTION( charger_thread, p) {
	(void) p;
	msg_t msg;
	chRegSetThreadName("Charger Thd");
	i2cStart(&CHARGER_IF, &charger_if_cfg);
	charger_read_register(BQ2560X_REG_0B);
		systime_t prev = chVTGetSystemTime(); // Current system time.
	while (true) {
		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(1000));
	}
}

uint8_t charger_read_register(uint8_t reg_addr) {
	uint8_t txbuff[1];
	uint8_t rxbuff[1];
	msg_t status;
	txbuff[0] = reg_addr;
	i2cAcquireBus(&CHARGER_IF);
	status = i2cMasterTransmitTimeout(&CHARGER_IF, CHARGER_ADDRESS, txbuff, 1,
			rxbuff, 1, 1000);
	i2cReleaseBus(&CHARGER_IF);
	if (status != MSG_OK) {
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*) &SD1,
				"Shit happened charger: status is %d\r\n", i2cGetErrors(&CHARGER_IF));
		chSemSignal(&usart1_semaph);
	}
	chSemWait(&usart1_semaph);
	chprintf((BaseSequentialStream*) &SD1, "Readed from charger %d\r\n",
			rxbuff[0]);
	chSemSignal(&usart1_semaph);
}

void start_charger_module(void){
	chThdCreateStatic(charger_thread_wa, sizeof(charger_thread_wa), NORMALPRIO, charger_thread, NULL);
}
