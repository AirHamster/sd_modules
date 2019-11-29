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

charger_t *charger;
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
	uint8_t i;
	chRegSetThreadName("Charger Thd");
	i2cStart(&CHARGER_IF, &charger_if_cfg);
	charger_read_register(BQ2560X_REG_00, &charger->reg00);

	for (i = 0; i < 0x0B; i++) {
		chprintf(SHELL_IFACE, "Readed from charger %d %d\r\n", i,
				*(&(charger->reg00) + i));
	}

	charger_read_register(BQ2560X_REG_01, &charger->reg01);
	chprintf(SHELL_IFACE, "Readed from charger1 %d\r\n", charger->reg01);
	charger_read_register(BQ2560X_REG_02, &charger->reg02);
	chprintf(SHELL_IFACE, "Readed from charger2 %d\r\n", charger->reg02);
	charger_read_register(BQ2560X_REG_03, &charger->reg03);
	chprintf(SHELL_IFACE, "Readed from charger3 %d\r\n", charger->reg03);
	charger_read_register(BQ2560X_REG_04, &charger->reg04);
	chprintf(SHELL_IFACE, "Readed from charger4 %d\r\n", charger->reg04);
	charger_read_register(BQ2560X_REG_05, &charger->reg05);
	chprintf(SHELL_IFACE, "Readed from charger5 %d\r\n", charger->reg05);
	charger_read_register(BQ2560X_REG_06, &charger->reg06);
	chprintf(SHELL_IFACE, "Readed from charger6 %d\r\n", charger->reg06);
	charger_read_register(BQ2560X_REG_07, &charger->reg07);
	chprintf(SHELL_IFACE, "Readed from charger7 %d\r\n", charger->reg07);
	charger_read_register(BQ2560X_REG_08, &charger->reg08);
	chprintf(SHELL_IFACE, "Readed from charger8 %d\r\n", charger->reg08);
	charger_read_register(BQ2560X_REG_09, &charger->reg09);
	chprintf(SHELL_IFACE, "Readed from charger9 %d\r\n", charger->reg09);
	charger_read_register(BQ2560X_REG_0A, &charger->reg0A);
	chprintf(SHELL_IFACE, "Readed from chargerA %d\r\n", charger->reg0A);
	charger_read_register(BQ2560X_REG_0B, &charger->reg0B);
	chprintf(SHELL_IFACE, "Readed from chargerB %d\r\n", charger->reg0B);s
		systime_t prev = chVTGetSystemTime(); // Current system time.
	while (true) {
		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(1000));
	}
}

uint8_t charger_read_register(uint8_t reg_addr, uint8_t *buf) {
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
				"Shit happened withs charger: status is %d\r\n", i2cGetErrors(&CHARGER_IF));
		chSemSignal(&usart1_semaph);
		return -1;
	}
	*buf = rxbuff[0];
	return 0;
	/*chSemWait(&usart1_semaph);
	chprintf((BaseSequentialStream*) &SD1, "Readed from charger %d\r\n",
			rxbuff[0]);
	chSemSignal(&usart1_semaph);*/
}

void start_charger_module(void){
	chThdCreateStatic(charger_thread_wa, sizeof(charger_thread_wa), NORMALPRIO, charger_thread, NULL);
}
