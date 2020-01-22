/*
 * bq2560X.c
 *
 *  Created on: Sep 12, 2019
 *      Author: a-h
 */
#include "config.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"
#include "bq27441.h"
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

fuel_t *fuel;
extern struct ch_semaphore usart1_semaph;

#ifdef SD_MODULE_TRAINER
const I2CConfig fuel_if_cfg = {
  0xD0D43C4C,
  0,
  0
};
#endif

#ifdef SENSOR_BOX
const I2CConfig fuel_if_cfg = {
  0x10E37AFF,
  0,
  0
};
#endif

static uint8_t fuel_read_register(uint8_t command, uint8_t *buf, uint8_t num);
static uint8_t fuel_get_parameter(uint8_t param, uint16_t *buffer);

static THD_WORKING_AREA(fuel_thread_wa, 256);
static THD_FUNCTION(fuel_thread, p) {
	(void) p;
	chRegSetThreadName("Fuel gauge Thd");
	//i2cStart(&FUEL_IF, &fuel_if_cfg);

	while (true) {
		systime_t prev = chVTGetSystemTime(); // Current system time.
		fuel_get_parameter(BQ27441_COMMAND_VOLTAGE, &fuel->voltage);
		fuel_get_parameter(BQ27441_COMMAND_FLAGS, &fuel->flags);
		fuel_get_parameter(BQ27441_COMMAND_SOC, &fuel->soc);
		fuel_get_parameter(BQ27441_COMMAND_REM_CAPACITY, &fuel->remaining_capacity);
		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(1000));

	}
}

static uint8_t fuel_get_parameter(uint8_t param, uint16_t *buffer){
	uint8_t rx_buff[2];
		fuel_read_register(param, rx_buff, 2);
		*buffer = (rx_buff[1] << 8) | rx_buff[0];

		/*chSemWait(&usart1_semaph);
		chprintf(SHELL_IFACE, "Readed from fuel_gauge: %x\r\n\r\n", buffer);
		chSemSignal(&usart1_semaph);*/
		return 0;
}


static uint8_t fuel_read_register(uint8_t command, uint8_t *buf, uint8_t num){
	uint8_t txbuff[2];
	uint8_t rxbuff[num];
	msg_t status;
	//txbuff[0] = reg_addr;
	txbuff[0] = command;
	i2cAcquireBus(&FUEL_IF);
	status = i2cMasterTransmitTimeout(&FUEL_IF, FUEL_ADDRESS, txbuff, 1,
			rxbuff, num, 1000);
	i2cReleaseBus(&FUEL_IF);
	if (status != MSG_OK) {
		/*chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*) &SD1,
				"Shit happened fuel: status is %d\r\n", i2cGetErrors(&FUEL_IF));
		chSemSignal(&usart1_semaph);*/
		return -1;
	}
	memcpy(buf, rxbuff, num);
	/*chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*) &SD1, "Readed from charger %d\r\n",
				rxbuff[0]);
		chSemSignal(&usart1_semaph);*/
	return 0;

}

void start_fuel_gauge_module(void){
	chThdCreateStatic(fuel_thread_wa, sizeof(fuel_thread_wa), NORMALPRIO, fuel_thread, NULL);
}
