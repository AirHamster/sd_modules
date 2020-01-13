/*
 * mcu-mcu_i2c.c
 *
 *  Created on: Nov 26, 2019
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
#include "mcu-mcu_i2c.h"
#include "software_uart.h"
#include "bq27441.h"

extern fuel_t *fuel;
mcu_mcu_data_t *mcu_mcu_data;
power_data_t power;
power_data_t *power_data = &power;

#ifdef MAIN_MCU
const I2CConfig mcu_mcu_if_cfg = {
  0xD0D43C4C,
  0,
  0
};
#endif

#ifdef POWER_MCU
const I2CConfig mcu_mcu_if_cfg = {
  0x10E37AFF,
  0,
  0
};
#endif

static THD_WORKING_AREA(mcu_mcu_thread_wa, 256);
static THD_FUNCTION( mcu_mcu_thread, p) {
	(void) p;
	chRegSetThreadName("MCU-MCU Thd");
	//i2cStart(&MCU_MCU_IF, &mcu_mcu_if_cfg);
	susart_init();
	char char_arr[4];
	while (true) {

		systime_t prev = chVTGetSystemTime(); // Current system time.
		//mcu_mcu_read_power_parameters(power_data);
		//chprintf((BaseSequentialStream*) &SD1, "Sending S\r\n");
		itoa(fuel->voltage, char_arr, 10);
		//chprintf(SHELL_IFACE, "\r\nATOI:\t%d %s\r\n", fuel->voltage);
		_putchar('V');
		_putchar(char_arr[0]);
		_putchar(char_arr[1]);
		_putchar(char_arr[2]);
		_putchar(char_arr[3]);
		itoa(fuel->soc, char_arr, 10);


		_putchar('S');

		if (fuel->soc < 100) {
			_putchar(char_arr[0]);
			_putchar(char_arr[1]);
		} else {
			_putchar(char_arr[0]);
			_putchar(char_arr[1]);
			_putchar(char_arr[2]);
		}

		_putchar('\r');
		_putchar('\n');

		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(1000));

#ifdef SLAVE_MCU
		//needed to patch ChibiOS code for support slave configuration
		//result = i2cSlaveRecieveTimeout(TIMEOUT_INFINITE);
#endif

	}
}

void start_mcu_mcu_module(void){
	chThdCreateStatic(mcu_mcu_thread_wa, sizeof(mcu_mcu_thread_wa), NORMALPRIO + 3, mcu_mcu_thread, NULL);
}

#ifdef MAIN_MCU

static int8_t mcu_mcu_read_power_data(power_data_t *p_data){
	mcu_mcu_read_from_slave(POWER_MCU_DATA, (uint8_t*)data, sizeof(power_data_t));

}

static int8_t mcu_mcu_read_from_slave(uint8_t reg_addr, uint8_t *buf, uint8_t num){
	uint8_t txbuff[1];
		uint8_t rxbuff[10];
		msg_t status;
		txbuff[0] = reg_addr;
		i2cAcquireBus(&POWER_MCU_IF);
		status = i2cMasterTransmitTimeout(&POWER_MCU_IF, POWER_SLAVE_ADDRESS, txbuff, 1,
				rxbuff, num, 1000);
		i2cReleaseBus(&POWER_MCU_IF);
		if (status != MSG_OK) {
			chSemWait(&usart1_semaph);
			chprintf((BaseSequentialStream*) &SD1,
					"Shit happened in mcu-mcu comm: status is %d\r\n", i2cGetErrors(&CHARGER_IF));
			chSemSignal(&usart1_semaph);
			return -1;
		}
		memcpy(buf, rxbuff, num);
		return 0;
}

static int8_t mcu_mcu_send_data_packet_to_slave(mcu_mcu_data_t *data) {
	uint8_t txbuff[sizeof(mcu_mcu_data_t)+1];
	uint8_t rxbuff[1];
	msg_t status;
	//txbuff[0] = reg_addr;
	txbuff[0] = DATA;
	memcpy(&txbuff[1], data, sizeof(mcu_mcu_data_t));
	i2cAcquireBus(&MCU_MCU_IF);
	status = i2cMasterTransmitTimeout(&MCU_MCU_IF, MCU_MCU_SLAVE_ADDRESS, txbuff, sizeof(mcu_mcu_data_t)+1, NULL,
			0, 1000);
	i2cReleaseBus(&MCU_MCU_IF);

	if (status != MSG_OK) {
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*) &SD1,
				"Shit happened in mcu-mcu comm: status is %d\r\n",
				i2cGetErrors(&MCU_MCU_IF));
		chSemSignal(&usart1_semaph);
		return -1;
	}
}

static int8_t mcu_mcu_send_message(msg_reason_enum msg){
	uint8_t txbuff;
	//uint8_t rxbuff[num];
	msg_t status;
	//txbuff[0] = reg_addr;
	txbuff = msg;

	i2cAcquireBus(&MCU_MCU_IF);
	status = i2cMasterTransmitTimeout(&MCU_MCU_IF, MCU_MCU_SLAVE_ADDRESS, &txbuff, 1, NULL,
			0, 1000);
	i2cReleaseBus(&MCU_MCU_IF);

	if (status != MSG_OK) {
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*) &SD1,
				"Shit happened in mcu-mcu comm: status is %d\r\n",
				i2cGetErrors(&MCU_MCU_IF));
		chSemSignal(&usart1_semaph);
		return -1;
	}
}

#endif


#ifdef SLAVE_MCU
static int8_t mcu_mcu_parse_data_packet_from_master(uint8_t *data, mcu_mcu_data_t *data_struct){

	memcpu(data_struct, data, sizeof(mcu_mcu_data_t));

}


static int8_t mcu_mcu_parse_message_from_master(uint8_t *data){

	switch(data[0]){
	case DATA:
		mcu_mcu_parse_data_packet_from_master(&data[1], mcu_mcu_data);
		break;
	case STARTUP_NORMAL:
		//start_threads(ALL);
		break;
	case STARTUP_CHARGE:
		//start_threads(CHARGING);
		break;
	case STARTUP_BAT_LOW:
		//start_threads(BAT_LOW);
		break;
	case SHUTDOWN:
		//stop_threads(ALL);
		break;
	case KEY_PRESS:
		//process_keypress(data[1]);
		break;
	default:
		break;
	}

}
#endif
