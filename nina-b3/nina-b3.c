/*
 * nina-b3.c
 *
 *  Created on: Apr 26, 2019
 *      Author: a-h
 */

#include "nina-b3.h"
#include "config.h"
#include <stdlib.h>
#include <string.h>
#include "stdint.h"
#include "math.h"
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

static const SerialConfig nina_config =
		{ 115200, 0, USART_CR2_STOP1_BITS, 0 };

static void nina_init_module(void);
static void nina_get_discoverable_status(void);
static THD_WORKING_AREA(ble_thread_wa, 4096);
static THD_FUNCTION(ble_thread, arg);

static void nina_get_discoverable_status(void){
	chprintf(NINA_IFACE, "at\r");
}

void start_ble_module(void){
	chThdCreateStatic(ble_thread_wa, sizeof(ble_thread_wa), NORMALPRIO + 1,
			ble_thread, NULL);
}


/*
 * Thread to process data collection and filtering from NEO-M8P
 */

static THD_FUNCTION(ble_thread, arg) {
	(void) arg;
	uint8_t token;
	chRegSetThreadName("BLE Thread");

	nina_init_module();
	systime_t prev = chVTGetSystemTime(); // Current system time.
	while (true) {
		token = sdGet(&NINA_IF);
		chprintf((BaseSequentialStream*) &SD1, "Token: %c\r\n", token);
	//	palToggleLine(LINE_ORANGE_LED);
		//prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(500));
	}
}

static void nina_init_module(void){
	chThdSleepMilliseconds(500);
	sdStart(&NINA_IF, &nina_config);
	palSetLine(LINE_ORANGE_LED);
	chThdSleepMilliseconds(500);
	//palClearLine(LINE_ORANGE_LED);
	chThdSleepMilliseconds(500);
	nina_get_discoverable_status();
}
