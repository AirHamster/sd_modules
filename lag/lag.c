/*
 * lag.c
 *
 *  Created on: Sep 9, 2019
 *      Author: a-h
 */
#include "config.h"
#include <stdlib.h>
#include <string.h>
#include "stdint.h"
#include "math.h"
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"
#ifdef USE_BNO055_MODULE
#include "bno055.h"
#include "bno055_i2c.h"
#endif
#include "sd_shell_cmds.h"
#include "eeprom.h"
#include "lag.h"

static void lag_init_pins(void);
extern struct ch_semaphore usart1_semaph;
lag_t *lag;
extern time_measurement_t time;

static THD_WORKING_AREA(lag_thread_wa, 512);
static THD_FUNCTION(lag_thread, arg);

void start_lag_module(void){

	chThdCreateStatic(lag_thread_wa, sizeof(lag_thread_wa), NORMALPRIO,  lag_thread, NULL);
}


/*
 * Thread to process data collection and filtering from MPU9250
 */

static THD_FUNCTION(lag_thread, arg) {
	(void) arg;
	chRegSetThreadName("LAG Thread");
	lag_init_pins();
	systime_t prev = chVTGetSystemTime(); // Current system time.
	while (true) {

		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(100));
		lag->hz = (uint16_t)(1000.0 / lag->millis);
		chprintf((BaseSequentialStream*) &SD1, "LAG: %d, %d\r\n", lag->millis, lag->hz);
	}
}

/* Callback associated to the event. */
static void lag_callback(void *arg) {
  (void)arg;
  chTMStopMeasurementX(&time);
//  lag->rtc_cnt = time.last;
  lag->millis = RTC2MS(STM32_SYSCLK, time.last);
 // chSemWait(&usart1_semaph);
/*  chprintf((BaseSequentialStream*) &SD1,
  				"LAG calback: %d\r\n", TIME_I2MS(lag->time.last));*/
 // chprintf((BaseSequentialStream*) &SD1, "LAG calback: %d\r\n");
 // chSemSignal(&usart1_semaph);
  chTMStartMeasurementX(&time);
}

static void lag_init_pins(void){
	/* Enabling event on falling edge of PA0 signal.*/
	  palEnablePadEvent(GPIOA, GPIOA_ADC_IN1, PAL_EVENT_MODE_RISING_EDGE);

	 /* Assigning a callback to PA0 passing no arguments.*/
	  palSetPadCallback(GPIOA, GPIOA_ADC_IN1, lag_callback, NULL);
}
