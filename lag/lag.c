/*
 * lag.c
 *
 *  Created on: Sep 9, 2019
 *      Author: a-h
 */

/**
 * @file    lag.c
 * @brief   Log Driver funcs.
 *
 * @addtogroup LOG
 * @{
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

/**
 * @brief Start log threads
 */
void start_lag_module(void){

	chThdCreateStatic(lag_thread_wa, sizeof(lag_thread_wa), NORMALPRIO,  lag_thread, NULL);
}


/**
 * @brief Thread to process data collection and filtering from MPU9250
 */
static THD_FUNCTION(lag_thread, arg) {
	(void) arg;
	chRegSetThreadName("LAG Thread");
	lag_init_pins();
	systime_t prev = chVTGetSystemTime(); // Current system time.
	while (true) {
		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(100));
		lag->hz = (float)(1000.0 / lag->millis);
		lag->meters = (float)lag->hz * lag->calib_num / 2;
	//	chprintf((BaseSequentialStream*) &SD1, "LAG: %f, %f\r\n", lag->meters, lag->hz);
	}
}

/**
 *  @brief Callback associated to the impulse .
 */
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

/**
 * @brief Measurement pins initialization
 */
static void lag_init_pins(void){
	/* Enabling event on falling edge of PA0 signal.*/
#ifdef SD_SENSOR_BOX_LAG
	eeprom_read(EEPROM_LAG_CALIB_NUMBER, (uint8_t*)&lag->calib_num, 4);
	chprintf((BaseSequentialStream*) &SD1, "Readed calib number: %f\r\n", lag->calib_num);
	  palEnablePadEvent(GPIOA, GPIOA_ADC_IN1, PAL_EVENT_MODE_RISING_EDGE);

	 /* Assigning a callback to PA0 passing no arguments.*/
	  palSetPadCallback(GPIOA, GPIOA_ADC_IN1, lag_callback, NULL);
#endif
}
