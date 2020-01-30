/*
 * tenso.c
 *
 *  Created on: Jan 30, 2020
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
#include "adc.h"
#include "tenso.h"

#ifdef USE_EEPROM_MODULE
#include "eeprom.h"
#endif

tenso_data_t *tenso;

static THD_WORKING_AREA(tenso_thread_wa, 256);
static THD_FUNCTION( tenso_thread, p) {
	(void) p;
	chRegSetThreadName("Tenso Thd");
	systime_t prev = chVTGetSystemTime(); // Current system time.
	uint32_t tmp;
	//tenso_read_coefs_from_eeprom;
	while (true) {
		 tenso->adc_native = adc_get_avg_measure(ADC_CH1, 30);
//		 tenso->newtons = tenso->adc_native * tenso->coef_newton;
//		 tenso->coef_kilograms = tenso->adc_native * tenso->coef_kilograms;
		 chprintf((BaseSequentialStream*) &SD1, "ADC native:  %d\r\n",
					(uint16_t)tenso->adc_native);
		/*
			chprintf((BaseSequentialStream*) &SD1, "ADC percent: %f\r\n",
					rudder->percent);
			chprintf((BaseSequentialStream*) &SD1, "ADC degrees: %f\r\n",
					rudder->degrees);*/
			prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(100));
		}

}

void start_tenso_module(void){
	//adcStart(&ADCD1, NULL);
	//adcSTM32EnableVREF(&ADCD1);
	chThdCreateStatic(tenso_thread_wa, sizeof(tenso_thread_wa), NORMALPRIO, tenso_thread, NULL);
}
