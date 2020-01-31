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
		 tenso->kilograms = tenso_calculate_kilograms(tenso);
		 tenso->newtons = tenso_calculate_newtons(tenso);
//		 tenso->newtons = tenso->adc_native * tenso->coef_newton;
//		 tenso->coef_kilograms = tenso->adc_native * tenso->coef_kilograms;
		 chprintf((BaseSequentialStream*) &SD1, "ADC native:  %d\r\n",
					(uint16_t)tenso->adc_native);

			chprintf((BaseSequentialStream*) &SD1, "ADC kgs: %f\r\n",
					tenso->kilograms);
			chprintf((BaseSequentialStream*) &SD1, "ADC newtons: %f\r\n",
					tenso->newtons);
			prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(1000));
		}
}

void start_tenso_module(void){
	//adcStart(&ADCD1, NULL);
	//adcSTM32EnableVREF(&ADCD1);
	chThdCreateStatic(tenso_thread_wa, sizeof(tenso_thread_wa), NORMALPRIO, tenso_thread, NULL);
}

int8_t tenso_read_coefs_from_eeprom(tenso_data_t *tenso_data){

	int8_t result = 0;

	result |= eeprom_read(EEPROM_TENSO_COEF_KILOGRAMS, &tenso_data->coef_kilograms, sizeof(float));
	result |= eeprom_read(EEPROM_TENSO_COEF_NEWTONS, &tenso_data->coef_newton, sizeof(float));
	result |= eeprom_read(EEPROM_TENSO_POINT_ONE_KILOGRAMS, &tenso_data->point_one_kgs, sizeof(float));
	result |= eeprom_read(EEPROM_TENSO_POINT_TWO_KILOGRAMS, &tenso_data->point_two_kgs, sizeof(float));
	result |= eeprom_read(EEPROM_TENSO_POINT_ONE_NATIVE, &tenso_data->point_one_native, sizeof(uint16_t));
	result |= eeprom_read(EEPROM_TENSO_POINT_TWO_NATIVE, &tenso_data->point_two_native, sizeof(uint16_t));


	return result;

}

int8_t tenso_write_coefs_to_eeprom(tenso_data_t *tenso_data){

	int8_t result = 0;

	result |= eeprom_write(EEPROM_TENSO_COEF_KILOGRAMS, &tenso_data->coef_kilograms, sizeof(float));
	result |= eeprom_write(EEPROM_TENSO_COEF_NEWTONS, &tenso_data->coef_newton, sizeof(float));
	result |= eeprom_write(EEPROM_TENSO_POINT_ONE_KILOGRAMS, &tenso_data->point_one_kgs, sizeof(float));
	result |= eeprom_write(EEPROM_TENSO_POINT_TWO_KILOGRAMS, &tenso_data->point_two_kgs, sizeof(float));
	result |= eeprom_write(EEPROM_TENSO_POINT_ONE_NATIVE, &tenso_data->point_one_native, sizeof(uint16_t));
	result |= eeprom_write(EEPROM_TENSO_POINT_TWO_NATIVE, &tenso_data->point_two_native, sizeof(uint16_t));

	return result;

}

float tenso_calculate_kilograms(tenso_data_t *tenso_data){

	//linear
	return tenso_data->coef_kilograms * tenso_data->adc_native - tenso->offset;
}

float tenso_calculate_newtons(tenso_data_t *tenso_data){

	//linear
	return tenso_data->coef_newton * tenso_data->adc_native - tenso->offset;
}

int8_t cmd_tenso_calibrate(BaseSequentialStream* chp, int argc, char* argv[]) {
	if (argc != 0) {
		if (strcmp(argv[0], "point_one") == 0) {
			tenso->point_one_kgs = atof(argv[1]);
			tenso_write_coefs_to_eeprom(tenso);
			if (tenso_calculate_coefs(tenso) == -1) {
				chprintf(chp, "Failed to calculate coefs: wrong points?\n\r");
				return -1;
			}
			chprintf(chp, "Saved first point of tenso characteristic\n\r");
			return 0;
		} else if (strcmp(argv[0], "point_two") == 0) {
			tenso->point_two_kgs = atof(argv[1]);
			tenso_write_coefs_to_eeprom(tenso);
			chprintf(chp, "Saved second point of tenso characteristic\n\r");
			if (tenso_calculate_coefs(tenso) == -1) {
				chprintf(chp, "Failed to calculate coefs: wrong points?\n\r");
				return -1;
			}
			return 0;
		} else {
			chprintf(chp, "Usage: tenso calibrate point_one|point_two <kg>\n\r");
		}
	} else {
		chprintf(chp, "Usage: tenso calibrate point_one|point_two <kg>\n\r");
	}
}

int8_t tenso_calculate_coefs(tenso_data_t *tenso_data) {

	float k, b;
	if ((tenso_data->point_one_kgs < tenso_data->point_two_kgs)
			|| (tenso_data->point_one_native < tenso_data->point_two_native)) {
		return -1;
	}

	k = atan2(tenso_data->point_two_native - tenso_data->point_one_native,
			tenso_data->point_one_kgs - tenso_data->point_two_kgs);
	b = tenso_data->point_one_kgs - k * tenso_data->point_one_native;
	tenso_data->coef_kilograms = k;
	tenso_data->offset = b;
	return 0;
}
