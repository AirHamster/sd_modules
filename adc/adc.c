/*
 * adc.c
 *
 *  Created on: Sep 3, 2019
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
#include "adc.h"
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
#ifdef USE_EEPROM_MODULE
#include "eeprom.h"
#endif
rudder_t *rudder;

#define IR_ADC_GRP1_NUM_CHANNELS 1
#define IR_ADC_GRP1_BUF_DEPTH 4
#ifdef SD_SENSOR_BOX_RUDDER
static void adcendcallback(ADCDriver *adcp);
static adcsample_t irSamples[IR_ADC_GRP1_NUM_CHANNELS * IR_ADC_GRP1_BUF_DEPTH];

const ADCConfig adccfg = {
  .difsel       = 0U
};


const ADCConversionGroup adcgrpcfg = {
  .circular     = FALSE,
  .num_channels = IR_ADC_GRP1_NUM_CHANNELS,
  .end_cb       = adcendcallback,
  .error_cb     = NULL,
  .cfgr         = ADC_CFGR_CONT,
  .cfgr2        = 0U,
  .tr1          = ADC_TR(0, 4095),
  .smpr         = {
    0U,
    0U
  },
  .sqr          = {
    ADC_SQR1_SQ1_N(ADC_CHANNEL_IN6),
    0U,
    0U,
    0U
  }
};

static void adcendcallback(ADCDriver *adcp) {
  (void)adcp;
//  chprintf((BaseSequentialStream*) &SD1, "ADC end: %d\r\n", irSamples[0]);
}

static void adcerrorcallback(ADCDriver *adcp) {
  (void)adcp;
//  chprintf((BaseSequentialStream*) &SD1, "ADC end: %d\r\n", irSamples[0]);
}

static THD_WORKING_AREA(adc_thread_wa, 1024);
static THD_FUNCTION( adc_thread, p) {
	(void) p;
	uint8_t i = 0;
	uint32_t tmp;
	chRegSetThreadName("ADC Thd");
	systime_t prev = chVTGetSystemTime(); // Current system time.
	rudder->min_native = 100;
	rudder->max_native = 4000;
	rudder->native_full_scale = rudder->max_native - rudder->min_native;
	while (true) {
		adcConvert(&ADCD1, &adcgrpcfg, irSamples, IR_ADC_GRP1_BUF_DEPTH);
		tmp = 0;
		for (i = 0; i < IR_ADC_GRP1_BUF_DEPTH; i++) {
			tmp += irSamples[i];
		}
		tmp = tmp / IR_ADC_GRP1_BUF_DEPTH;
		adc_convert_to_rudder(tmp, rudder);
	/*	chprintf((BaseSequentialStream*) &SD1, "ADC native:  %d\r\n",
				(uint16_t) rudder->native);
		chprintf((BaseSequentialStream*) &SD1, "ADC percent: %f\r\n",
				rudder->percent);
		chprintf((BaseSequentialStream*) &SD1, "ADC degrees: %f\r\n",
				rudder->degrees);*/
		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(100));
	}
}

void start_adc_module(void){
	adcStart(&ADCD1, NULL);
	adcSTM32EnableVREF(&ADCD1);
	chThdCreateStatic(adc_thread_wa, sizeof(adc_thread_wa), NORMALPRIO, adc_thread, NULL);
}

void adc_convert_to_rudder(uint16_t tmp, rudder_t *rud) {
	//Workaround situation when sensor was instaled in mirrored position
	if (rud->min_native <= rud->max_native) {

		if (tmp < rud->min_native) {
			rud->native = rud->min_native;
		} else if (tmp > rud->max_native) {
			rud->native = rud->max_native;
		} else {
			rud->native = tmp;
		}
	} else {
		if (tmp < rud->max_native) {
			rud->native = rud->max_native;
		} else if (tmp > rud->min_native) {
			rud->native = rud->min_native;
		} else {
			rud->native = tmp;
		}
	}
	rud->percent = ((float) (rud->native - 100) / (float) rud->native_full_scale * 100.0);
	rud->degrees = ((float) rud->percent / 100.0 * 180.0 - 90.0);
}
void adc_update_rudder_struct(rudder_t *rud){
	eeprom_read(EEPROM_RUDDER_CALIB_LEFT, (uint8_t*)&rud->min_native, 2);
	eeprom_read(EEPROM_RUDDER_CALIB_RIGHT, (uint8_t*)&rud->max_native, 2);
}
void adc_print_rudder_info(rudder_t *rud){
	chprintf((BaseSequentialStream*) &SD1, "ADC native:  %d\r\n",
			(uint16_t) rud->native);
	chprintf((BaseSequentialStream*) &SD1, "ADC percent: %f\r\n",
			rud->percent);
	chprintf((BaseSequentialStream*) &SD1, "ADC degrees: %f\r\n",
			rud->degrees);
}
#endif
