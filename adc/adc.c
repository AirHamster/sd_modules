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
#include <stdint.h>
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
dots_t *dots;
coefs_t *coefs;

#define IR_ADC_GRP1_NUM_CHANNELS 1
#define IR_ADC_GRP1_BUF_DEPTH 4
#if defined(USE_ADC_MODULE) && defined(USE_RUDDER_MODULE) && defined(SD_SENSOR_BOX)
static void adcendcallback(ADCDriver *adcp);
static adcsample_t irSamples[IR_ADC_GRP1_NUM_CHANNELS * IR_ADC_GRP1_BUF_DEPTH];

const ADCConfig adccfg = {
  .difsel       = 0U
};


ADCConversionGroup adcgrpcfg = {
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
    ADC_SQR1_SQ1_N(ADC_CHANNEL_IN5),
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
	init_coefs(dots, coefs);
	while (true) {
	//	adcConvert(&ADCD1, &adcgrpcfg, irSamples, IR_ADC_GRP1_BUF_DEPTH);
		tmp = 0;
		for (i = 0; i < IR_ADC_GRP1_BUF_DEPTH; i++) {
			tmp += irSamples[i];
		}
		tmp = tmp / IR_ADC_GRP1_BUF_DEPTH;
		adc_convert_to_rudder(tmp, rudder);
/*		chprintf((BaseSequentialStream*) &SD1, "ADC native:  %d\r\n",
				(uint16_t) rudder->native);*/
	/*
		chprintf((BaseSequentialStream*) &SD1, "ADC percent: %f\r\n",
				rudder->percent);
		chprintf((BaseSequentialStream*) &SD1, "ADC degrees: %f\r\n",
				rudder->degrees);*/
		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(100));
	}
}



void start_adc_module(void){
	//adcStart(&ADCD1, NULL);
	//adcSTM32EnableVREF(&ADCD1);
	chThdCreateStatic(adc_thread_wa, sizeof(adc_thread_wa), NORMALPRIO, adc_thread, NULL);
}

void adc_convert_to_rudder(uint16_t tmp, rudder_t *rud) {
	//Workaround situation when sensor was instaled in mirrored position
	/*if (rud->min_native <= rud->max_native) {

	 if (tmp < rud->min_native) {
	 rud->native = rud->min_native;
	 } else if (tmp > rud->max_native) {
	 rud->native = rud->max_native;
	 } else {
	 rud->native = tmp;
	 }
	 } else*/
	/*
	 {
	 if (tmp < rud->max_native) {
	 rud->native = rud->max_native;
	 } else if (tmp > rud->min_native) {
	 rud->native = rud->min_native;
	 } else {
	 rud->native = tmp;
	 }
	 }
	 */
	rud->native = tmp;
	//rud->percent = ((float) (rud->native - 100) / (float) rud->native_full_scale * 100.0);
	rud->degrees = get_polynom_degrees(rud->native, coefs);
	if (rud->degrees < rud->min_degrees) {
		rud->degrees = rud->min_degrees;
	} else if (rud->degrees > rud->max_degrees) {
		rud->degrees = rud->max_degrees;
	}

	//rud->degrees = ((float) rud->percent / 100.0 * 180.0 - 90.0);
}

uint16_t adc_get_avg_measure(uint8_t channel, uint8_t sample_num) {

	adcsample_t Samples[IR_ADC_GRP1_NUM_CHANNELS * sample_num];
	uint8_t i = 0;
	uint32_t tmp;

	switch (channel) {
	case 1:
		adcgrpcfg.sqr[0] = ADC_SQR1_SQ1_N(ADC_CHANNEL_IN5);
		break;
	case 2:
		adcgrpcfg.sqr[0] = ADC_SQR1_SQ1_N(ADC_CHANNEL_IN6);
		break;
	case 3:
		adcgrpcfg.sqr[0] = ADC_SQR1_SQ1_N(ADC_CHANNEL_IN3);
		break;
	case 4:
		adcgrpcfg.sqr[0] = ADC_SQR1_SQ1_N(ADC_CHANNEL_IN4);
	}
	adcStart(&ADCD1, NULL);
	adcSTM32EnableVREF(&ADCD1);
	adcConvert(&ADCD1, &adcgrpcfg, Samples, sample_num);

	tmp = 0;
	for (i = 0; i < sample_num; i++) {
		tmp += Samples[i];
	}
	tmp = tmp / sample_num;

	return (uint16_t)tmp;
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

void init_coefs(dots_t *dots, coefs_t *coefs){
	uint8_t temp;
	///wtf???
	/*eeprom_read(EEPROM_RUDDER_CALIB_FLAG_ADDR, &temp, 1);
	if (temp == 0){
		dots->x1 = 100.0;
		dots->x2 = 1950.0;
		dots->x3 = 4000.0;

		dots->y1 = -90.0;
		dots->y2 = 0.0;
		dots->y2 = 90.0;
	}else{ */
		EEPROM_READ(RUDDER_MEMORY.RUDDER_CALIB_NATIVE_LEFT, (uint8_t*)&dots->x1);
		//eeprom_read(EEPROM_RUDDER_CALIB_NATIVE_LEFT, (uint8_t*)&dots->x1, 4);
		rudder->min_native = dots->x1;
		//chThdSleepMilliseconds(5);
		EEPROM_READ(RUDDER_MEMORY.RUDDER_CALIB_NATIVE_CENTER, (uint8_t*)&dots->x2);
		//eeprom_read(EEPROM_RUDDER_CALIB_NATIVE_CENTER, (uint8_t*)&dots->x2, 4);
		//chThdSleepMilliseconds(5);
		EEPROM_READ(RUDDER_MEMORY.RUDDER_CALIB_NATIVE_RIGHT, (uint8_t*)&dots->x3);
		//eeprom_read(EEPROM_RUDDER_CALIB_NATIVE_RIGHT, (uint8_t*)&dots->x3, 4);
		rudder->max_native = dots->x3;
		//chThdSleepMilliseconds(5);

		EEPROM_READ(RUDDER_MEMORY.RUDDER_CALIB_DEGREES_LEFT, (uint8_t*)&dots->y1);
		//eeprom_read(EEPROM_RUDDER_CALIB_DEGREES_LEFT, (uint8_t*)&dots->y1, 4);
		rudder->min_degrees = dots->y1;
		//chThdSleepMilliseconds(5);
		EEPROM_READ(RUDDER_MEMORY.RUDDER_CALIB_DEGREES_CENTER, (uint8_t*)&dots->y2);
		//eeprom_read(EEPROM_RUDDER_CALIB_DEGREES_CENTER, (uint8_t*)&dots->y2, 4);
		//chThdSleepMilliseconds(5);
		EEPROM_READ(RUDDER_MEMORY.RUDDER_CALIB_DEGREES_RIGHT, (uint8_t*)&dots->y3);
		//eeprom_read(EEPROM_RUDDER_CALIB_DEGREES_RIGHT, (uint8_t*)&dots->y3, 4);
		rudder->max_degrees = dots->y3;
	//}
	rudder->native_full_scale = rudder->max_native - rudder->min_native;
	calculate_polynom_coefs(dots, coefs);
}

void adc_update_rudder_struct(rudder_t *rud){
//	eeprom_read(EEPROM_RUDDER_CALIB_LEFT, (uint8_t*)&rud->min_native, 2);
//	eeprom_read(EEPROM_RUDDER_CALIB_RIGHT, (uint8_t*)&rud->max_native, 2);
	calculate_polynom_coefs(dots, coefs);
}

float get_polynom_degrees(float x, coefs_t *coefs){
	float y = 0.0;
	y = coefs->a*x*x+coefs->b*x+coefs->c;
	return y;
}

// http://accel.ru/inform/edu/algorithms/begin_numeth-pr2013/index.php?fname=_1_2_simple_itpl_poly.php
void calculate_polynom_coefs(dots_t *dots, coefs_t *coefs){
    float x0 = dots->x1;
    float x1 = dots->x2;
    float x2 = dots->x3;
    float y0 = dots->y1;
    float y1 = dots->y2;
    float y2 = dots->y3;

    float a0 = 0.0;
    float a1 = 0.0;
    float a2 = 0.0;

	if ((x2 == x0) || (x2 == x1) || (x1 == x0)) {
		coefs->a = 0;
		coefs->b = 0;
		coefs->c = 0;
	} else {
		a0 = (1 / (x1 * (x2 - x0))
				* (x0 * ((y2 * x1 * x1 - y1 * x2 * x2) / (x2 - x1))
						- x2 * ((y1 * x0 * x0 - y0 * x1 * x1) / (x1 - x0))));
		a1 = -(1 / ((x2 - x0))
				* ((((y2 - y1) * (x1 + x0)) / (x2 - x1))
						- (((y1 - y0) * (x2 + x1)) / (x1 - x0))));
		a2 = (1 / ((x2 - x0))
				* ((((y2 - y1)) / (x2 - x1)) - (((y1 - y0)) / (x1 - x0))));
		coefs->a = a2;
		coefs->b = a1;
		coefs->c = a0;
	}
}
