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

#define IR_ADC_GRP1_NUM_CHANNELS 1
#define IR_ADC_GRP1_BUF_DEPTH 32
static adcsample_t irSamples[IR_ADC_GRP1_NUM_CHANNELS * IR_ADC_GRP1_BUF_DEPTH];

const ADCConfig adccfg = {
  .difsel       = 0U
};


const ADCConversionGroup adcgrpcfg = {
  .circular     = false,
  .num_channels = IR_ADC_GRP1_NUM_CHANNELS,
  .end_cb       = NULL,
  .error_cb     = NULL,
  .cfgr         = 0U,
  .cfgr2        = 0U,
  .tr1          = ADC_TR(0, 4095),
  .smpr         = {
    ADC_SMPR1_SMP_AN0(ADC_SMPR_SMP_247P5) |
    ADC_SMPR1_SMP_AN5(ADC_SMPR_SMP_247P5),
    0U
  },
  .sqr          = {
    ADC_SQR1_SQ2_N(ADC_CHANNEL_IN5),
    0U,
    0U,
    0U
  }
};

static THD_WORKING_AREA(adc_thread_wa, 1024);
static THD_FUNCTION( adc_thread, p) {
	(void) p;
	msg_t msg;
	chRegSetThreadName("ADC Thd");
	systime_t prev = chVTGetSystemTime(); // Current system time.
	while (true) {
		/* Performing a one-shot conversion on two channels.*/
		adcStartConversion(&ADCD1, &adcgrpcfg, irSamples, IR_ADC_GRP1_BUF_DEPTH);

		  chprintf((BaseSequentialStream*) &SD1, "ADC: %d\r\n", irSamples[0]);
		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(500));
	}
}

void adc_init(void){
	adcStart(&ADCD1, &adccfg);
	chThdCreateStatic(adc_thread_wa, sizeof(adc_thread_wa), NORMALPRIO, adc_thread, NULL);
}
