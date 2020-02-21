/*
 * adc.h
 *
 *  Created on: Sep 3, 2019
 *      Author: a-h
 */
#ifndef SD_MODULES_ADC_ADC_H_
#define SD_MODULES_ADC_ADC_H_

#include "stdint.h"
#include <string.h>
#include <stdlib.h>
#include "ch.h"
#include "hal.h"

#define ADC_CH1	1
#define ADC_CH2	2
#define ADC_CH3	3
#define ADC_CH4	4

typedef struct{
	float degrees;
	float percent;
	float native;
	float min_native;
	float center_native;
	float max_native;
	float native_full_scale;
	float min_degrees;
	float center_degrees;
	float max_degrees;
}rudder_t;

typedef struct {
	float x1;
	float x2;
	float x3;
	float y1;
	float y2;
	float y3;
} dots_t;

typedef struct {
	float a;
	float b;
	float c;
} coefs_t;

uint16_t adc_get_avg_measure(uint8_t channel, uint8_t sample_num);
void adc_print_rudder_info(rudder_t *rud);
void adc_convert_to_rudder(uint16_t tmp, rudder_t *rud);
void start_adc_module(void);
void adc_update_rudder_struct(rudder_t *rud);
float get_polynom_degrees(float x, coefs_t *coefs);
void calculate_polynom_coefs(dots_t *dots, coefs_t *coefs);
void init_coefs(dots_t *dots, coefs_t *coefs);
#endif /* SD_MODULES_ADC_ADC_H_ */
