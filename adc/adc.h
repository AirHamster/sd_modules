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
typedef struct{
	float degrees;
	float percent;
	uint16_t native;
	uint16_t min_native;
	uint16_t max_native;
	uint16_t native_full_scale;
}rudder_t;
void adc_print_rudder_info(rudder_t *rud);
void adc_convert_to_rudder(uint16_t tmp, rudder_t *rud);
void start_adc_module(void);
void adc_update_rudder_struct(rudder_t *rud);
#endif /* SD_MODULES_ADC_ADC_H_ */
