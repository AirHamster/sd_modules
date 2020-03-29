/*
 * tenso.h
 *
 *  Created on: Jan 30, 2020
 *      Author: a-h
 */
/**
 * @file    tenso.h
 * @brief   Tenso driver macros and structures.
 *
 * @addtogroup TENSO
 * @{
 */

#ifndef SD_MODULES_TENSO_TENSO_H_
#define SD_MODULES_TENSO_TENSO_H_

#include "stdint.h"
#include <string.h>
#include <stdlib.h>
#include "ch.h"
#include "hal.h"

typedef struct{
	float newtons;
	float kilograms;
	float coef_newton;
	float coef_kilograms;
	float point_kilograms;
	float point_one_kgs;
	float point_two_kgs;
	float offset;
	uint16_t adc_native;

	uint16_t point_one_native;
	uint16_t point_two_native;
}tenso_data_t;

int8_t tenso_read_coefs_from_eeprom(tenso_data_t *tenso_data);
int8_t tenso_calculate_coefs(tenso_data_t *tenso_data);
int8_t cmd_tenso_calibrate(BaseSequentialStream* chp, int argc, char* argv[]);
float tenso_calculate_kilograms(tenso_data_t *tenso_data);
float tenso_calculate_newtons(tenso_data_t *tenso_data);
void tenso_print_info(tenso_data_t *tenso_data);

#endif /* SD_MODULES_TENSO_TENSO_H_ */
