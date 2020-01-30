/*
 * tenso.h
 *
 *  Created on: Jan 30, 2020
 *      Author: a-h
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
	uint16_t adc_native;
}tenso_data_t;

#endif /* SD_MODULES_TENSO_TENSO_H_ */
