/*
 * neo_ubx.h
 *
 *  Created on: Apr 16, 2019
 *      Author: a-h
 */

#ifndef SD_MODULES_NEO_M8_NEO_UBX_H_
#define SD_MODULES_NEO_M8_NEO_UBX_H_
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"
#include "neo-m8.h"

void neo_set_vel_gain(char* val);
void neo_set_cog_pos_acc(char* val);
void neo_set_cog_gain(char* val);
void neo_set_cog_speed(char* val);
void neo_toggle_vel_lpf(uint8_t stat);
void neo_toggle_cog_lpf(uint8_t stat);
void neo_toggle_sbas(uint8_t stat);
void neo_toggle_slas(uint8_t stat);
void neo_toggle_rtk(uint8_t stat);
void neo_get_lpf_status(void);
void neo_get_full_status(void);
void neo_get_rtk_status(void);
void neo_get_slas_sbas_status(void);
#endif /* SD_MODULES_NEO_M8_NEO_UBX_H_ */
