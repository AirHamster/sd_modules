/*
 * nina-b3.h
 *
 *  Created on: Apr 26, 2019
 *      Author: a-h
 */

#ifndef SD_MODULES_NINA_B3_NINA_B3_H_
#define SD_MODULES_NINA_B3_NINA_B3_H_
#include "stdint.h"
#include "ch.h"
#include "hal.h"
#define NINA_SUCCESS 1
typedef struct{

}ble_t;
typedef struct{
	uint16_t value_handle;
	uint16_t cccd_handle;
	uint16_t uuid;
	uint32_t value;
	uint8_t properties;
	uint8_t security_read;
	uint8_t security_write;
	uint8_t read_auth;
	uint8_t max_lenth;
}ble_charac_t;

typedef struct {
	uint16_t value_handle;
	uint16_t cccd_handle;
	uint8_t parsed;
} ble_temp_charac_t;

#define NUM_OF_CHARACTS 10
void start_ble_module(void);
void nina_send_at(void);
void nina_init_module(void);
void nina_fill_memory(void);
void nina_send_one(uint8_t data);
void nina_send_two(void);
uint8_t nina_parse_command(int8_t *strp);
uint8_t nina_init_services(void);
uint8_t nina_wait_response(int8_t *at_command);

#endif /* SD_MODULES_NINA_B3_NINA_B3_H_ */
