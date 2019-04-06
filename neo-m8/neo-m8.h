/*
 * neo-m8.h
 *
 *  Created on: Mar 19, 2019
 *      Author: a-h
 */

#ifndef SD_MODULES_NEO_M8_NEO_M8_H_
#define SD_MODULES_NEO_M8_NEO_M8_H_

#include <stdlib.h>
#include <string.h>
#include "stdint.h"
#include "math.h"
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"
#include "minmea.h"

#define RTCM3_EN			0

#define UBX_PAYLOAD			6
#define UBX_HEADER			0xB562
#define UBX_CFG_CLASS		0x06
#define UBX_CFG_PRT_ID		0x00
#define UBX_CFG_PRT_LEN	20

#define UBX_CFG_MSG_LEN_SINGLE	3
#define UBX_CFG_MSG_ID			0x01
#define UBX_NAV_CLASS			0x01
#define UBX_NAV_PVT_ID			0x07
#define UBX_NAV_PVT_LEN			92
void neo_write_byte(SPIDriver *SPID, uint8_t reg_addr, uint8_t value);
uint8_t neo_read_byte(SPIDriver *SPID, uint8_t reg_addr);
void neo_read_bytes(SPIDriver *SPID, uint16_t num, uint8_t *rxbuf);
void neo_switch_to_ubx(void);
void neo_set_pvt_1hz(void);
void neo_poll_prt(void);

void neo_apply_header(uint8_t *buffer, uint16_t header);
void neo_apply_class(uint8_t *buffer, uint8_t class);
void neo_apply_id(uint8_t *buffer, uint8_t id);
void neo_apply_length(uint8_t *buffer, uint8_t len);
uint16_t neo_calc_crc(uint8_t *buffer, uint8_t len);
#endif /* SD_MODULES_NEO_M8_NEO_M8_H_ */
