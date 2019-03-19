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

void neo_write_byte(SPIDriver *SPID, uint8_t reg_addr, uint8_t value);
uint8_t neo_read_byte(SPIDriver *SPID, uint8_t reg_addr);
void neo_read_bytes(SPIDriver *SPID, uint8_t num, uint8_t *rxbuf);

#endif /* SD_MODULES_NEO_M8_NEO_M8_H_ */
