/*
 * mpu9250.c
 *
 *  Created on: Mar 14, 2019
 *      Author: a-h
 */

#include "eeprom.h"

static const I2CConfig i2ccfg = {
  STM32_TIMINGR_PRESC(15U) |
  STM32_TIMINGR_SCLDEL(4U) | STM32_TIMINGR_SDADEL(2U) |
  STM32_TIMINGR_SCLH(15U)  | STM32_TIMINGR_SCLL(21U),
  0,
  0
};


void eeprom_write_byte(uint8_t byte)
{

}

void eeprom_read_byte(uint8_t *byte)
{
	//EEPROM_ADDRESS | 0x01	if API didn't do this
}
