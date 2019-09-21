/*
 * mpu9250.c
 *
 *  Created on: Mar 14, 2019
 *      Author: a-h
 */

#include "eeprom.h"
#include "config.h"
#include <stdlib.h>
#include <string.h>

extern struct ch_semaphore usart1_semaph;

#ifdef SD_MODULE_TRAINER
const I2CConfig eeprom_i2c_cfg = {
  0xD0D43C4C,
  0,
  0
};
#endif
#ifdef SENSOR_BOX
const I2CConfig eeprom_i2c_cfg = {
  0x10E37AFF,
  0,
  0
};
#endif

int8_t eeprom_write(uint16_t byte_addr, const uint8_t *txbuf, size_t txbytes) {
	int8_t buff[txbytes + 2];
	msg_t status;
	buff[0] = byte_addr >> 8;
	buff[1] = byte_addr & 0xFF;
	memcpy(&buff[2], txbuf, txbytes);
	i2cAcquireBus(&EEPROM_IF);
	status = i2cMasterTransmitTimeout(&EEPROM_IF, EEPROM_ADDRESS, buff,
			txbytes + 2, NULL, 0, 1000);
	i2cReleaseBus(&EEPROM_IF);
	if (status != MSG_OK) {
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*) &SD1,
				"Shit happened: status is %d\r\n", i2cGetErrors(&EEPROM_IF));
		chSemSignal(&usart1_semaph);
		return -1;
	}
	return 0;
}

int8_t eeprom_read(uint16_t byte_addr, uint8_t *rxbuf, size_t rxbytes) {
	uint8_t buff[2];
	msg_t status;
	buff[0] = byte_addr >> 8;
	buff[1] = byte_addr & 0xFF;
	i2cAcquireBus(&EEPROM_IF);
	status = i2cMasterTransmitTimeout(&EEPROM_IF, EEPROM_ADDRESS, buff, 2,
			rxbuf, rxbytes, 1000);
	i2cReleaseBus(&EEPROM_IF);
	if (status != MSG_OK) {
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*) &SD1,
				"Shit happened: status is %d\r\n", i2cGetErrors(&EEPROM_IF));
		chSemSignal(&usart1_semaph);
		return -1;
	}
	return 0;
}

void eeprom_write_hw_version(void) {
	uint8_t txbuff[3];
	msg_t status;
	txbuff[0] = EEPROM_HW_VER_ADDR >> 8;
	txbuff[1] = EEPROM_HW_VER_ADDR & 0xFF;
	txbuff[2] = 1;
	i2cAcquireBus(&EEPROM_IF);
	status = i2cMasterTransmitTimeout(&EEPROM_IF, EEPROM_ADDRESS, txbuff, 3, NULL,
			0, 1000);
	i2cReleaseBus(&EEPROM_IF);
	if (status != MSG_OK) {
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*) &SD1,
				"Shit happened: status is %d\r\n", i2cGetErrors(&EEPROM_IF));
		chSemSignal(&usart1_semaph);
	}
}

void eeprom_check_i2c_bus(void) {
	uint8_t txbuff[3];
	msg_t status;
	txbuff[0] = EEPROM_HW_VER_ADDR >> 8;
	txbuff[1] = EEPROM_HW_VER_ADDR & 0xFF;
	txbuff[2] = 1;
	while (1) {
		i2cAcquireBus(&EEPROM_IF);
		status = i2cMasterTransmitTimeout(&EEPROM_IF, EEPROM_ADDRESS, txbuff, 3, NULL, 0,
				1000);
		i2cReleaseBus(&EEPROM_IF);
		if (status != MSG_OK) {
			chSemWait(&usart1_semaph);
			chprintf((BaseSequentialStream*) &SD1,
					"Shit happened: status is %d\r\n", i2cGetErrors(&EEPROM_IF));
			chSemSignal(&usart1_semaph);
		} else {
			chSemWait(&usart1_semaph);
			chprintf((BaseSequentialStream*) &SD1, "Asked %d\r\n", EEPROM_ADDRESS);
			chSemSignal(&usart1_semaph);
		}
	}
}

void eeprom_read_hw_version(void) {
	uint8_t txbuff[2];
	uint8_t rxbuff[1];
	msg_t status;
	txbuff[0] = EEPROM_HW_VER_ADDR >> 8;
	txbuff[1] = EEPROM_HW_VER_ADDR & 0xFF;
	i2cAcquireBus(&EEPROM_IF);
	status = i2cMasterTransmitTimeout(&EEPROM_IF, EEPROM_ADDRESS, txbuff, 2, rxbuff,
			1, 1000);
	i2cReleaseBus(&EEPROM_IF);
	if (status != MSG_OK) {
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*) &SD1,
				"Shit happened: status is %d\r\n", i2cGetErrors(&EEPROM_IF));
		chSemSignal(&usart1_semaph);
	}
	chSemWait(&usart1_semaph);
			chprintf((BaseSequentialStream*) &SD1,
					"Readed from EEPROM %d\r\n", rxbuff[0]);
			chSemSignal(&usart1_semaph);
}

void start_eeprom_module(void){
	i2cStart(&EEPROM_IF, &eeprom_i2c_cfg);
}
