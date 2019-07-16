/*
 * mpu9250.c
 *
 *  Created on: Mar 14, 2019
 *      Author: a-h
 */

#include "eeprom.h"
extern struct ch_semaphore usart1_semaph;

const I2CConfig i2ccfg = {
  0x20E7112A,	// Calculated in stm32cube
  0,
  0
};

/*
const I2CConfig i2ccfg = {
  STM32_TIMINGR_PRESC(8U) |
  STM32_TIMINGR_SCLDEL(2U) | STM32_TIMINGR_SDADEL(0U) |
  STM32_TIMINGR_SCLH(22U)  | STM32_TIMINGR_SCLL(32U),
  0,
  0
};
*/

void eeprom_write_hw_version(void){
	uint8_t txbuff[3];
	msg_t status;
	txbuff[0] = EEPROM_HW_VER_ADDR >> 8;
	txbuff[1] = EEPROM_HW_VER_ADDR & 0xFF;
	txbuff[2] = 1;
	i2cAcquireBus(&I2CD1);
	status = i2cMasterTransmitTimeout(&I2CD1, EEPROM_ADDRESS, txbuff, 3, NULL, 0, 1000);
	i2cReleaseBus(&I2CD1);
	if (status != MSG_OK){
		chSemWait(&usart1_semaph);
			chprintf((BaseSequentialStream*)&SD1, "Shit happened: status is %d\r\n", i2cGetErrors(&I2CD1));
			chSemSignal(&usart1_semaph);
	}
}

void eeprom_read_hw_version(void){
	uint8_t txbuff[2];
	uint8_t rxbuff[1];
	msg_t status;
	txbuff[0] = EEPROM_HW_VER_ADDR >> 8;
	txbuff[1] = EEPROM_HW_VER_ADDR & 0xFF;
	status = i2cMasterTransmitTimeout(&I2CD1, EEPROM_ADDRESS, txbuff, 2, rxbuff, 1, 1000);
	if (status != MSG_OK){
		chSemWait(&usart1_semaph);
			chprintf((BaseSequentialStream*)&SD1, "Shit happened: status is %d\r\n", i2cGetErrors(&I2CD1));
			chSemSignal(&usart1_semaph);
	}
	chSemWait(&usart1_semaph);
	chprintf((BaseSequentialStream*)&SD1, "Hardware version readed from EEPROM: %d\r\n", rxbuff[0]);
	chSemSignal(&usart1_semaph);
}

void bno055_read_id(void){
	uint8_t txbuff[2];
		uint8_t rxbuff[1];
		msg_t status;
		txbuff[0] = BNO055_CHIP_ID_ADDR;
		//txbuff[1] = EEPROM_HW_VER_ADDR & 0xFF;
		status = i2cMasterTransmitTimeout(&I2CD1, BNO055_ADDRESS, txbuff, 1, rxbuff, 1, 1000);
		if (status != MSG_OK){
			chSemWait(&usart1_semaph);
				chprintf((BaseSequentialStream*)&SD1, "Shit happened: status is %d\r\n", i2cGetErrors(&I2CD1));
				chSemSignal(&usart1_semaph);
		}
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*)&SD1, "CHIP_ID from BNO055: %d\r\n", rxbuff[0]);
		chSemSignal(&usart1_semaph);
}
