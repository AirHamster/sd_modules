/*
 * bno055_i2c.c
 *
 *  Created on: Jul 16, 2019
 *      Author: a-h
 */

#include <stdlib.h>
#include <string.h>
#include "stdint.h"
#include "math.h"
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"
#include "bno055.h"
#include "bno055_i2c.h"
extern struct ch_semaphore usart1_semaph;

int8_t bno055_i2c_routine(struct bno055_t *bno055)
{
	bno055->BNO055_I2C_bus_write= BNO055_I2C_bus_write;
	bno055->BNO055_I2C_bus_read = BNO055_I2C_bus_read;
	bno055->delay_msec = bno055_delay_ms;
	bno055->dev_addr = BNO055_I2C_ADDR1;

	return BNO055_INIT_VALUE;
}

void bno055_delay_ms(uint16_t msec){
	chThdSleepMilliseconds(msec);
}

int8_t bno055_read(uint8_t dev_addr, uint8_t *reg_data, uint8_t r_len){
		//uint8_t txbuff[1];
		//uint8_t rxbuff[1];
		msg_t status;
		//txbuff[0] = reg_addr;
		//txbuff[1] = EEPROM_HW_VER_ADDR & 0xFF;
		chSemWait(&usart1_semaph);
				chprintf((BaseSequentialStream*)&SD1, "Entered read func\r\n");
				chSemSignal(&usart1_semaph);
		status = i2cMasterTransmitTimeout(&I2CD1, dev_addr, reg_data, 1, reg_data, r_len, 1000);
		if (status != MSG_OK){
			chSemWait(&usart1_semaph);
				chprintf((BaseSequentialStream*)&SD1, "Shit happened: status is %d\r\n", i2cGetErrors(&I2CD1));
				chSemSignal(&usart1_semaph);
				return -1;
		}
		return 0;
		//chSemWait(&usart1_semaph);
		//chprintf((BaseSequentialStream*)&SD1, "CHIP_ID from BNO055: %d\r\n", *reg_data);
		//chSemSignal(&usart1_semaph);
}

int8_t bno055_write(uint8_t dev_addr, uint8_t *reg_data, uint8_t wr_len){
		//uint8_t txbuff[wr_len+1];
		uint8_t rxbuff[1];
		msg_t status;
		//txbuff[0] = reg_addr;
		//memcpy(&txbuff[1], reg_data, wr_len);
		//txbuff[1] = EEPROM_HW_VER_ADDR & 0xFF;
		chSemWait(&usart1_semaph);
						chprintf((BaseSequentialStream*)&SD1, "Entered write func: d_addr %x, reg %x, wr_len %d\r\n", dev_addr, *reg_data, wr_len);
						chSemSignal(&usart1_semaph);
		status = i2cMasterTransmitTimeout(&I2CD1, dev_addr, reg_data, wr_len, rxbuff, 0, 1000);
		if (status != MSG_OK){
			chSemWait(&usart1_semaph);
				chprintf((BaseSequentialStream*)&SD1, "Shit happened: status is %d\r\n", i2cGetErrors(&I2CD1));
				chSemSignal(&usart1_semaph);
				return -1;
		}
		chSemWait(&usart1_semaph);
						chprintf((BaseSequentialStream*)&SD1, "Shit not happened: status is %d\r\n", i2cGetErrors(&I2CD1));
						chSemSignal(&usart1_semaph);
		return 0;
		//chSemWait(&usart1_semaph);
		//chprintf((BaseSequentialStream*)&SD1, "CHIP_ID from BNO055: %d\r\n", rxbuff[0]);
		//chSemSignal(&usart1_semaph);
}

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 BNO055_iERROR = BNO055_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN];
	u8 stringpos = BNO055_INIT_VALUE;

	array[BNO055_INIT_VALUE] = reg_addr;
	for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
	{
		array[stringpos + 1] =
			*(reg_data + stringpos);
	}
	/*
	* Please take the below APIs as your reference for
	* write the data using I2C communication
	* "BNO055_iERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
	* add your I2C write APIs here
	* BNO055_iERROR is an return value of I2C read API
	* Please select your valid return value
	* In the driver BNO055_SUCCESS defined as 0
    * and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+1 operation done
	* in the I2C write string function
	* For more information please refer data sheet SPI communication:
	*/
	BNO055_iERROR = bno055_write(dev_addr, array, cnt+1);
	return (s8)BNO055_iERROR;
}

 /*	\Brief: The API is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *  will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *   which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 BNO055_iERROR = BNO055_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN] = {BNO055_INIT_VALUE};
	u8 stringpos = BNO055_INIT_VALUE;

	array[BNO055_INIT_VALUE] = reg_addr;

	/* Please take the below API as your reference
	 * for read the data using I2C communication
	 * add your I2C read API here.
	 * "BNO055_iERROR = I2C_WRITE_READ_STRING(DEV_ADDR,
	 * ARRAY, ARRAY, 1, CNT)"
	 * BNO055_iERROR is an return value of SPI write API
	 * Please select your valid return value
     * In the driver BNO055_SUCCESS defined as 0
     * and FAILURE defined as -1
	 */
	BNO055_iERROR = bno055_read(dev_addr, array, cnt);
	for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
		*(reg_data + stringpos) = array[stringpos];
	return (s8)BNO055_iERROR;
}
