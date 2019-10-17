/*
 * hmc6343_i2c.c
 *
 *  Created on: Jul 16, 2019
 *      Author: a-h
 */
#include "config.h"
#include <stdlib.h>
#include <string.h>
#include "stdint.h"
#include "math.h"
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

#include "hmc6343_i2c.h"
#include "sd_shell_cmds.h"
#include "eeprom.h"
#include "bno055.h"
#include "bno055_i2c.h"

extern bno055_t *bno055;
extern struct ch_semaphore usart1_semaph;
//struct ch_semaphore i2c1_semaph;

hmc6343_t *hmc6343;
static THD_WORKING_AREA(hmc6343_thread_wa, 4096*2);
static THD_FUNCTION(hmc6343_thread, arg);

const I2CConfig hmc6343_i2c_cfg = {
  0x30420F13,
		//0x20E7112A,
 // 0x40B45B69,
  0,
  0
};

void start_hmc6343_module(void){

	chThdCreateStatic(hmc6343_thread_wa, sizeof(hmc6343_thread_wa), NORMALPRIO + 1, hmc6343_thread, NULL);
}

/*
 * Thread to process data collection and filtering from MPU9250
 */

static THD_FUNCTION(hmc6343_thread, arg) {

	(void) arg;

	chRegSetThreadName("hmc6343 Thread");
	i2cStart(&GYRO_IF, &hmc6343_i2c_cfg);
	chThdSleepMilliseconds(500);
	//hmc6343_full_init();
	chThdSleepMilliseconds(2500);
	//hmc6343_calibration(dest1, dest2);
	systime_t prev = chVTGetSystemTime(); // Current system time.
	//chprintf(SHELL_IFACE, "MAGx,MAGy,MAGz,TRUE_MAGx,TRUE_MAGy,ROLL,PITCH,HDG,TRUE_HDG\r\n");

	while (true) {
		hmc6343_read_data(hmc6343);
	//	chprintf(SHELL_IFACE, "Magnetic: %d %d %d %x\r\n", hmc6343->x, hmc6343->y, hmc6343->z, hmc6343->status_reg);

	//	hmc6343_calculate();
		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(100));
	}
}

uint8_t hmc6343_read_data(hmc6343_t *hmc){
	uint8_t buff[7];
		buff[0] = HMC6343_POST_HEADING_DATA;	//set continuous mode
		buff[1] = 0;
		//temp = 0;
		hmc6343_write(HMC6343_I2C_ADDR, buff, 1);
		chThdSleepMilliseconds(1);
		hmc6343_read(HMC6343_I2C_ADDR, buff, 6);

		hmc->yaw16 = (buff[0] << 8) | buff[1];
		hmc->pitch16 = (buff[2] << 8) | buff[3];
		hmc->roll16 = (buff[4] << 8) | buff[5];

		hmc->yaw = hmc->yaw16 / 10.0;
		hmc->pitch = hmc->pitch16 / 10.0;
		hmc->roll = hmc->roll16 / 10.0;
//		chprintf(SHELL_IFACE, "YAW: %f %f %f\r\n", hmc->yaw, hmc->pitch, hmc->roll);
		//hmc->status_reg = buff[6];
		return 0;
}

int8_t hmc6343_full_init(void) {
	uint8_t buff[24];
	uint16_t i = 0;
	buff[0] = HMC6343_READ_EEPROM_CMD;	//set continuous mode
	buff[1] = HMC6343_EEPROM_START;
	buff[2] = 0x19;
	//hmc6343_write(HMC6343_I2C_ADDR, buff, 3);
	chThdSleepMilliseconds(1);
	//hmc6343_read(HMC6343_I2C_ADDR, buff, 1);
	chprintf(SHELL_IFACE, "EEPROM %x: %x\r\n", i, buff[0]);


	for (i = 0; i <= 127; i++){
		//buff[1] = i;
		hmc6343_write(i, buff, 2);
		chThdSleepMilliseconds(3);
		//hmc6343_read(i, &buff[2], 6);
		chprintf(SHELL_IFACE, "EEPROM %x: %x\r\n", i, buff[2]);
		chThdSleepMilliseconds(100);
	}

	return 0;
}

void hmc6343_delay_ms(uint16_t msec){
	chThdSleepMilliseconds(msec);
}

int8_t hmc6343_read(uint8_t dev_addr, uint8_t *reg_data, uint8_t r_len) {
	msg_t status;
	uint8_t databuff[64];
	uint8_t register_addr = reg_data[0];
	memset(databuff, 0, 64);

	i2cAcquireBus(&GYRO_IF);
	//status = i2cMasterTransmitTimeout(&GYRO_IF, dev_addr, NULL, 0, databuff,
			//r_len, TIME_MS2I(100));
	status = i2cMasterReceiveTimeout(&GYRO_IF, dev_addr, databuff,
				r_len, TIME_MS2I(100));
	i2cReleaseBus(&GYRO_IF);
	if (status != MSG_OK) {
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*) &SD1, "Shit happened: status %d\r\n",
				i2cGetErrors(&GYRO_IF));
		chSemSignal(&usart1_semaph);
		i2c_restart(&GYRO_IF);
		return -1;
	}
	memcpy(reg_data, databuff, r_len);
	return 0;

}

int8_t hmc6343_write(uint8_t dev_addr, uint8_t *reg_data, uint8_t wr_len) {
	uint8_t rxbuff[1];
	msg_t status;
	uint8_t databuff[64];
	//uint8_t register_addr = reg_data[0];
	memcpy(databuff, reg_data, wr_len);
	i2cAcquireBus(&GYRO_IF);
	status = i2cMasterTransmitTimeout(&GYRO_IF, dev_addr, databuff, wr_len,
			NULL, 0, TIME_MS2I(100));
	i2cReleaseBus(&GYRO_IF);
	if (status != MSG_OK) {
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*) &SD1,
				"Shit happened: status is %d\r\n", i2cGetErrors(&GYRO_IF));
		chSemSignal(&usart1_semaph);
		i2c_restart(&GYRO_IF);
		return -1;
	}
	return 0;
}

int8_t hmc6343_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	//int32_t hmc6343_iERROR = hmc6343_INIT_VALUE;
	uint8_t array[I2C_BUFFER_LEN];
	uint8_t stringpos = 0;

	array[0] = reg_addr;
	for (stringpos = 0; stringpos < cnt; stringpos++)
	{
		array[stringpos + 1] =
			*(reg_data + stringpos);
	}
	/*
	* Please take the below APIs as your reference for
	* write the data using I2C communication
	* "hmc6343_iERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
	* add your I2C write APIs here
	* hmc6343_iERROR is an return value of I2C read API
	* Please select your valid return value
	* In the driver hmc6343_SUCCESS defined as 0
    * and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+1 operation done
	* in the I2C write string function
	* For more information please refer data sheet SPI communication:
	*/
	hmc6343_write(dev_addr, array, cnt+1);
	return 0;
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
int8_t hmc6343_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	//int32_t hmc6343_iERROR = hmc6343_INIT_VALUE;
	uint8_t array[I2C_BUFFER_LEN] = {0};
	uint8_t stringpos = 0;

	array[0] = reg_addr;

	/* Please take the below API as your reference
	 * for read the data using I2C communication
	 * add your I2C read API here.
	 * "hmc6343_iERROR = I2C_WRITE_READ_STRING(DEV_ADDR,
	 * ARRAY, ARRAY, 1, CNT)"
	 * hmc6343_iERROR is an return value of SPI write API
	 * Please select your valid return value
     * In the driver hmc6343_SUCCESS defined as 0
     * and FAILURE defined as -1
	 */
	hmc6343_read(dev_addr, array, cnt);
	for (stringpos = 0; stringpos < cnt; stringpos++)
		*(reg_data + stringpos) = array[stringpos];
	return 0;
}
/*
void i2c_restart(I2CDriver *i2cp)
{
	i2cStop(i2cp);
	chThdSleepMilliseconds(1);
	i2cStart (i2cp, &hmc6343_i2c_cfg);
}
*/

uint8_t read_data_HMC6343L(HMC6343L_result * dest)
{
	uint8_t rx_data[16];
	uint8_t tx_data[16];

	msg_t status = 0;
	systime_t tmo = TIME_MS2I(4);

	tx_data[0] = 0x06;

	i2cAcquireBus(&GYRO_IF);
	status = i2cMasterReceiveTimeout(&GYRO_IF, HMC6343_I2C_ADDR, rx_data, 6, tmo);
	i2cReleaseBus(&GYRO_IF);

	if (status != 0) {
		return i2cGetErrors(&GYRO_IF);
	}

	tx_data[0] = 0x03;

	i2cAcquireBus(&GYRO_IF);
	status = i2cMasterTransmitTimeout(&GYRO_IF, HMC6343_I2C_ADDR, tx_data,
									  1, rx_data, 0, tmo);
	i2cReleaseBus(&GYRO_IF);

	//chThdSleepMilliseconds(67);

	if (status != 0) {
		return i2cGetErrors(&GYRO_IF);
	}

	dest->x = (rx_data[0] << 8) | rx_data[1];
	dest->z = (rx_data[2] << 8) | rx_data[3];
	dest->y = (rx_data[4] << 8) | rx_data[5];

	/* If overflown, make result invalid (set lower gain /higher GN#/) */
	if (
		( dest->x < -2048) || (dest->x > 2047) ||
		( dest->y < -2048) || (dest->y > 2047) ||
		( dest->z < -2048) || (dest->z > 2047)
		)
	{
		dest->valid = 0;
	}
	else
	{
		dest->valid = 1;
	}

	return 0;
}
