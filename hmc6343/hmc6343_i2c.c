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
	//i2cStart(&GYRO_IF, &hmc6343_i2c_cfg);
	chThdSleepMilliseconds(500);
	//hmc6343_full_init();
	chThdSleepMilliseconds(2500);
	//hmc6343_calibration(dest1, dest2);
	systime_t prev = chVTGetSystemTime(); // Current system time.
	//chprintf(SHELL_IFACE, "MAGx,MAGy,MAGz,TRUE_MAGx,TRUE_MAGy,ROLL,PITCH,HDG,TRUE_HDG\r\n");

	while (true) {
		hmc6343_read_hdg_data(hmc6343);
	//	chprintf(SHELL_IFACE, "Magnetic: %d %d %d %x\r\n", hmc6343->x, hmc6343->y, hmc6343->z, hmc6343->status_reg);
		chThdSleepMilliseconds(1);
		hmc6343_read_mag_data(hmc6343);
		chThdSleepMilliseconds(1);
		hmc6343_read_acc_data(hmc6343);
	//	hmc6343_calculate();
		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(100));
	}
}

uint8_t hmc6343_read_hdg_data(hmc6343_t *hmc){
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
		//chprintf(SHELL_IFACE, "YAW: %f %f %f\r\n", hmc->yaw, hmc->pitch, hmc->roll);
		//hmc->status_reg = buff[6];
		return 0;
}

uint8_t hmc6343_read_acc_data(hmc6343_t *hmc){
	uint8_t buff[7];
		buff[0] = HMC6343_POST_ACCEL_CMD;	//set continuous mode
		buff[1] = 0;
		//temp = 0;
		hmc6343_write(HMC6343_I2C_ADDR, buff, 1);
		chThdSleepMilliseconds(1);
		hmc6343_read(HMC6343_I2C_ADDR, buff, 6);

		hmc->ax16 = (buff[0] << 8) | buff[1];
		hmc->ay16 = (buff[2] << 8) | buff[3];
		hmc->az16 = (buff[4] << 8) | buff[5];

		return 0;
}

uint8_t hmc6343_read_mag_data(hmc6343_t *hmc){
	uint8_t buff[7];
		buff[0] = HMC6343_POST_MAG_CMD;	//set continuous mode
		buff[1] = 0;
		//temp = 0;
		hmc6343_write(HMC6343_I2C_ADDR, buff, 1);
		chThdSleepMilliseconds(1);
		hmc6343_read(HMC6343_I2C_ADDR, buff, 6);

		hmc->mx16 = (buff[0] << 8) | buff[1];
		hmc->my16 = (buff[2] << 8) | buff[3];
		hmc->mz16 = (buff[4] << 8) | buff[5];

		//hmc->yaw = hmc->yaw16 / 10.0;
		//hmc->pitch = hmc->pitch16 / 10.0;
		//hmc->roll = hmc->roll16 / 10.0;
		//chprintf(SHELL_IFACE, "YAW: %f %f %f\r\n", hmc->yaw, hmc->pitch, hmc->roll);
		//hmc->status_reg = buff[6];
		return 0;
}

int8_t hmc6343_full_init(void) {
	uint8_t buff[24];
	//uint16_t i = 0;
	buff[0] = HMC6343_WRITE_EEPROM_CMD;	//set continuous mode
	buff[1] = HMC6343_EEPROM_OP_MODE2;
	buff[2] = 1 << 1;
	hmc6343_write(HMC6343_I2C_ADDR, buff, 3);
	chThdSleepMilliseconds(1);
	//hmc6343_read(HMC6343_I2C_ADDR, buff, 1);
	//chprintf(SHELL_IFACE, "EEPROM %x: %x\r\n", i, buff[0]);
/*
	for (i = 0; i <= 127; i++){
		//buff[1] = i;
		hmc6343_write(i, buff, 2);
		chThdSleepMilliseconds(3);
		//hmc6343_read(i, &buff[2], 6);
		chprintf(SHELL_IFACE, "EEPROM %x: %x\r\n", i, buff[2]);
		chThdSleepMilliseconds(100);
	}
*/
	return 0;
}

void hmc6343_start_calibration(hmc6343_t *hmc) {
	uint8_t buff;

	hmc6343_read_calib_data(hmc);

	buff = HMC6343_ENTER_USER_CALIBRATE_CMD;
	hmc6343_write(HMC6343_I2C_ADDR, &buff, 1);
	chThdSleepMilliseconds(1);
}

void hmc6343_stop_calibration(hmc6343_t *hmc) {
	uint8_t buff;
	chprintf(SHELL_IFACE, "Previous calibration parameters:\r\n\r\n");
	chprintf(SHELL_IFACE, "X_Offset: %x\tY_Offset: %x\tZ_Offset: %x\t\r\n", hmc->x_offset, hmc->y_offset, hmc->z_offset);
	buff = HMC6343_EXIT_USER_CALIBRATE_CMD;
	hmc6343_write(HMC6343_I2C_ADDR, &buff, 1);
	chThdSleepMilliseconds(100);

	hmc6343_read_calib_data(hmc);
	chprintf(SHELL_IFACE, "New calibration parameters:\r\n\r\n");
	chprintf(SHELL_IFACE, "X_Offset: %x\tY_Offset: %x\tZ_Offset: %x\t\r\n", hmc->x_offset, hmc->y_offset, hmc->z_offset);

}

void hmc6343_read_calib_data(hmc6343_t *hmc)
{
	uint8_t buff[2];
	buff[0] = HMC6343_READ_EEPROM_CMD;
	buff[1] = HMC6343_EEPROM_X_OFFSET_LSB;
	hmc6343_write(HMC6343_I2C_ADDR, buff, 2);
	chThdSleepMilliseconds(15);
	hmc6343_read(HMC6343_I2C_ADDR, buff, 1);
	hmc->x_offset = buff[0];

	buff[0] = HMC6343_READ_EEPROM_CMD;
	buff[1] = HMC6343_EEPROM_X_OFFSET_MSB;
	hmc6343_write(HMC6343_I2C_ADDR, buff, 2);
	chThdSleepMilliseconds(15);
	hmc6343_read(HMC6343_I2C_ADDR, buff, 1);
	hmc->x_offset |= buff[0] << 8;

	buff[0] = HMC6343_READ_EEPROM_CMD;
	buff[1] = HMC6343_EEPROM_Y_OFFSET_LSB;
	hmc6343_write(HMC6343_I2C_ADDR, buff, 2);
	chThdSleepMilliseconds(15);
	hmc6343_read(HMC6343_I2C_ADDR, buff, 1);
	hmc->y_offset = buff[0];

	buff[0] = HMC6343_READ_EEPROM_CMD;
	buff[1] = HMC6343_EEPROM_Y_OFFSET_MSB;
	hmc6343_write(HMC6343_I2C_ADDR, buff, 2);
	chThdSleepMilliseconds(15);
	hmc6343_read(HMC6343_I2C_ADDR, buff, 1);
	hmc->y_offset |= buff[0] << 8;

	buff[0] = HMC6343_READ_EEPROM_CMD;
	buff[1] = HMC6343_EEPROM_Z_OFFSET_LSB;
	hmc6343_write(HMC6343_I2C_ADDR, buff, 2);
	chThdSleepMilliseconds(15);
	hmc6343_read(HMC6343_I2C_ADDR, buff, 1);
	hmc->z_offset = buff[0];

	buff[0] = HMC6343_READ_EEPROM_CMD;
	buff[1] = HMC6343_EEPROM_Z_OFFSET_MSB;
	hmc6343_write(HMC6343_I2C_ADDR, buff, 2);
	chThdSleepMilliseconds(15);
	hmc6343_read(HMC6343_I2C_ADDR, buff, 1);
	hmc->z_offset |= buff[0] << 8;
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




