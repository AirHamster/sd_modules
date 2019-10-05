/*
 * hmc5883_i2c.c
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

#include "hmc5883_i2c.h"
#include "sd_shell_cmds.h"
#include "eeprom.h"
#include "bno055.h"
#include "bno055_i2c.h"

extern bno055_t *bno055;
extern struct ch_semaphore usart1_semaph;
//struct ch_semaphore i2c1_semaph;

hmc5883_t *hmc5883;
static THD_WORKING_AREA(hmc5883_thread_wa, 4096*2);
static THD_FUNCTION(hmc5883_thread, arg);

const I2CConfig hmc5883_i2c_cfg = {
  0x30420F13,
		//0x20E7112A,
 // 0x40B45B69,
  0,
  0
};

void start_hmc5883_module(void){

	chThdCreateStatic(hmc5883_thread_wa, sizeof(hmc5883_thread_wa), NORMALPRIO + 3, hmc5883_thread, NULL);
}

/*
 * Thread to process data collection and filtering from MPU9250
 */

static THD_FUNCTION(hmc5883_thread, arg) {

	(void) arg;
	HMC5883L_result result;
float dest1[3];
float dest2[3];
float cos_roll;
float sin_roll;
float cos_pitch;
float sin_pitch;
float temp = 0.0;
float true_mag_x;
float true_mag_y;
int16_t true_mag_z;
	chRegSetThreadName("hmc5883 Thread");
	chThdSleepMilliseconds(500);
	i2cStart(&GYRO_IF, &hmc5883_i2c_cfg);
	hmc5883_full_init();
	chThdSleepMilliseconds(2500);
	//hmc5883_calibration(dest1, dest2);
	systime_t prev = chVTGetSystemTime(); // Current system time.
	chprintf(SHELL_IFACE, "MAGx,MAGy,MAGz,TRUE_MAGx,TRUE_MAGy,ROLL,PITCH,HDG,TRUE_HDG\r\n");

	while (true) {
	//	hmc5883_read_data(hmc5883);
	//	chprintf(SHELL_IFACE, "Magnetic: %d %d %d %x\r\n", hmc5883->x, hmc5883->y, hmc5883->z, hmc5883->status_reg);

		read_data_HMC5883L(&result);

		hmc5883->x = result.x + 41;
		hmc5883->y = result.y*0.94 + 58;
		hmc5883->z = result.z;
		/*temp = atan2((hmc5883->y),(hmc5883->x)) * 180.0/3.1415;
		temp += 50;
		if(temp < 0){
			temp += 360.0;
		}
		hmc5883->yaw = temp;*/
		cos_roll = cos(bno055->d_euler_hpr.r * 3.1415/180.0);
		sin_roll = sin(bno055->d_euler_hpr.r * 3.1415/180.0);
		cos_pitch = cos(bno055->d_euler_hpr.p * 3.1415/180.0);
		sin_pitch = sin(bno055->d_euler_hpr.p * 3.1415/180.0);

		//my_solution
		//true_mag_x = hmc5883->x * cos_pitch - hmc5883->y * sin_roll*sin_pitch - hmc5883->z * cos_roll*cos_pitch;
		//true_mag_y = hmc5883->y * cos_roll + hmc5883->z * sin_roll;

		//Egor's solution
		true_mag_x = hmc5883->x * cos_pitch - hmc5883->y * sin_roll*sin_pitch + hmc5883->z * cos_roll*sin_pitch;
		true_mag_y = hmc5883->y * cos_roll + hmc5883->z * sin_roll;


		//true_mag_x = hmc5883->x * cos_pitch + hmc5883->z * sin_pitch;
		//true_mag_y = hmc5883->x * sin_roll * sin_pitch + hmc5883->y * cos_roll - hmc5883->z * sin_roll * cos_pitch;

		temp = atan2((true_mag_y),(true_mag_x)) * 180.0/3.1415;
		temp += 30;
				if(temp < 0){
					temp += 360.0;
				}
		hmc5883->yaw = temp;
		//hmc5883->yaw = atan2((hmc5883->y),(hmc5883->x)) * 180.0/3.1415;
		/*chprintf(SHELL_IFACE, "\r\nX: %d\tY: %d\tZ: %d\tvalid: %d\r\n",
										 result.x , result.y + 55, result.z, result.valid);
		chprintf(SHELL_IFACE, "HEADING1: %f, %f, %f\r\n", hmc5883->yaw, bno055->d_euler_hpr.r, bno055->d_euler_hpr.p);
		chprintf(SHELL_IFACE, "TRUE: %f, %f\r\n", true_mag_x, true_mag_y);
		chprintf(SHELL_IFACE, "TRUE_HDG: %f\r\n", temp);*/
				//chprintf(SHELL_IFACE, "%d,%d,%d,%f,%f,%f,%f,%f,%f\r\n", result.x , result.y + 55, result.z, true_mag_x, true_mag_y, bno055->d_euler_hpr.r, bno055->d_euler_hpr.p, hmc5883->yaw, temp);

/*		temp = atan2((true_mag_y),(true_mag_x)) * 180.0/3.1415;
						if(temp < 0){
							temp += 360.0;
						}
						hmc5883->yaw = temp;

				chprintf(SHELL_IFACE, "HEADING2: %f, %f, %f\r\n", hmc5883->yaw, bno055->d_euler_hpr.r, bno055->d_euler_hpr.p);
*/
		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(100));
	}
}

uint8_t hmc5883_read_data(hmc5883_t *hmc){
	uint8_t buff[7];
		buff[0] = HMC5883_DATA_X_MSB_REG_ADDR;	//set continuous mode
		//temp = 0;
			hmc5883_read(HMC5883_I2C_ADDR, buff, 7);
		hmc->x = (buff[0] << 8) | buff[1];
		hmc->z = (buff[2] << 8) | buff[3];
		hmc->y = (buff[4] << 8) | buff[5];
		hmc->status_reg = buff[6];
		return 0;
}


int8_t hmc5883_full_init(void) {
	 /* Initialize your host interface to the BMI160 */
	i2cStart(&GYRO_IF, &hmc5883_i2c_cfg);
	//uint8_t temp = 0;
/*	uint8_t buff[2];
	buff[0] = HMC5883_MODE_REG_ADDR;	//set continuous mode
	buff[1] = 0;
	//temp = 0;
		hmc5883_write(HMC5883_I2C_ADDR, buff, 1);
*/
	uint8_t rx_data[16];
		uint8_t tx_data[16];

		msg_t status = 0;
		systime_t tmo = TIME_MS2I(4);

		/* configure magnetometer */
		tx_data[0] = 0x00; /* register address */
		tx_data[1] = 0x70 | (1 << 3); /* average of 8 samples + 75hz update */

		/* sending */
		i2cAcquireBus(&GYRO_IF);
		status = i2cMasterTransmitTimeout(&I2CD2, HMC5883_I2C_ADDR,
										  tx_data, 2, rx_data, 0, tmo);
		i2cReleaseBus(&GYRO_IF);

		if (status != 0) {
			return i2cGetErrors(&GYRO_IF);
		}

		tx_data[0] = 0x01; /* register address */
		tx_data[1] = 0x60; /* set high gain */

		i2cAcquireBus(&GYRO_IF);
		status = i2cMasterTransmitTimeout(&GYRO_IF, HMC5883_I2C_ADDR,
										  tx_data, 2, rx_data, 0, tmo);
		i2cReleaseBus(&GYRO_IF);

		if (status != 0) {
			return i2cGetErrors(&GYRO_IF);
		}

		tx_data[0] = 0x02; /* register address */
		tx_data[1] = 0x00; /* continuous measurement */

		i2cAcquireBus(&GYRO_IF);
		status = i2cMasterTransmitTimeout(&GYRO_IF, HMC5883_I2C_ADDR,
										  tx_data, 2, rx_data, 0, tmo);
		i2cReleaseBus(&GYRO_IF);

		if (status != 0) {
			return i2cGetErrors(&GYRO_IF);
		}

		return 0;

	// return 0;
}

void hmc5883_delay_ms(uint16_t msec){
	chThdSleepMilliseconds(msec);
}

int8_t hmc5883_read(uint8_t dev_addr, uint8_t *reg_data, uint8_t r_len) {
	msg_t status;
	uint8_t databuff[64];
	uint8_t register_addr = reg_data[0];
	memset(databuff, 0, 64);

	i2cAcquireBus(&GYRO_IF);
	status = i2cMasterTransmitTimeout(&GYRO_IF, dev_addr, &register_addr, 1, databuff,
			r_len, TIME_INFINITE);
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

int8_t hmc5883_write(uint8_t dev_addr, uint8_t *reg_data, uint8_t wr_len) {
	uint8_t rxbuff[1];
	msg_t status;
	uint8_t databuff[64];
	//uint8_t register_addr = reg_data[0];
	memcpy(databuff, reg_data, wr_len);
	i2cAcquireBus(&GYRO_IF);
	status = i2cMasterTransmitTimeout(&GYRO_IF, dev_addr, databuff, wr_len,
			NULL, 0, TIME_INFINITE);
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

int8_t hmc5883_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	//int32_t hmc5883_iERROR = hmc5883_INIT_VALUE;
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
	* "hmc5883_iERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
	* add your I2C write APIs here
	* hmc5883_iERROR is an return value of I2C read API
	* Please select your valid return value
	* In the driver hmc5883_SUCCESS defined as 0
    * and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+1 operation done
	* in the I2C write string function
	* For more information please refer data sheet SPI communication:
	*/
	hmc5883_write(dev_addr, array, cnt+1);
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
int8_t hmc5883_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	//int32_t hmc5883_iERROR = hmc5883_INIT_VALUE;
	uint8_t array[I2C_BUFFER_LEN] = {0};
	uint8_t stringpos = 0;

	array[0] = reg_addr;

	/* Please take the below API as your reference
	 * for read the data using I2C communication
	 * add your I2C read API here.
	 * "hmc5883_iERROR = I2C_WRITE_READ_STRING(DEV_ADDR,
	 * ARRAY, ARRAY, 1, CNT)"
	 * hmc5883_iERROR is an return value of SPI write API
	 * Please select your valid return value
     * In the driver hmc5883_SUCCESS defined as 0
     * and FAILURE defined as -1
	 */
	hmc5883_read(dev_addr, array, cnt);
	for (stringpos = 0; stringpos < cnt; stringpos++)
		*(reg_data + stringpos) = array[stringpos];
	return 0;
}
/*
void i2c_restart(I2CDriver *i2cp)
{
	i2cStop(i2cp);
	chThdSleepMilliseconds(1);
	i2cStart (i2cp, &hmc5883_i2c_cfg);
}
*/

uint8_t read_data_HMC5883L(HMC5883L_result * dest)
{
	uint8_t rx_data[16];
	uint8_t tx_data[16];

	msg_t status = 0;
	systime_t tmo = TIME_MS2I(4);

	tx_data[0] = 0x06;

	i2cAcquireBus(&GYRO_IF);
	status = i2cMasterReceiveTimeout(&GYRO_IF, HMC5883_I2C_ADDR, rx_data, 6, tmo);
	i2cReleaseBus(&GYRO_IF);

	if (status != 0) {
		return i2cGetErrors(&GYRO_IF);
	}

	tx_data[0] = 0x03;

	i2cAcquireBus(&GYRO_IF);
	status = i2cMasterTransmitTimeout(&GYRO_IF, HMC5883_I2C_ADDR, tx_data,
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

void hmc5883_calibration(float * dest1, float * dest2) {
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = { 0, 0, 0 }, mag_scale[3] = { 0, 0, 0 };
	int16_t mag_max[3] = { -2048, -2048, -2048 }, mag_min[3] = { 2048,
			2048, 2048 }, mag_temp[3] = { 0, 0, 0 };
	HMC5883L_result result;
	chSemWait(&usart1_semaph);
	chprintf((BaseSequentialStream*) &SD1, "Mag Calibration: Wave device in a figure eight until done!\r\n");
	chSemSignal(&usart1_semaph);
	chThdSleepMilliseconds(4000);

// shoot for ~fifteen seconds of mag data
	//if (MPU9250Mmode == 0x02)
		//sample_count = 128; // at 8 Hz ODR, new mag data is available every 125 ms
	//if (MPU9250Mmode == 0x06)
		sample_count = 1000; // at 100 Hz ODR, new mag data is available every 10 ms
	for (ii = 0; ii < sample_count; ii++) {
		read_data_HMC5883L(&result);
		chSemWait(&usart1_semaph);

				chprintf(SHELL_IFACE, "X: %d\tY: %d\tZ: %d\tvalid: %d\r\n",
								 result.x, result.y, result.z, result.valid);
		chSemSignal(&usart1_semaph);
		hmc5883->magCount[0] = result.x;
		hmc5883->magCount[1] = result.y;
		hmc5883->magCount[2] = result.z;
		//MPU9250readMagData(mag_temp);  // Read the mag data
		for (int jj = 0; jj < 3; jj++) {
			if (hmc5883->magCount[jj] > mag_max[jj])
				mag_max[jj] = hmc5883->magCount[jj];
			if (hmc5883->magCount[jj] < mag_min[jj])
				mag_min[jj] = hmc5883->magCount[jj];
		}
		//if (MPU9250Mmode == 0x02)
			//delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
		//if (MPU9250Mmode == 0x06)
		chThdSleepMilliseconds(10);  // at 100 Hz ODR, new mag data is available every 10 ms
	}

	chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*) &SD1, "max x = %d, max y = %d, max z = %d\r\n", mag_max[0], mag_max[1], mag_max[2]);
		chprintf((BaseSequentialStream*) &SD1, "min x = %d, min y = %d, min z = %d\r\n", mag_min[0], mag_min[1], mag_min[2]);
	chSemSignal(&usart1_semaph);
// Get hard iron correction
	mag_bias[0] = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
	mag_bias[1] = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
	mag_bias[2] = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

	//dest1[0] = (float) mag_bias[0] * mpu->mRes * mpu->magCalibration[0]; // save mag biases in G for main program
	//dest1[1] = (float) mag_bias[1] * mpu->mRes * mpu->magCalibration[1];
	//dest1[2] = (float) mag_bias[2] * mpu->mRes * mpu->magCalibration[2];

	dest1[0] = (float) mag_bias[0]; // save mag biases in G for main program
	dest1[1] = (float) mag_bias[1];
	dest1[2] = (float) mag_bias[2];

// Get soft iron correction estimate
	mag_scale[0] = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
	mag_scale[1] = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
	mag_scale[2] = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	dest2[0] = avg_rad / ((float) mag_scale[0]);
	dest2[1] = avg_rad / ((float) mag_scale[1]);
	dest2[2] = avg_rad / ((float) mag_scale[2]);

	chSemWait(&usart1_semaph);
	chprintf((BaseSequentialStream*) &SD1, "Mag Calibration done!\r\ndest1 x = %f, dest1 y = %f, dest1 z = %f\r\ndest2 x = %f, dest2 y = %f, dest2 z = %f\r\n",
				dest1[0], dest1[1], dest1[2], dest2[0], dest2[1], dest2[2]);
	chSemSignal(&usart1_semaph);
}
