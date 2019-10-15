/*
 * bno055_i2c.c
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
#include "bno055.h"
#include "bno055_i2c.h"
#include "sd_shell_cmds.h"
#include "hmc5883_i2c.h"
#include "eeprom.h"
extern struct ch_semaphore usart1_semaph;
struct ch_semaphore i2c1_semaph;
//static bno055_t bno055_struct;
//bno055_t *bno055 = &bno055_struct;
bno055_t *bno055;
extern hmc5883_t *hmc5883;

static THD_WORKING_AREA(bno055_thread_wa, 4096*2);
static THD_FUNCTION(bno055_thread, arg);
const I2CConfig bno055_i2c_cfg = {
  0x30420F13,
		//0x20E7112A,
 // 0x40B45B69,
  0,
  0
};

void start_bno055_module(void){

	chThdCreateStatic(bno055_thread_wa, sizeof(bno055_thread_wa), NORMALPRIO + 3, bno055_thread, NULL);
}

/*
 * Thread to process data collection and filtering from MPU9250
 */

static THD_FUNCTION(bno055_thread, arg) {

	(void) arg;
	uint8_t static_cal_update_cnt = 0;
	chRegSetThreadName("BNO055 Thread");
	chThdSleepMilliseconds(500);
	bno055_full_init(bno055);
	systime_t prev = chVTGetSystemTime(); // Current system time.
	while (true) {
		switch (bno055->read_type) {
		case OUTPUT_NONE:
			break;
		case OUTPUT_TEST:
			bno055_read_euler(bno055);
			bno055_read_status(bno055);
			//hmc5883_calculate();
			/*if (bno055->static_calib == 1){
				if (static_cal_update_cnt++ >=200){
					bno055_apply_calib_to_chip(bno055);
					static_cal_update_cnt = 0;
				}
			}*/
			break;
		case OUTPUT_ALL_CALIB:
			bno055_read_status(bno055);
			bno055_read_euler(bno055);
			break;
		default:
			break;
		}
		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(100));
	}
}

int8_t bno055_full_init(bno055_t *bno055) {
	int8_t comres = BNO055_INIT_VALUE;
	bno055_i2c_routine(bno055);
	i2cStart(&GYRO_IF, &bno055_i2c_cfg);
	/*--------------------------------------------------------------------------*
	 *  This API used to assign the value/reference of
	 *	the following parameters
	 *	I2C address
	 *	Bus Write
	 *	Bus read
	 *	Chip id
	 *	Page id
	 *	Accel revision id
	 *	Mag revision id
	 *	Gyro revision id
	 *	Boot loader revision id
	 *	Software revision id
	 *-------------------------------------------------------------------------*/
	chThdSleepMilliseconds(700); //Power_on_reset Recommended delay
	comres = bno055_init(bno055);

	/*	For initializing the BNO sensor it is required to the operation mode
	 of the sensor as NORMAL
	 Normal mode can set from the register
	 Page - page0
	 register - 0x3E
	 bit positions - 0 and 1*/
	/* set the power mode as NORMAL*/
	comres += bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);

	/*For reading fusion data it is required to set the
	 operation modes of the sensor
	 operation mode can set from the register
	 page - page0
	 register - 0x3D
	 bit - 0 to 3
	 for sensor data read following operation mode have to set
	 *FUSION MODE
	 *0x08 - BNO055_OPERATION_MODE_IMUPLUS
	 *0x09 - BNO055_OPERATION_MODE_COMPASS
	 *0x0A - BNO055_OPERATION_MODE_M4G
	 *0x0B - BNO055_OPERATION_MODE_NDOF_FMC_OFF
	 *0x0C - BNO055_OPERATION_MODE_NDOF
	 based on the user need configure the operation mode*/
	if (bno955_read_static_flag_from_eeprom(bno055) == 1){
		bno055_read_calib_from_eeprom(bno055);
		bno055_apply_calib_to_chip(bno055);
	}else{
		comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
	}
	comres += bno055_write_page_id(BNO055_PAGE_ZERO);
	comres += bno055_set_euler_unit(BNO055_EULER_UNIT_DEG);

	return comres;
}

int8_t bno055_save_calib_to_eeprom(bno055_t *bno055){
	eeprom_write(EEPROM_MAGN_X_OFFSET_ADDR, &bno055->magn_offset.x, sizeof(struct bno055_mag_offset_t) + sizeof(struct bno055_gyro_offset_t) + sizeof(struct bno055_accel_offset_t));
}

int8_t bno055_read_calib_from_eeprom(bno055_t *bno055){
	eeprom_read(EEPROM_MAGN_X_OFFSET_ADDR, &bno055->magn_offset.x, sizeof(struct bno055_mag_offset_t) + sizeof(struct bno055_gyro_offset_t) + sizeof(struct bno055_accel_offset_t));
}

int8_t bno955_read_static_flag_from_eeprom(bno055_t *bno055){
	uint8_t temp;
		if (eeprom_read(EEPROM_STATIC_CALIB_FLAG_ADDR, &temp, 1) != -1) {
			bno055->static_calib = temp;
			return temp;
		}
		return -1;
}

int8_t bno055_set_static_calib(bno055_t *bno055) {
	uint8_t temp = 1;
	if (eeprom_write(EEPROM_STATIC_CALIB_FLAG_ADDR, &temp, 1) != -1) {
		bno055->static_calib = 1;
		return 0;
	}
	return -1;
}

int8_t bno055_set_dynamic_calib(bno055_t *bno055) {
	uint8_t temp = 0;
	if (eeprom_write(EEPROM_STATIC_CALIB_FLAG_ADDR, &temp, 1) != -1) {
		bno055->static_calib = 0;
		return 0;
	}
	return -1;
}


int8_t bno055_apply_calib_to_chip(bno055_t *bno055){
	bno055->read_type = OUTPUT_NONE;
	bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
	bno055_write_mag_offset(&bno055->magn_offset);
	bno055_write_gyro_offset(&bno055->gyro_offset);
	bno055_write_accel_offset(&bno055->accel_offset);
	bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
	bno055->read_type = OUTPUT_TEST;
}

int8_t bno055_start_calibration(bno055_t *bno055){
	int8_t comres = BNO055_INIT_VALUE;
	bno055->read_type = OUTPUT_NONE;
	comres += bno055_set_sys_rst(0x01);
	chThdSleepMilliseconds(700);
	comres = bno055_init(bno055);
	comres += bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
	comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
	comres += bno055_write_page_id(BNO055_PAGE_ZERO);
	comres += bno055_set_euler_unit(BNO055_EULER_UNIT_DEG);
	bno055->read_type = OUTPUT_ALL_CALIB;
	return comres;
}

int8_t bno055_read_euler(bno055_t *bno055){
	int8_t comres = BNO055_INIT_VALUE;
	uint8_t euler_data[6];
//	struct bno055_euler_t reg_euler = {BNO055_INIT_VALUE,
	//	BNO055_INIT_VALUE, BNO055_INIT_VALUE};
	//euler_data[0] = BNO055_EULER_H_LSB_VALUEH_REG;
	//bno055_read(bno055->dev_addr, euler_data, 6);
	comres += bno055_convert_float_euler_hpr_deg(&bno055->d_euler_hpr);
	bno055->d_euler_hpr.h = hmc5883->yaw;
	//bno055->d_euler_hpr.h = (float)(((int16_t)(euler_data[0] | euler_data[1] << 8))/BNO055_EULER_DIV_DEG);
	//bno055->d_euler_hpr.r = (float)(((int16_t)(euler_data[2] | euler_data[3] << 8))/BNO055_EULER_DIV_DEG);
	//bno055->d_euler_hpr.p = (float)(((int16_t)(euler_data[4] | euler_data[5] << 8))/BNO055_EULER_DIV_DEG);
	//bno055->d_euler_hpr.h = (float)(reg_euler.h/BNO055_EULER_DIV_DEG);
	//bno055->d_euler_hpr.r = (float)(reg_euler.r/BNO055_EULER_DIV_DEG);
	//bno055->d_euler_hpr.p = (float)(reg_euler.p/BNO055_EULER_DIV_DEG);
		return comres;
}

int8_t bno055_read_status(bno055_t *bno055){
	int8_t comres = BNO055_INIT_VALUE;
	comres += bno055_get_mag_calib_stat(&bno055->calib_stat.magn);
	comres += bno055_get_accel_calib_stat(&bno055->calib_stat.accel);
	comres += bno055_get_gyro_calib_stat(&bno055->calib_stat.gyro);
	comres += bno055_get_sys_calib_stat(&bno055->calib_stat.system);
	return comres;
}

int8_t bno055_check_calib_coefs(bno055_t *bno055){
	int8_t comres = BNO055_INIT_VALUE;
	comres += bno055_read_sic_matrix(&bno055->sic_matrix);
	comres += bno055_read_accel_offset(&bno055->accel_offset);
	comres += bno055_read_mag_offset(&bno055->magn_offset);
	comres += bno055_read_gyro_offset(&bno055->gyro_offset);
	return comres;
}
int8_t bno055_i2c_routine(bno055_t *bno055)
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

int8_t bno055_read(uint8_t dev_addr, uint8_t *reg_data, uint8_t r_len) {
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

int8_t bno055_write(uint8_t dev_addr, uint8_t *reg_data, uint8_t wr_len) {
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

void i2c_restart(I2CDriver *i2cp)
{
	i2cStop(i2cp);
	chThdSleepMilliseconds(1);
	i2cStart (i2cp, &bno055_i2c_cfg);
}

