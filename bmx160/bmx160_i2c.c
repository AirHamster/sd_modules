/*
 * bmx160_i2c.c
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

#include "bmi160.h"
#include "bmm150.h"
#include "bmx160_i2c.h"
#include "sd_shell_cmds.h"
#include "eeprom.h"
extern struct ch_semaphore usart1_semaph;
struct ch_semaphore i2c1_semaph;
int8_t rslt = BMI160_OK;
struct bmi160_sensor_data accel;
struct bmi160_sensor_data gyro;
//static bmx160_t bmx160_struct;
//bmx160_t *bmx160 = &bmx160_struct;
//bmx160_t *bmx160;


/* Macros for frames to be read */

#define ACC_FRAMES	10 /* 10 Frames are available every 100ms @ 100Hz */
#define GYR_FRAMES	10
#define MAG_FRAMES	10
/* 10 frames containing a 1 byte header, 6 bytes of accelerometer,
 * 6 bytes of gyroscope and 8 bytes of magnetometer data. This results in
 * 21 bytes per frame. Additional 40 bytes in case sensor time readout is enabled */
#define FIFO_SIZE	250

/* Variable declarations */

struct bmi160_dev bmi;
struct bmm150_dev bmm;

uint8_t fifo_buff[FIFO_SIZE];
struct bmi160_fifo_frame fifo_frame;
struct bmi160_aux_data aux_data[MAG_FRAMES];
struct bmm150_mag_data mag_data[MAG_FRAMES];
struct bmi160_sensor_data gyro_data[GYR_FRAMES], accel_data[ACC_FRAMES];

int8_t rslt;

/* Auxiliary function declarations */
int8_t bmm150_aux_read(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len);
int8_t bmm150_aux_write(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len);


static THD_WORKING_AREA(bmx160_thread_wa, 4096*2);
static THD_FUNCTION(bmx160_thread, arg);
const I2CConfig bmx160_i2c_cfg = {
  0x30420F13,
		//0x20E7112A,
 // 0x40B45B69,
  0,
  0
};

void start_bmx160_module(void){

	chThdCreateStatic(bmx160_thread_wa, sizeof(bmx160_thread_wa), NORMALPRIO + 3, bmx160_thread, NULL);
}

/*
 * Thread to process data collection and filtering from MPU9250
 */

static THD_FUNCTION(bmx160_thread, arg) {

	(void) arg;
	uint8_t static_cal_update_cnt = 0;
	chRegSetThreadName("bmx160 Thread");
	chThdSleepMilliseconds(500);
	i2cStart(&GYRO_IF, &bmx160_i2c_cfg);
	bmx160_full_init();
	systime_t prev = chVTGetSystemTime(); // Current system time.
	while (true) {
		/* To read both Accel and Gyro data */
		bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, &bmi);
		rslt = bmm150_read_mag_data(&bmm);

			/* Print the Mag data */
			//chprintf("\n Magnetometer data \n");

		chprintf(SHELL_IFACE, "\r\nReaded from BMX160 accel %d %d %d\r\n", accel.x, accel.y, accel.z);
		chprintf(SHELL_IFACE, "Readed from BMX160 gyro  %d %d %d\r\n", gyro.x, gyro.y, gyro.z);
		chprintf(SHELL_IFACE, "Readed from BMX160 magn  %02f %02f %02f\r\n", bmm.data.x, bmm.data.y, bmm.data.z);
	/*	switch (bmx160->read_type) {
		case OUTPUT_NONE:
			break;
		case OUTPUT_TEST:
			bmx160_read_euler(bmx160);
			bmx160_read_status(bmx160);
			if (bmx160->static_calib == 1){
				if (static_cal_update_cnt++ >=200){
					bmx160_apply_calib_to_chip(bmx160);
					static_cal_update_cnt = 0;
				}
			}
			break;
		case OUTPUT_ALL_CALIB:
			bmx160_read_status(bmx160);
			bmx160_read_euler(bmx160);
			break;
		default:
			break;
		}*/
		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(100));
	}
}

int8_t bmx160_full_init(void) {
	 /* Initialize your host interface to the BMI160 */

	    /* This example uses I2C as the host interface */
	    bmi.id = BMI160_I2C_ADDR;
	    bmi.read = bmx160_I2C_bus_read;
	    bmi.write = bmx160_I2C_bus_write;
	    bmi.delay_ms = bmx160_delay_ms;
	    bmi.interface = BMI160_I2C_INTF;

	    /* The BMM150 API tunnels through the auxiliary interface of the BMI160 */
	    /* Check the pins of the BMM150 for the right I2C address */
	    bmm.dev_id = BMM150_DEFAULT_I2C_ADDRESS;
	    bmm.intf = BMM150_I2C_INTF;
	    bmm.read = bmm150_aux_read;
	    bmm.write = bmm150_aux_write;
	    bmm.delay_ms = bmx160_delay_ms;

	    rslt = bmi160_init(&bmi);
	    /* Check rslt for any error codes */

	    /* Configure the BMI160's auxiliary interface for the BMM150 */
	    bmi.aux_cfg.aux_sensor_enable = BMI160_ENABLE;
	    bmi.aux_cfg.aux_i2c_addr = bmm.dev_id;
	    bmi.aux_cfg.manual_enable = BMI160_ENABLE; /* Manual mode */
	    bmi.aux_cfg.aux_rd_burst_len = BMI160_AUX_READ_LEN_3; /* 8 bytes */

	    rslt = bmi160_aux_init(&bmi);
	    /* Check rslt for any error codes */

	    rslt = bmm150_init(&bmm);
	    /* Check rslt for any error codes */

	    /* Configure the accelerometer */
	    bmi.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
	    bmi.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
	    bmi.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
	    bmi.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

	    /* Configure the gyroscope */
	    bmi.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
	    bmi.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
	    bmi.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
	    bmi.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

	    rslt = bmi160_set_sens_conf(&bmi);
	    /* Check rslt for any error codes */

	    /* Configure the magnetometer. The regular preset supports up to 100Hz in Forced mode */
	    bmm.settings.preset_mode = BMM150_PRESETMODE_REGULAR;
	    rslt = bmm150_set_presetmode(&bmm);
	    /* Check rslt for any error codes */

	    /* It is important that the last write to the BMM150 sets the forced mode.
	     * This is because the BMI160 writes the last value to the auxiliary sensor
	     * after every read */
	    bmm.settings.pwr_mode = BMM150_FORCED_MODE;
	    rslt = bmm150_set_op_mode(&bmm);
	    /* Check rslt for any error codes */

	    uint8_t bmm150_data_start = BMM150_DATA_X_LSB;
	    bmi.aux_cfg.aux_odr = BMI160_AUX_ODR_100HZ;
	    rslt = bmi160_set_aux_auto_mode(&bmm150_data_start, &bmi);
	    /* Check rslt for any error codes */

	    /* Link the FIFO memory location */
//	    fifo_frame.data = fifo_buff;
//	    fifo_frame.length = FIFO_SIZE;
//	    bmi.fifo = &fifo_frame;

	    /* Clear all existing FIFO configurations */
//	    rslt = bmi160_set_fifo_config(BMI160_FIFO_CONFIG_1_MASK , BMI160_DISABLE, &bmi);
	    /* Check rslt for any error codes */

//	    uint8_t fifo_config = BMI160_FIFO_HEADER | BMI160_FIFO_AUX |  BMI160_FIFO_ACCEL | BMI160_FIFO_GYRO;
//	    rslt = bmi160_set_fifo_config(fifo_config, BMI160_ENABLE, &bmi);
	    /* Check rslt for any error codes */
}
/*
int8_t bmx160_save_calib_to_eeprom(bmx160_t *bmx160){
	eeprom_write(EEPROM_MAGN_X_OFFSET_ADDR, &bmx160->magn_offset.x, sizeof(struct bmx160_mag_offset_t) + sizeof(struct bmx160_gyro_offset_t) + sizeof(struct bmx160_accel_offset_t));
}

int8_t bmx160_read_calib_from_eeprom(bmx160_t *bmx160){
	eeprom_read(EEPROM_MAGN_X_OFFSET_ADDR, &bmx160->magn_offset.x, sizeof(struct bmx160_mag_offset_t) + sizeof(struct bmx160_gyro_offset_t) + sizeof(struct bmx160_accel_offset_t));
}

int8_t bno955_read_static_flag_from_eeprom(bmx160_t *bmx160){
	uint8_t temp;
		if (eeprom_read(EEPROM_STATIC_CALIB_FLAG_ADDR, &temp, 1) != -1) {
			bmx160->static_calib = temp;
			return temp;
		}
		return -1;
}

int8_t bmx160_set_static_calib(bmx160_t *bmx160) {
	uint8_t temp = 1;
	if (eeprom_write(EEPROM_STATIC_CALIB_FLAG_ADDR, &temp, 1) != -1) {
		bmx160->static_calib = 1;
		return 0;
	}
	return -1;
}

int8_t bmx160_set_dynamic_calib(bmx160_t *bmx160) {
	uint8_t temp = 0;
	if (eeprom_write(EEPROM_STATIC_CALIB_FLAG_ADDR, &temp, 1) != -1) {
		bmx160->static_calib = 0;
		return 0;
	}
	return -1;
}


int8_t bmx160_apply_calib_to_chip(bmx160_t *bmx160){
	bmx160->read_type = OUTPUT_NONE;
	bmx160_set_operation_mode(bmx160_OPERATION_MODE_CONFIG);
	bmx160_write_mag_offset(&bmx160->magn_offset);
	bmx160_write_gyro_offset(&bmx160->gyro_offset);
	bmx160_write_accel_offset(&bmx160->accel_offset);
	bmx160_set_operation_mode(bmx160_OPERATION_MODE_NDOF);
	bmx160->read_type = OUTPUT_TEST;
}

int8_t bmx160_start_calibration(bmx160_t *bmx160){
	int8_t comres = bmx160_INIT_VALUE;
	bmx160->read_type = OUTPUT_NONE;
	comres += bmx160_set_sys_rst(0x01);
	chThdSleepMilliseconds(700);
	comres = bmx160_init(bmx160);
	comres += bmx160_set_power_mode(bmx160_POWER_MODE_NORMAL);
	comres += bmx160_set_operation_mode(bmx160_OPERATION_MODE_NDOF);
	comres += bmx160_write_page_id(bmx160_PAGE_ZERO);
	comres += bmx160_set_euler_unit(bmx160_EULER_UNIT_DEG);
	bmx160->read_type = OUTPUT_ALL_CALIB;
	return comres;
}

int8_t bmx160_read_euler(bmx160_t *bmx160){
	int8_t comres = bmx160_INIT_VALUE;
	uint8_t euler_data[6];
//	struct bmx160_euler_t reg_euler = {bmx160_INIT_VALUE,
	//	bmx160_INIT_VALUE, bmx160_INIT_VALUE};
	//euler_data[0] = bmx160_EULER_H_LSB_VALUEH_REG;
	//bmx160_read(bmx160->dev_addr, euler_data, 6);
	comres += bmx160_convert_float_euler_hpr_deg(&bmx160->d_euler_hpr);
	//bmx160->d_euler_hpr.h = (float)(((int16_t)(euler_data[0] | euler_data[1] << 8))/bmx160_EULER_DIV_DEG);
	//bmx160->d_euler_hpr.r = (float)(((int16_t)(euler_data[2] | euler_data[3] << 8))/bmx160_EULER_DIV_DEG);
	//bmx160->d_euler_hpr.p = (float)(((int16_t)(euler_data[4] | euler_data[5] << 8))/bmx160_EULER_DIV_DEG);
	//bmx160->d_euler_hpr.h = (float)(reg_euler.h/bmx160_EULER_DIV_DEG);
	//bmx160->d_euler_hpr.r = (float)(reg_euler.r/bmx160_EULER_DIV_DEG);
	//bmx160->d_euler_hpr.p = (float)(reg_euler.p/bmx160_EULER_DIV_DEG);
		return comres;
}

int8_t bmx160_read_status(bmx160_t *bmx160){
	int8_t comres = bmx160_INIT_VALUE;
	comres += bmx160_get_mag_calib_stat(&bmx160->calib_stat.magn);
	comres += bmx160_get_accel_calib_stat(&bmx160->calib_stat.accel);
	comres += bmx160_get_gyro_calib_stat(&bmx160->calib_stat.gyro);
	comres += bmx160_get_sys_calib_stat(&bmx160->calib_stat.system);
	return comres;
}

int8_t bmx160_check_calib_coefs(bmx160_t *bmx160){
	int8_t comres = bmx160_INIT_VALUE;
	comres += bmx160_read_sic_matrix(&bmx160->sic_matrix);
	comres += bmx160_read_accel_offset(&bmx160->accel_offset);
	comres += bmx160_read_mag_offset(&bmx160->magn_offset);
	comres += bmx160_read_gyro_offset(&bmx160->gyro_offset);
	return comres;
}
int8_t bmx160_i2c_routine(bmx160_t *bmx160)
{
	bmx160->bmx160_I2C_bus_write= bmx160_I2C_bus_write;
	bmx160->bmx160_I2C_bus_read = bmx160_I2C_bus_read;
	bmx160->delay_msec = bmx160_delay_ms;
	bmx160->dev_addr = bmx160_I2C_ADDR1;

	return bmx160_INIT_VALUE;
}
*/
void bmx160_delay_ms(uint16_t msec){
	chThdSleepMilliseconds(msec);
}

int8_t bmx160_read(uint8_t dev_addr, uint8_t *reg_data, uint8_t r_len) {
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

int8_t bmx160_write(uint8_t dev_addr, uint8_t *reg_data, uint8_t wr_len) {
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

int8_t bmx160_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	//int32_t bmx160_iERROR = bmx160_INIT_VALUE;
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
	* "bmx160_iERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
	* add your I2C write APIs here
	* bmx160_iERROR is an return value of I2C read API
	* Please select your valid return value
	* In the driver bmx160_SUCCESS defined as 0
    * and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+1 operation done
	* in the I2C write string function
	* For more information please refer data sheet SPI communication:
	*/
	bmx160_write(dev_addr, array, cnt+1);
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
int8_t bmx160_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	//int32_t bmx160_iERROR = bmx160_INIT_VALUE;
	uint8_t array[I2C_BUFFER_LEN] = {0};
	uint8_t stringpos = 0;

	array[0] = reg_addr;

	/* Please take the below API as your reference
	 * for read the data using I2C communication
	 * add your I2C read API here.
	 * "bmx160_iERROR = I2C_WRITE_READ_STRING(DEV_ADDR,
	 * ARRAY, ARRAY, 1, CNT)"
	 * bmx160_iERROR is an return value of SPI write API
	 * Please select your valid return value
     * In the driver bmx160_SUCCESS defined as 0
     * and FAILURE defined as -1
	 */
	bmx160_read(dev_addr, array, cnt);
	for (stringpos = 0; stringpos < cnt; stringpos++)
		*(reg_data + stringpos) = array[stringpos];
	return 0;
}

void i2c_restart(I2CDriver *i2cp)
{
	i2cStop(i2cp);
	chThdSleepMilliseconds(1);
	i2cStart (i2cp, &bmx160_i2c_cfg);
}

/* Auxiliary function definitions */
int8_t bmm150_aux_read(uint8_t id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {

    (void) id; /* id is unused here */

    return bmi160_aux_read(reg_addr, reg_data, len, &bmi);
}

int8_t bmm150_aux_write(uint8_t id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {

    (void) id; /* id is unused here */

    return bmi160_aux_write(reg_addr, reg_data, len, &bmi);
}
