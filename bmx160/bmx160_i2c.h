/*
 * bmx160_i2c.h
 *
 *  Created on: Jul 16, 2019
 *      Author: a-h
 */



#ifndef SD_MODULES_bmx160_bmx160_I2C_H_
#define SD_MODULES_bmx160_bmx160_I2C_H_

#include "stdint.h"
#include "ch.h"
#include "hal.h"
//#include "bmx160.h"

#include "bmi160.h"
#include "bmm150.h"
typedef struct{
	uint8_t kek;
}bmx160_t;
#define I2C_BUFFER_LEN 128
void start_bmx160_module(void);
int8_t bmx160_full_init(void);
int8_t bmx160_i2c_routine(bmx160_t *bmx160);
int8_t bmx160_read(uint8_t dev_addr, uint8_t *reg_data, uint8_t r_len);
int8_t bmx160_write(uint8_t dev_addr, uint8_t *reg_data, uint8_t wr_len);
int8_t bmx160_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t bmx160_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t bmx160_read_euler(bmx160_t *bmx160);
int8_t bmx160_read_status(bmx160_t *bmx160);
int8_t bmx160_check_calib_coefs(bmx160_t *bmx160);
int8_t bmx160_start_calibration(bmx160_t *bmx160);
int8_t bmx160_save_calib_to_eeprom(bmx160_t *bmx160);
int8_t bmx160_apply_calib_to_chip(bmx160_t *bmx160);
int8_t bmx160_set_dynamic_calib(bmx160_t *bmx160);
int8_t bmx160_set_static_calib(bmx160_t *bmx160);
int8_t bno955_read_static_flag_from_eeprom(bmx160_t *bmx160);
int8_t bmx160_read_calib_from_eeprom(bmx160_t *bmx160);
int8_t bmx160_save_calib_to_eeprom(bmx160_t *bmx160);

void i2c_restart(I2CDriver *i2cp);
void bmx160_delay_ms(uint16_t msec);
#endif /* SD_MODULES_bmx160_bmx160_I2C_H_ */
