/*
 * bno055_i2c.h
 *
 *  Created on: Jul 16, 2019
 *      Author: a-h
 */

#include "stdint.h"
#include "ch.h"
#include "hal.h"
#include "bno055.h"

#ifndef SD_MODULES_BNO055_BNO055_I2C_H_
#define SD_MODULES_BNO055_BNO055_I2C_H_

#define I2C_BUFFER_LEN 128

/**
 * Starting BNO055 threads
 */
void start_bno055_module(void);

/**
 * Initialize BNO055
 * @param bno055
 * @return
 */
int8_t bno055_full_init(bno055_t *bno055);

/**
 * Initialize BNO055 API struct
 * @param bno055
 * @return
 */
int8_t bno055_i2c_routine(bno055_t *bno055);

/**
 * BNO055 register read API
 * @param dev_addr
 * @param reg_data
 * @param r_len
 * @return
 */
int8_t bno055_read(uint8_t dev_addr, uint8_t *reg_data, uint8_t r_len);

/**
 * BNO055 register write API
 * @param dev_addr
 * @param reg_data
 * @param wr_len
 * @return
 */
int8_t bno055_write(uint8_t dev_addr, uint8_t *reg_data, uint8_t wr_len);

/**
 * Low-level I2C register write API, used in Bosch library
 * @param dev_addr
 * @param reg_addr
 * @param reg_data
 * @param cnt
 * @return
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

/**
 * Low-level I2C register write API, used in Bosch library
 * @param dev_addr
 * @param reg_addr
 * @param reg_data
 * @param cnt
 * @return
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

/**
 * Get euler angles for yaw, pitch, roll
 * @param bno055
 * @return
 */
int8_t bno055_read_euler(bno055_t *bno055);

/**
 * Getting calibration statuses
 * @param bno055
 * @return
 */
int8_t bno055_read_status(bno055_t *bno055);

/**
 * Getting calibration coeffs from BNO055 chip
 * @param bno055
 * @return
 */
int8_t bno055_check_calib_coefs(bno055_t *bno055);

/**
 * Reset chip and initializing calibration procedure
 * @param bno055
 * @return
 */
int8_t bno055_start_calibration(bno055_t *bno055);

/**
 * Writes calibration parameters to EEPROM
 * @param bno055
 * @return
 */
int8_t bno055_save_calib_to_eeprom(bno055_t *bno055);

/**
 * Rewrite calibration parameters to chip
 * @param bno055
 * @return
 */
int8_t bno055_apply_calib_to_chip(bno055_t *bno055);

/**
 * Allow dynamic calibration
 * @param bno055
 * @return
 */
int8_t bno055_set_dynamic_calib(bno055_t *bno055);

/**
 * Forbid dynamic calibration
 * @param bno055
 * @return
 */
int8_t bno055_set_static_calib(bno055_t *bno055);

/**
 * Reads static calibration flag from EEPROM
 * @param bno055
 * @return
 */
int8_t bno955_read_static_flag_from_eeprom(bno055_t *bno055);

/**
 * Reads calibration parameters from EEPROM
 * @param bno055
 * @return
 */
int8_t bno055_read_calib_from_eeprom(bno055_t *bno055);

/**
 * Writes calibration parameters to EEPROM
 * @param bno055
 * @return
 */
int8_t bno055_save_calib_to_eeprom(bno055_t *bno055);

/**
 * Restart I2C in case of bus error
 * @param i2cp
 */
void i2c_restart(I2CDriver *i2cp);

/**
 * Delay API used in Bosch library
 * @param msec
 */
void bno055_delay_ms(uint16_t msec);

#endif /* SD_MODULES_BNO055_BNO055_I2C_H_ */
