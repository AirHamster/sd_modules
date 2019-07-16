/*
 * bno055_i2c.h
 *
 *  Created on: Jul 16, 2019
 *      Author: a-h
 */

#include "stdint.h"

#ifndef SD_MODULES_BNO055_BNO055_I2C_H_
#define SD_MODULES_BNO055_BNO055_I2C_H_

#define I2C_BUFFER_LEN 128
int8_t bno055_i2c_routine(struct bno055_t *bno055);
int8_t bno055_read(uint8_t dev_addr, uint8_t *reg_data, uint8_t r_len);
int8_t bno055_write(uint8_t dev_addr, uint8_t *reg_data, uint8_t wr_len);
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void bno055_delay_ms(uint16_t msec);
#endif /* SD_MODULES_BNO055_BNO055_I2C_H_ */
