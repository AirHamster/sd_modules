#ifndef EEPROM_H
#define EEPROM_H

#include <stdlib.h>
#include "stdint.h"
#include "config.h"
#include "math.h"
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

#ifdef SD_MODULE_TRAINER
#define EEPROM_ADDRESS 0x57
#endif

#ifdef SENSOR_BOX
#define EEPROM_ADDRESS	0x50
#endif

#define EEPROM_INFO_MEMORY_START		0x0000	//32 bytes for info
#define EEPROM_FLAGS_MEMORY_START		0x001F	//32 bytes for flags
#define EEPROM_CALIB_MEMORY_START		0x003F

#define EEPROM_HW_VER_ADDR 				EEPROM_INFO_MEMORY_START
#define EEPROM_SW_VER_ADDR 				EEPROM_INFO_MEMORY_START + 1

#define EEPROM_STATIC_CALIB_FLAG_ADDR	EEPROM_FLAGS_MEMORY_START

#define EEPROM_MAGN_X_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START
#define EEPROM_MAGN_Y_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 2
#define EEPROM_MAGN_Z_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 4
#define EEPROM_MAGN_RADIUS_ADDR			EEPROM_CALIB_MEMORY_START + 6

#define EEPROM_GYRO_X_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 8
#define EEPROM_GYRO_Y_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 10
#define EEPROM_GYRO_Z_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 12

#define EEPROM_ACCEL_X_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 14
#define EEPROM_ACCEL_Y_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 16
#define EEPROM_ACCEL_Z_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 18
#define EEPROM_ACCEL_RADIUS_ADDR		EEPROM_CALIB_MEMORY_START + 20

#define EEPROM_RUDDER_CALIB_LEFT		EEPROM_CALIB_MEMORY_START + 22
#define EEPROM_RUDDER_CALIB_RIGHT		EEPROM_CALIB_MEMORY_START + 24
#define EEPROM_RUDDER_CALIB_CENTER		EEPROM_CALIB_MEMORY_START + 26


#define BNO055_ADDRESS	0x28

int8_t eeprom_write(uint16_t byte_addr, const int8_t *txbuf, size_t txbytes);
int8_t eeprom_read(uint16_t byte_addr, int8_t *rxbuf, size_t rxbytes);
void eeprom_check_i2c_bus(void);
void eeprom_read_hw_version(void);
void eeprom_write_hw_version(void);
void bno055_read_id(void);
void start_eeprom_module(void);
#endif
