#ifndef EEPROM_H
#define EEPROM_H

#include <stdlib.h>
#include "stdint.h"
#include "math.h"
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"


#define EEPROM_ADDRESS	0x57
#define EEPROM_MEMORY_START		0x0001
#define EEPROM_HW_VER_ADDR 		(EEPROM_MEMORY_START)

#define EEPROM_MAGN_X_OFFSET_ADDR		EEPROM_MEMORY_START + 4
#define EEPROM_MAGN_Y_OFFSET_ADDR		EEPROM_MEMORY_START + 8
#define EEPROM_MAGN_Z_OFFSET_ADDR		EEPROM_MEMORY_START + 12
#define EEPROM_MAGN_RADIUS_ADDR		EEPROM_MEMORY_START + 16

#define EEPROM_GYRO_X_OFFSET_ADDR		EEPROM_MEMORY_START + 20
#define EEPROM_GYRO_Y_OFFSET_ADDR		EEPROM_MEMORY_START + 24
#define EEPROM_GYRO_Z_OFFSET_ADDR		EEPROM_MEMORY_START + 28

#define EEPROM_ACCEL_X_OFFSET_ADDR		EEPROM_MEMORY_START + 32
#define EEPROM_ACCEL_Y_OFFSET_ADDR		EEPROM_MEMORY_START + 36
#define EEPROM_ACCEL_Z_OFFSET_ADDR		EEPROM_MEMORY_START + 40

#define BNO055_ADDRESS	0x28
#define BNO055_CHIP_ID_ADDR	0x00

void eeprom_read_hw_version(void);
void eeprom_write_hw_version(void);
void bno055_read_id(void);

#endif
