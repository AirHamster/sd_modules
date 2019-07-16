#ifndef EEPROM_H
#define EEPROM_H

#include <stdlib.h>
#include "stdint.h"
#include "math.h"
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"


#define EEPROM_ADDRESS	0x53
#define EEPROM_HW_VER_ADDR 0x00
#define BNO055_ADDRESS	0x28
#define BNO055_CHIP_ID_ADDR	0x00

void eeprom_read_hw_version(void);
void eeprom_write_hw_version(void);
void bno055_read_id(void);

#endif
