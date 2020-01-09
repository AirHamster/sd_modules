/*
 * mcu-mcu_i2c.h.h
 *
 *  Created on: Nov 26, 2019
 *      Author: a-h
 */

#ifndef SD_MODULES_SUSART_H_
#define SD_MODULES_SUSART_H_

#include "config.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

void susart_init(void);
void _putchar( char ch );

#endif	//SD_MODULES_SUSART_H_
