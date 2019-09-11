/*
 * service_mode.h
 *
 *  Created on: Aug 31, 2019
 *      Author: a-h
 */
#include <ch.h>
#include "hal.h"
#include "chprintf.h"
#include <string.h>
#ifndef SD_MODULES_SD_SHELL_CMD_SERVICE_MODE_H_
#define SD_MODULES_SD_SHELL_CMD_SERVICE_MODE_H_
void cmd_service(BaseSequentialStream* chp, int argc, char* argv[]);
void cmd_microsd(BaseSequentialStream* chp, int argc, char* argv[]);
void cmd_gyro(BaseSequentialStream* chp, int argc, char* argv[]);

#endif /* SD_MODULES_SD_SHELL_CMD_SERVICE_MODE_H_ */
