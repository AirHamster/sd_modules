/*
 * service_mode.h
 *
 *  Created on: Aug 31, 2019
 *      Author: a-h
 */
#include <ch.h>
#include "hal.h"
#include "chprintf.h"
#include "config.h"
#include <string.h>
#ifndef SD_MODULES_SD_SHELL_CMD_SERVICE_MODE_H_
#define SD_MODULES_SD_SHELL_CMD_SERVICE_MODE_H_
void cmd_service(BaseSequentialStream* chp, int argc, char* argv[]);
void cmd_microsd(BaseSequentialStream* chp, int argc, char* argv[]);
void cmd_compass(BaseSequentialStream* chp, int argc, char* argv[]);
void cmd_bno055(BaseSequentialStream* chp, int argc, char* argv[]);
void cmd_ble(BaseSequentialStream* chp, int argc, char* argv[]);
void cmd_rudder(BaseSequentialStream* chp, int argc, char* argv[]);
void cmd_lag(BaseSequentialStream* chp, int argc, char* argv[]);
#ifdef SD_MODULE_TRAINER
void cmd_load_math_cal(BaseSequentialStream* chp, int argc, char* argv[]);
void cmd_get_math_cal(BaseSequentialStream* chp, int argc, char* argv[]);
#endif
#endif /* SD_MODULES_SD_SHELL_CMD_SERVICE_MODE_H_ */
