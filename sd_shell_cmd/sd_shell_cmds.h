#ifndef _SHELL_CMDS_H_
#define _SHELL_CMDS_H_

#include <ch.h>
#include "hal.h"
#include "chprintf.h"
#include <string.h>
#define SHELL_SD         SD1
#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(4096*2)

enum output_threads{
	GPS = 0,
	YPR,
	GYRO
};

thread_t *cmd_init(void);

void cmd_xbee(BaseSequentialStream* chp, int argc, char* argv[]);
void cmd_attn(BaseSequentialStream* chp, int argc, char* argv[]);

void cmd_start(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_help(BaseSequentialStream* chp, int argc, char* argv[]);
void cmd_c(BaseSequentialStream* chp, int argc, char* argv[]);
void cmd_ublox(BaseSequentialStream* chp, int argc, char* argv[]);
void cmd_gyro(BaseSequentialStream* chp, int argc, char* argv[]);
void cmd_xbee(BaseSequentialStream* chp, int argc, char* argv[]);

#endif
