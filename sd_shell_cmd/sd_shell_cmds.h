#ifndef _SHELL_CMDS_H_
#define _SHELL_CMDS_H_

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#define	START_STRING	"start"
#define	STOP_STRING		"stop"
#define	SET_PWM_STRING	"pwm"
#define	GAS_STRING		"gas"
#define	WELDING_STRING	"weld"

void cmd_start(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_stop(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_set_pwm(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_gas(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_welding(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_info(BaseSequentialStream *chp, int argc, char *argv[]);

#endif
