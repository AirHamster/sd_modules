#ifndef _CALIBRATIONS_H_
#define _CALIBRATIONS_H_

#include <ch.h>
#include "hal.h"
#include "chprintf.h"
#include <string.h>
#include "config.h"
#include "adc.h"


typedef struct{
	rudder_t rudder_calib;
	dots_t	rudder_dots;
	coefs_t rudder_coefs;
	float MagneticDeclanation;
	float WindCorrection;
	float PitchCorrection;
	float HeelCorrection;
	float CompassCorrection;
	float HSPCorrection;
	float RudderCorrection;
	uint8_t WindowSize1;
	uint8_t WindowSize2;
	uint8_t WindowSize3;
} calib_parameters_t;


void cmd_get_math_cal(BaseSequentialStream* chp, int argc, char* argv[]);
void cmd_load_math_cal(BaseSequentialStream* chp, int argc, char* argv[]);
void calib_init_params(void);
void calib_parse_shell_command(BaseSequentialStream* chp, char* argv[]);
void calib_print_calib_to_shell(BaseSequentialStream* chp, uint8_t dev_num);

#endif
