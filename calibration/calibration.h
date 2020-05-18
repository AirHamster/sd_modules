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

int8_t calib_update_compass_correction(uint8_t dev_num, float calib_val);
int8_t calib_update_hsp_correction(uint8_t dev_num, float calib_val);
int8_t calib_update_heel_correction(uint8_t dev_num, float calib_val);
int8_t calib_update_magnetic_declanation(uint8_t dev_num, float calib_val);
int8_t calib_update_pitch_correction(uint8_t dev_num, float calib_val);
int8_t calib_update_rudder_correction(uint8_t dev_num, float calib_val);
int8_t calib_update_wind_correction(uint8_t dev_num, float calib_val);
int8_t calib_update_window_size_1(uint8_t dev_num, uint8_t calib_val_i);
int8_t calib_update_window_size_2(uint8_t dev_num, uint8_t calib_val_i);
int8_t calib_update_window_size_3(uint8_t dev_num, uint8_t calib_val_i);
int8_t calib_update_rudder_left(uint8_t dev_num, float calib_val);
int8_t calib_update_rudder_center(uint8_t dev_num, float calib_val);
int8_t calib_update_rudder_right(uint8_t dev_num, float calib_val);

#endif
