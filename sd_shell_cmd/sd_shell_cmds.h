#ifndef _SHELL_CMDS_H_
#define _SHELL_CMDS_H_

#include <ch.h>
#include "hal.h"
#include "chprintf.h"
#include <string.h>
#define SHELL_SD         SD1
#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(1024)

#define SHELL_UBX_COG_STATUS		1
#define SHELL_UBX_RATE_STATUS		2
#define SHELL_UBX_RATE_SET			3

enum output_threads{
	GPS = 0,
	YPR,
	GYRO
};

typedef struct{
	uint8_t suspend_state;
	uint8_t test;
	uint8_t gps;
	uint8_t ypr;
	uint8_t gyro;
	uint8_t xbee;
	uint8_t service;

}output_t;

thread_t *cmd_init(void);

void cmd_xbee(BaseSequentialStream* chp, int argc, char* argv[]);
void cmd_attn(BaseSequentialStream* chp, int argc, char* argv[]);

void cmd_start(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_help(BaseSequentialStream* chp, int argc, char* argv[]);
void cmd_c(BaseSequentialStream* chp, int argc, char* argv[]);
void cmd_ublox(BaseSequentialStream* chp, int argc, char* argv[]);
void cmd_xbee(BaseSequentialStream* chp, int argc, char* argv[]);
void start_json_module(void);
void toggle_test_output(void);
void toggle_gps_output(void);
void toggle_ypr_output(void);
void toggle_gyro_output(void);
void stop_all_tests(void);
#endif
