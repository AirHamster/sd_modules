#ifndef _SHELL_CMDS_H_
#define _SHELL_CMDS_H_

#include <ch.h>
#include "hal.h"
#include "chprintf.h"
#include <string.h>
#ifdef USE_ADC_MODULE
#include "adc.h"
#endif
#include "config.h"
#ifdef SD_SENSOR_BOX_LAG
#include "lag.h"
#endif

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(1024)

#define SHELL_UBX_COG_STATUS		1
#define SHELL_UBX_RATE_STATUS		2
#define SHELL_UBX_RATE_SET			3

enum output_threads{
	OUTPUT_NONE = 0,
	OUTPUT_TEST,
	OUTPUT_SERVICE,
	OUTPUT_MAGN_CALIB,
	OUTPUT_GYRO_CALIB,
	OUTPUT_ACCEL_CALIB,
	OUTPUT_ALL_CALIB,
	OUTPUT_RUDDER_CALIB,
	OUTPUT_RUDDER_BLE,
	OUTPUT_LAG_CALIB,
	OUTPUT_LAG_BLE,
	OUTPUT_BLE
};

typedef struct{
	uint8_t type;
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
#ifdef SD_SENSOR_BOX_RUDDER
void send_rudder_over_ble(rudder_t *rudder);
#endif
#ifdef SD_SENSOR_BOX_LAG
void send_lag_over_ble(lag_t *lag);
#endif
uint8_t output_magn_calib(void);
uint8_t output_accel_calib(void);
uint8_t output_gyro_calib(void);
uint8_t output_all_calib(void);
void start_json_module(void);
void toggle_test_output(void);
void toggle_ble_output(void);
void toggle_gps_output(void);
void toggle_ypr_output(void);
void toggle_gyro_output(void);
void stop_all_tests(void);
#endif
