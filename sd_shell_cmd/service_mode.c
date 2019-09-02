/*
 * service_mode.c
 *
 *  Created on: Aug 31, 2019
 *      Author: a-h
 */
#include "config.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "shellconf.h"
#include "chprintf.h"
#ifdef USE_UBLOX_GPS_MODULE
#include "neo-m8.h"
extern ubx_nav_pvt_t *pvt_box;
#endif
#ifdef USE_SD_SHELL
#include "sd_shell_cmds.h"
extern output_t *output;
#endif
#ifdef USE_BNO055_MODULE
#include "bno055_i2c.h"
extern bno055_t *bno055;
#endif
#ifdef USE_MICROSD_MODULE
#include "microsd.h"
#endif
#ifdef USE_WINDSENSOR_MODULE
#include "windsensor.h"
extern windsensor_t *wind;
#endif

void cmd_service(BaseSequentialStream* chp, int argc, char* argv[]) {
	if (argc != 0) {
		if (strcmp(argv[0], "help") == 0) {
			chprintf(chp, "\r\nActive modules and available commands:\n\r");
#ifdef USE_UBLOX_GPS_MODULE
			chprintf(chp, "  - gps\n\r");
#endif
#ifdef USE_BNO055_MODULE
			chprintf(chp,
					"  - gyro status|calibrate|get_cal_params|write_cal_params\n\r");
#endif
#ifdef USE_MICROSD_MODULE
			chprintf(chp, "  - microsd info|mount|umount|mkfs|ls|tree|cat\n\r");
#endif
#ifdef USE_WINDSENSOR_MODULE
			chprintf(chp, "  - wind\n\r");
#endif
			return;
		}
		chprintf(chp, "Usage: service <help>\n\r");
	}
	stop_all_tests();
	output->service = 1;
	chprintf(chp,
			"\r\nService mode activated. Write <service help> to get more info\n\r");
}

#ifdef USE_BNO055_MODULE
void cmd_gyro(BaseSequentialStream* chp, int argc, char* argv[]) {

	if (!output->service) {
		return;
	}
	if (argc != 0) {
		if (strcmp(argv[0], "help") == 0) {
			chprintf(chp,
					"Usage: gyro status|calibrate|get_cal_params|write_cal_params\r\n");
		} else if (strcmp(argv[0], "calibrate") == 0) {
			chprintf(chp, "Starting gyro calibration routine\r\n");
		} else if (strcmp(argv[0], "get_cal_params") == 0) {
			chprintf(chp, "BNO055 calibration parameters:\r\n");
			chprintf(chp, "\t- magn x offset: \r\n");
			chprintf(chp, "\t- magn y offset: \r\n");
			chprintf(chp, "\t- magn z offset: \r\n");
			chprintf(chp, "\t- magn radius: \r\n");

			chprintf(chp, "\t- gyro x offset: \r\n");
			chprintf(chp, "\t- gyro y offset: \r\n");
			chprintf(chp, "\t- gyro z offset: \r\n");

			chprintf(chp, "\t- accel x offset: \r\n");
			chprintf(chp, "\t- accel y offset: \r\n");
			chprintf(chp, "\t- accel z offset: \r\n");
		} else if (strcmp(argv[0], "write_cal_params") == 0) {

			chprintf(chp, "Writing calibration parameters to EEPROM:\r\n");
			chprintf(chp, "magn x offset: \r\n");
			chprintf(chp, "magn y offset: \r\n");
			chprintf(chp, "magn z offset: \r\n");
			chprintf(chp, "magn radius: \r\n");

			chprintf(chp, "gyro x offset: \r\n");
			chprintf(chp, "gyro y offset: \r\n");
			chprintf(chp, "gyro z offset: \r\n");

			chprintf(chp, "accel x offset: \r\n");
			chprintf(chp, "accel y offset: \r\n");
			chprintf(chp, "accel z offset: \r\n");
		} else if (strcmp(argv[0], "status") == 0) {
			chprintf(chp,
					"BNO055 status:\r\n\t- compass: \r\n\t- gyro: \r\n\t- accel: \r\n\t- system: \r\n");
		}
	}
}
#endif

#ifdef USE_UBLOX_GPS_MODULE
void cmd_gps(BaseSequentialStream* chp, int argc, char* argv[]) {
	if (!output->service) {
		return;
	}
}
#endif

#ifdef USE_MICROSD_MODULE
void cmd_microsd(BaseSequentialStream* chp, int argc, char* argv[]) {
	if (!output->service) {
		return;
	}
	if (argc != 0) {
		if (strcmp(argv[0], "help") == 0) {
			chprintf(chp,
					"Usage: - microsd info|mount|umount|mkfs|ls|tree|cat\n\r");
		} else if (strcmp(argv[0], "info") == 0) {
			chprintf(chp, "microSD information:\r\n");
		} else if (strcmp(argv[0], "mount") == 0) {
			chprintf(chp, "Trying to mount SD card...\r\n");
		} else if (strcmp(argv[0], "umount") == 0) {
			chprintf(chp, "Trying to umount SD card...\r\n");
		} else if (strcmp(argv[0], "mkfs") == 0) {
			chprintf(chp, "Formatting SD card... \r\n");
		} else if (strcmp(argv[0], "tree") == 0) {
			chprintf(chp, "Tree \r\n");
		} else if (strcmp(argv[0], "ls") == 0) {
			chprintf(chp, "LS \r\n");
		}
	}
}
#endif

#ifdef USE_WINDSENSOR_MODULE
void cmd_wind(BaseSequentialStream* chp, int argc, char* argv[]) {
	if (!output->service) {
		return;
	}
}
#endif

#ifdef USE_RUDDER_MODULE
void cmd_rudder(BaseSequentialStream* chp, int argc, char* argv[]) {
	if (!output->service) {
		return;
	}
	if (argc != 0) {
		if (strcmp(argv[0], "help") == 0) {
			chprintf(chp,
					"Usage: - rudder calibrate|left|middle|right\n\r");
		} else if (strcmp(argv[0], "calibrate") == 0) {
			chprintf(chp, "Starting rudder calibration procedure\r\n");
		} else if (strcmp(argv[0], "left") == 0) {
			chprintf(chp, "Saved left position\r\n");
		} else if (strcmp(argv[0], "middle") == 0) {
			chprintf(chp, "Saved middle position\r\n");
		} else if (strcmp(argv[0], "right") == 0) {
			chprintf(chp, "Saved right position\r\n");
		}
	}
}

#endif


#ifdef USE_LAG_MODULE
#endif
