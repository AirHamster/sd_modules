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
#include "service_mode.h"
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
#ifdef USE_BNO055_MODULE
#include "bno055_i2c.h"
#include "bno055.h"
extern bno055_t *bno055;
#endif
#ifdef USE_BLE_MODULE
#include "nina-b3.h"
extern ble_t *ble;
#endif
extern struct ch_semaphore usart1_semaph;

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
#ifdef USE_BLE_MODULE
			chprintf(chp, "  - ble AT|NULL|NULL\n\r");
#endif
#ifdef USE_WINDSENSOR_MODULE
			chprintf(chp, "  - wind\n\r");
#endif
			return;
		}
		chprintf(chp, "Usage: service <help>\n\r");
	}
	stop_all_tests();
	output->type = OUTPUT_SERVICE;
	chprintf(chp,
			"\r\nService mode activated. Write <service help> to get more info\n\r");
}

#ifdef USE_BNO055_MODULE
void cmd_gyro(BaseSequentialStream* chp, int argc, char* argv[]) {

	if (output->type != OUTPUT_SERVICE) {
		return;
	}
	if (argc != 0) {
		if (strcmp(argv[0], "help") == 0) {
			chprintf(chp,
					"Usage: gyro status|calibrate|get_cal_params|write_cal_params\r\n");
		} else if (strcmp(argv[0], "calibrate") == 0) {
			chprintf(chp, "Starting gyro calibration routine\r\n");
			bno055_start_calibration(bno055);
			output->type = OUTPUT_ALL_CALIB;
		} else if (strcmp(argv[0], "get_cal_params") == 0) {
			chprintf(chp, "BNO055 calibration parameters:\r\n");
			bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
			bno055_check_calib_coefs(bno055);
			chSemWait(&usart1_semaph);
			chprintf(SHELL_IFACE,
					"\r\n{\"msg_type\":\"calib_data\",\r\n\t\t\"boat_1\":{\r\n\t\t\t");
			chprintf(SHELL_IFACE, "\"magn_cal\":%d,\r\n\t\t\t",
					bno055->calib_stat.magn);
			chprintf(SHELL_IFACE, "\"accel_cal\":%d,\r\n\t\t\t",
					bno055->calib_stat.accel);
			chprintf(SHELL_IFACE, "\"gyro_cal\":%d,\r\n\t\t\t",
					bno055->calib_stat.gyro);
			chprintf(SHELL_IFACE, "\"sys_cal\":%d,\r\n\t\t\t",
					bno055->calib_stat.system);
			chprintf(SHELL_IFACE, "\"magn_x_offset\":%x,\r\n\t\t\t",
					(int16_t) bno055->magn_offset.x);
			chprintf(SHELL_IFACE, "\"magn_y_offset\":%x,\r\n\t\t\t",
					(int16_t) bno055->magn_offset.y);
			chprintf(SHELL_IFACE, "\"magn_z_offset\":%x,\r\n\t\t\t",
					(int16_t) bno055->magn_offset.z);
			chprintf(SHELL_IFACE, "\"magn_radius\":  %x,\r\n\t\t\t",
					(int16_t) bno055->magn_offset.r);
			chprintf(SHELL_IFACE, "\"gyro_x_offset\":%x,\r\n\t\t\t",
					(int16_t) bno055->gyro_offset.x);
			chprintf(SHELL_IFACE, "\"gyro_y_offset\":%x,\r\n\t\t\t",
					(int16_t) bno055->gyro_offset.y);
			chprintf(SHELL_IFACE, "\"gyro_z_offset\":%x,\r\n\t\t\t",
					(int16_t) bno055->gyro_offset.z);
			chprintf(SHELL_IFACE, "\"accel_x_offset\":%x,\r\n\t\t\t",
					(int16_t) bno055->accel_offset.x);
			chprintf(SHELL_IFACE, "\"accel_y_offset\":%x,\r\n\t\t\t",
					(int16_t) bno055->accel_offset.y);
			chprintf(SHELL_IFACE, "\"accel_z_offset\":%x,\r\n\t\t\t",
					(int16_t) bno055->accel_offset.z);
			chprintf(SHELL_IFACE, "\"accel_radius\":%x,\r\n\t\t\t",
					(int16_t) bno055->accel_offset.r);
			chprintf(SHELL_IFACE, "}\r\n\t}\r\n");
			chSemSignal(&usart1_semaph);
			bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
		} else if (strcmp(argv[0], "save_cal_params") == 0) {
			chSemWait(&usart1_semaph);
			chprintf(chp, "Writing calibration parameters to EEPROM:\r\n");
			chprintf(SHELL_IFACE,
					"\r\n{\"msg_type\":\"calib_data\",\r\n\t\t\"boat_1\":{\r\n\t\t\t");
			chprintf(SHELL_IFACE, "\"magn_cal\":%d,\r\n\t\t\t",
					bno055->calib_stat.magn);
			chprintf(SHELL_IFACE, "\"accel_cal\":%d,\r\n\t\t\t",
					bno055->calib_stat.accel);
			chprintf(SHELL_IFACE, "\"gyro_cal\":%d,\r\n\t\t\t",
					bno055->calib_stat.gyro);
			chprintf(SHELL_IFACE, "\"sys_cal\":%d,\r\n\t\t\t",
					bno055->calib_stat.system);
			chprintf(SHELL_IFACE, "\"magn_x_offset\":%x,\r\n\t\t\t",
					(int16_t) bno055->magn_offset.x);
			chprintf(SHELL_IFACE, "\"magn_y_offset\":%x,\r\n\t\t\t",
					(int16_t) bno055->magn_offset.y);
			chprintf(SHELL_IFACE, "\"magn_z_offset\":%x,\r\n\t\t\t",
					(int16_t) bno055->magn_offset.z);
			chprintf(SHELL_IFACE, "\"magn_radius\":  %x,\r\n\t\t\t",
					(int16_t) bno055->magn_offset.r);
			chprintf(SHELL_IFACE, "\"gyro_x_offset\":%x,\r\n\t\t\t",
					(int16_t) bno055->gyro_offset.x);
			chprintf(SHELL_IFACE, "\"gyro_y_offset\":%x,\r\n\t\t\t",
					(int16_t) bno055->gyro_offset.y);
			chprintf(SHELL_IFACE, "\"gyro_z_offset\":%x,\r\n\t\t\t",
					(int16_t) bno055->gyro_offset.z);
			chprintf(SHELL_IFACE, "\"accel_x_offset\":%x,\r\n\t\t\t",
					(int16_t) bno055->accel_offset.x);
			chprintf(SHELL_IFACE, "\"accel_y_offset\":%x,\r\n\t\t\t",
					(int16_t) bno055->accel_offset.y);
			chprintf(SHELL_IFACE, "\"accel_z_offset\":%x,\r\n\t\t\t",
					(int16_t) bno055->accel_offset.z);
			chprintf(SHELL_IFACE, "\"accel_radius\":%x,\r\n\t\t\t",
					(int16_t) bno055->accel_offset.r);
			chprintf(SHELL_IFACE, "}\r\n\t}\r\n");
			chSemSignal(&usart1_semaph);
			bno055_save_calib_to_eeprom(bno055);
		} else if (strcmp(argv[0], "set_static_cal") == 0) {
			if (argc == 2) {
				if (strcmp(argv[1], "1") == 0) {
					bno055_set_static_calib(bno055);
				} else if (strcmp(argv[1], "0") == 0) {
					bno055_set_dynamic_calib(bno055);
				} else {
					chprintf(chp, "Usage: gyro set_static_cal [0, 1]\n\r");
				}
			} else {
				chprintf(chp, "Usage: gyro set_static_cal [0, 1]\n\r");
			}
		} else if (strcmp(argv[0], "status") == 0) {
			chprintf(chp,
					"BNO055 status:\r\n\t- compass: \r\n\t- gyro: \r\n\t- accel: \r\n\t- system: \r\n");
		}
	}
}
#endif

#ifdef USE_UBLOX_GPS_MODULE
void cmd_gps(BaseSequentialStream* chp, int argc, char* argv[]) {
	if (output->type != OUTPUT_SERVICE) {
		return;
	}
}
#endif

#ifdef USE_BLE_MODULE
void cmd_ble(BaseSequentialStream* chp, int argc, char* argv[]) {
	if (output->type != OUTPUT_SERVICE) {
		return;
	}

	if (argc != 0) {
			if (strcmp(argv[0], "help") == 0) {
				chprintf(chp,
						"Usage: gyro status|calibrate|get_cal_params|write_cal_params\r\n");
			} else if (strcmp(argv[0], "calibrate") == 0) {
				chprintf(chp, "Starting gyro calibration routine\r\n");
				bno055_start_calibration(bno055);
				output->type = OUTPUT_ALL_CALIB;
			} else if (strcmp(argv[0], "get_cal_params") == 0) {
				chprintf(chp, "BNO055 calibration parameters:\r\n");
			}
	}

}
#endif


#ifdef USE_MICROSD_MODULE
void cmd_microsd(BaseSequentialStream* chp, int argc, char* argv[]) {
	if (output->type != OUTPUT_SERVICE) {
		return;
	}
	if (argc != 0) {
		if (strcmp(argv[0], "help") == 0) {
			chprintf(chp, "Usage: - ble AT|NULL|NULL\n\r");
		} else if (strcmp(argv[0], "AT") == 0) {
			chprintf(chp, "Sending AT request:\r\n");
			nina_send_at();
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
	if (output->type != OUTPUT_SERVICE) {
		return;
	}
}
#endif

#ifdef USE_RUDDER_MODULE
void cmd_rudder(BaseSequentialStream* chp, int argc, char* argv[]) {
	if (output->type != OUTPUT_SERVICE) {
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
