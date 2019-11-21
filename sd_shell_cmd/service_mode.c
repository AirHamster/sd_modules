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
#endif //USE_UBLOX_GPS_MODULE
#ifdef USE_SD_SHELL
#include "sd_shell_cmds.h"
extern output_t *output;
#endif //USE_SD_SHELL
#ifdef USE_MICROSD_MODULE
#include "microsd.h"
#endif //USE_MICROSD_MODULE
#ifdef USE_WINDSENSOR_MODULE
#include "windsensor.h"
extern windsensor_t *wind;
#endif //USE_WINDSENSOR_MODULE
#ifdef USE_BNO055_MODULE
#include "bno055_i2c.h"
#include "bno055.h"
extern bno055_t *bno055;
#endif
#ifdef USE_BLE_MODULE
#include "nina-b3.h"
#include "adc.h"
extern ble_t *ble;
//extern lag_t *r_lag;
extern rudder_t *r_rudder;
#endif //USE_BNO055_MODULE
#ifdef USE_ADC_MODULE
#include "adc.h"
extern coefs_t *coefs;
extern dots_t *dots;
extern rudder_t *rudder;
#endif //USE_ADC_MODULE
#ifdef USE_EEPROM_MODULE
#include "eeprom.h"
#endif //USE_EEPROM_MODULE
#ifdef SD_SENSOR_BOX_LAG
#include "lag.h"
extern lag_t *lag;
#endif

#ifdef USE_HMC6343_MODULE
#include "hmc6343_i2c.h"
extern hmc6343_t *hmc6343;
#endif

#ifdef USE_HMC5883_MODULE
#include "hmc5883_i2c.h"
extern hmc5883_t *hmc6343;
#endif

#include "adc.h"
extern dots_t *r_rudder_dots;
extern coefs_t *r_rudder_coefs;

#ifdef SD_MODULE_TRAINER
#ifdef USE_MATH_MODULE
#include "sailDataMath.h"
#include "sd_math.h"
extern CalibrationParmDef paramSD;
#endif
#endif
extern struct ch_semaphore usart1_semaph;

void cmd_service(BaseSequentialStream* chp, int argc, char* argv[]) {
	if (argc != 0) {
		chSemWait(&usart1_semaph);
		if (strcmp(argv[0], "help") == 0) {
			chprintf(chp, "\r\nActive modules and available commands:\n\r");
#ifdef USE_UBLOX_GPS_MODULE
			chprintf(chp, "  - gps\n\r");
#endif
#ifdef USE_HMC6343_MODULE
			chprintf(chp,
					"  - compass status|calibrate|get_cal_params|write_cal_params\n\r");
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
		chSemSignal(&usart1_semaph);
	}
	stop_all_tests();
	output->type = OUTPUT_SERVICE;
	chSemWait(&usart1_semaph);
	chprintf(chp,
			"\r\nService mode activated. Write <service help> to get more info\n\r");
	chSemSignal(&usart1_semaph);
}

#ifdef USE_HMC6343_MODULE
void cmd_compass(BaseSequentialStream* chp, int argc, char* argv[]) {
	float dest1[3];
	float dest2[3];
	if (output->type != OUTPUT_SERVICE) {
		return;
	}
	if (argc != 0) {
		if (strcmp(argv[0], "help") == 0) {
			chprintf(chp,
					"Usage: compass status|calibrate|get_cal_params|write_cal_params|set_static_cal|get_static_cal\r\n");
		} else if (strcmp(argv[0], "calibrate") == 0) {
			chprintf(chp, "Starting compass calibration routine\r\n");
			//bno055_start_calibration(bno055);
			hmc6343_start_calibration(hmc6343);
			output->type = OUTPUT_ALL_CALIB;
		//	hmc5883_calibration(dest1, dest2);
		} else if (strcmp(argv[0], "get_cal_params") == 0) {
			chprintf(chp, "HMC6343 calibration parameters:\r\n");
			bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
			bno055_check_calib_coefs(bno055);
			chSemWait(&usart1_semaph);
			chprintf(SHELL_IFACE,
					"\r\n{\"msg_type\":\"gyro_calib_data\",\r\n\t\t\"boat_1\":{\r\n\t\t\t");
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
					"\r\n{\"msg_type\":\"gyro_calib_data\",\r\n\t\t\"boat_1\":{\r\n\t\t\t");
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
					if (bno055_set_static_calib(bno055) == -1){
						chprintf(chp, "Error writing parameters to EEPROM\n\r");
					}else{
						chprintf(chp, "Static calibration set\n\r");
					}
				} else if (strcmp(argv[1], "0") == 0) {
					if (bno055_set_dynamic_calib(bno055) == -1){
						chprintf(chp, "Error writing parameters to EEPROM\n\r");
					}else{
						chprintf(chp, "Dynamic calibration set\n\r");
					}
				} else {
					chprintf(chp, "Usage: gyro set_static_cal [0, 1]\n\r");
				}
			} else {
				chprintf(chp, "Usage: gyro set_static_cal [0, 1]\n\r");
			}
		}else if (strcmp(argv[0], "get_static_cal") == 0) {
			if (bno955_read_static_flag_from_eeprom(bno055) == -1){
				chprintf(chp, "Error reading parameters from EEPROM\n\r");
			}else{
				chprintf(chp, "Static calibration flag: %d\n\r", bno055->static_calib);
			}

		}else if (strcmp(argv[0], "status") == 0) {
			chprintf(chp,
					"BNO055 status:\r\n\t- compass: \r\n\t- gyro: \r\n\t- accel: \r\n\t- system: \r\n");
		}
	}
}
#endif

#ifdef USE_BNO055_MODULE
void cmd_bno055(BaseSequentialStream* chp, int argc, char* argv[]) {

	if (output->type != OUTPUT_SERVICE) {
		return;
	}
	if (argc != 0) {
		if (strcmp(argv[0], "help") == 0) {
			chprintf(chp,
					"Usage: gyro status|calibrate|get_cal_params|write_cal_params|set_static_cal|get_static_cal\r\n");
		} else if (strcmp(argv[0], "calibrate") == 0) {
			chprintf(chp, "Starting gyro calibration routine\r\n");
			//bno055_start_calibration(bno055);
			//output->type = OUTPUT_ALL_CALIB;
		//	hmc5883_calibration(dest1, dest2);
		} else if (strcmp(argv[0], "get_cal_params") == 0) {
			chprintf(chp, "BNO055 calibration parameters:\r\n");
			bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
			bno055_check_calib_coefs(bno055);
			chSemWait(&usart1_semaph);
			chprintf(SHELL_IFACE,
					"\r\n{\"msg_type\":\"gyro_calib_data\",\r\n\t\t\"boat_1\":{\r\n\t\t\t");
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
					"\r\n{\"msg_type\":\"gyro_calib_data\",\r\n\t\t\"boat_1\":{\r\n\t\t\t");
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
					if (bno055_set_static_calib(bno055) == -1){
						chprintf(chp, "Error writing parameters to EEPROM\n\r");
					}else{
						chprintf(chp, "Static calibration set\n\r");
					}
				} else if (strcmp(argv[1], "0") == 0) {
					if (bno055_set_dynamic_calib(bno055) == -1){
						chprintf(chp, "Error writing parameters to EEPROM\n\r");
					}else{
						chprintf(chp, "Dynamic calibration set\n\r");
					}
				} else {
					chprintf(chp, "Usage: gyro set_static_cal [0, 1]\n\r");
				}
			} else {
				chprintf(chp, "Usage: gyro set_static_cal [0, 1]\n\r");
			}
		}else if (strcmp(argv[0], "get_static_cal") == 0) {
			if (bno955_read_static_flag_from_eeprom(bno055) == -1){
				chprintf(chp, "Error reading parameters from EEPROM\n\r");
			}else{
				chprintf(chp, "Static calibration flag: %d\n\r", bno055->static_calib);
			}

		}else if (strcmp(argv[0], "status") == 0) {
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
						"Usage: ble at|init|one|two\r\n");
			} else if (strcmp(argv[0], "at") == 0) {
				chprintf(chp, "sending AT request\r\n");
				nina_send_at();
			} else if (strcmp(argv[0], "init") == 0) {
				chprintf(chp, "Initialization NINA module\r\n");
				nina_init_module();
			} else if (strcmp(argv[0], "one") == 0) {
				chprintf(chp, "Sending one\r\n");
				//nina_send_one(1);
			}else if (strcmp(argv[0], "two") == 0) {
				chprintf(chp, "sending two\r\n");
			//	nina_send_two();
			}else if (strcmp(argv[0], "start") == 0) {
				chprintf(chp, "starting\r\n");
				toggle_ble_output();
			}else if (strcmp(argv[0], "conn_lag") == 0){
				chprintf(chp, "Connecting to lag module\r\n");
				nina_connect("CCF95781688F", 0);
			}else if (strcmp(argv[0], "conn_rudder") == 0){
				chprintf(chp, "Connecting to rudder module\r\n");
				nina_connect("CCF957816647", 0);
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
			chprintf(chp, "Usage: - microsd mount|umount|mkfs|tree|ls\n\r");
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
			//cmd_tree(chp);
		}
	}
}
#endif

#ifdef SD_MODULE_TRAINER
#ifdef USE_MATH_MODULE
void cmd_get_math_cal(BaseSequentialStream* chp, int argc, char* argv[]) {
	chSemWait(&usart1_semaph);
	chprintf(SHELL_IFACE,
			"\r\n{\"msg_type\":\"math_calib_data\",\r\n\t\t\"boat_1\":{\r\n\t\t\t");
	chprintf(SHELL_IFACE, "\"CompassCorrection\":%f,\r\n\t\t\t",
			paramSD.CompassCorrection);
	chprintf(SHELL_IFACE, "\"HSPCorrection\":%f,\r\n\t\t\t",
			paramSD.HSPCorrection);
	chprintf(SHELL_IFACE, "\"HeelCorrection\":%f,\r\n\t\t\t",
			paramSD.HeelCorrection);
	chprintf(SHELL_IFACE, "\"MagneticDeclanation\":%f,\r\n\t\t\t",
			paramSD.MagneticDeclanation);
	chprintf(SHELL_IFACE, "\"PitchCorrection\":%f,\r\n\t\t\t",
			paramSD.PitchCorrection);
	chprintf(SHELL_IFACE, "\"RudderCorrection\":%f,\r\n\t\t\t",
			paramSD.RudderCorrection);
	chprintf(SHELL_IFACE, "\"WindCorrection\":%f,\r\n\t\t\t",
			paramSD.WindCorrection);
	chprintf(SHELL_IFACE, "\"WindowSize1\":%d,\r\n\t\t\t",
			paramSD.WindowSize1);
	chprintf(SHELL_IFACE, "\"WindowSize2\":%d,\r\n\t\t\t",
			paramSD.WindowSize2);
	chprintf(SHELL_IFACE, "\"WindowSize3\":%d,\r\n\t\t\t",
			paramSD.WindowSize3);
	chprintf(SHELL_IFACE, "}\r\n\t}\r\n");
	chSemSignal(&usart1_semaph);
}

void cmd_load_math_cal(BaseSequentialStream* chp, int argc, char* argv[]) {
	float calib_val;
	int16_t calib_val_i;
	if (argc != 0) {
		if (strcmp(argv[0], "help") == 0) {
			chprintf(chp,
					"Usage: - load_math_calib CompassCorrection|HSPCorrection|HeelCorrection|MagneticDeclanation|PitchCorrection|RudderCorrection|WindCorrection|WindowSize1|WindowSize2|WindowSize3 <float>\n\r");
		} else if (strcmp(argv[0], "CompassCorrection") == 0) {
			if (strlen(argv[1]) != 0) {
				calib_val = atof(argv[1]);
				//chprintf(chp, "Writing new calib number: %f\r\n", calib_val);
				eeprom_write(EEPROM_MATH_COMPASS_CORRECTION,
						(uint8_t*) &calib_val, 4);
				chprintf(chp, "Saved CompassCorrection value: %f\r\n",
						calib_val);
				paramSD.CompassCorrection = calib_val;
				microsd_update_calibfile();
			} else {
				chprintf(chp, "Error: no value\r\n");
			}
		} else if (strcmp(argv[0], "HSPCorrection") == 0) {
			if (strlen(argv[1]) != 0) {
				calib_val = atof(argv[1]);
				//chprintf(chp, "Writing new calib number: %f\r\n", calib_val);
				eeprom_write(EEPROM_MATH_HSP_CORRECTION, (uint8_t*) &calib_val,
						4);
				chprintf(chp, "Saved HSPCorrection value: %f\r\n", calib_val);
				paramSD.HSPCorrection = calib_val;
				microsd_update_calibfile();
			} else {
				chprintf(chp, "Error: no value\r\n");
			}
		} else if (strcmp(argv[0], "HeelCorrection") == 0) {
			if (strlen(argv[1]) != 0) {
				calib_val = atof(argv[1]);
				//chprintf(chp, "Writing new calib number: %f\r\n", calib_val);
				eeprom_write(EEPROM_MATH_HEEL_CORRECTION, (uint8_t*) &calib_val,
						4);
				chprintf(chp, "Saved HeelCorrection value: %f\r\n", calib_val);
				paramSD.HeelCorrection = calib_val;
				microsd_update_calibfile();
			} else {
				chprintf(chp, "Error: no value\r\n");
			}
		} else if (strcmp(argv[0], "MagneticDeclanation") == 0) {
			if (strlen(argv[1]) != 0) {
				calib_val = atof(argv[1]);
				//chprintf(chp, "Writing new calib number: %f\r\n", calib_val);
				eeprom_write(EEPROM_MATH_DECLANATION_CORRECTION,
						(uint8_t*) &calib_val, 4);
				chprintf(chp, "Saved MagneticDeclanation value: %f\r\n",
						calib_val);
				paramSD.MagneticDeclanation = calib_val;
				microsd_update_calibfile();
			} else {
				chprintf(chp, "Error: no value\r\n");
			}
		} else if (strcmp(argv[0], "PitchCorrection") == 0) {
			if (strlen(argv[1]) != 0) {
				calib_val = atof(argv[1]);
				//chprintf(chp, "Writing new calib number: %f\r\n", calib_val);
				eeprom_write(EEPROM_MATH_PITCH_CORRECTION,
						(uint8_t*) &calib_val, 4);
				chprintf(chp, "Saved PitchCorrection value: %f\r\n", calib_val);
				paramSD.PitchCorrection = calib_val;
				microsd_update_calibfile();
			} else {
				chprintf(chp, "Error: no value\r\n");
			}
		} else if (strcmp(argv[0], "RudderCorrection") == 0) {
			if (strlen(argv[1]) != 0) {
				calib_val = atof(argv[1]);
				//chprintf(chp, "Writing new calib number: %f\r\n", calib_val);
				eeprom_write(EEPROM_MATH_RUDDER_CORRECTION,
						(uint8_t*) &calib_val, 4);

				chprintf(chp, "Saved RudderCorrection value: %f\r\n",
						calib_val);
				paramSD.RudderCorrection = calib_val;
				microsd_update_calibfile();
			} else {
				chprintf(chp, "Error: no value\r\n");
			}
		} else if (strcmp(argv[0], "WindCorrection") == 0) {
			if (strlen(argv[1]) != 0) {
				calib_val = atof(argv[1]);
				//chprintf(chp, "Writing new calib number: %f\r\n", calib_val);
				eeprom_write(EEPROM_MATH_WIND_CORRECTION, (uint8_t*) &calib_val,
						4);

				chprintf(chp, "Saved WindCorrection value: %f\r\n", calib_val);
				paramSD.WindCorrection = calib_val;
				microsd_update_calibfile();
			} else {
				chprintf(chp, "Error: no value\r\n");
			}
		} else if (strcmp(argv[0], "WindowSize1") == 0) {
			if (strlen(argv[1]) != 0) {
				calib_val_i = atoi(argv[1]);
				if (calib_val_i <= FILTER_BUFFER_SIZE) {
					//chprintf(chp, "Writing new calib number: %f\r\n", calib_val);
					eeprom_write(EEPROM_MATH_WINSIZE1_CORRECTION,
							(uint8_t*) &calib_val_i, 1);
					chprintf(chp, "Saved WindowSize1 value: %d\r\n",
							calib_val_i);
					paramSD.WindowSize1 = calib_val_i;
					microsd_update_calibfile();

				} else {
					chprintf(chp,
							"Saving WindowSize1 error: value is greater than FILTER_BUFFER_SIZE\r\n");
				}
			} else {
				chprintf(chp, "Error: no value %f\r\n");
			}

		} else if (strcmp(argv[0], "WindowSize2") == 0) {
			if (strlen(argv[1]) != 0) {
				calib_val_i = atoi(argv[1]);
				//chprintf(chp, "Writing new calib number: %f\r\n", calib_val);
				if (calib_val_i <= FILTER_BUFFER_SIZE) {
					eeprom_write(EEPROM_MATH_WINSIZE2_CORRECTION,
							(uint8_t*) &calib_val_i, 1);
					chprintf(chp, "Saved WindowSize2 value: %d\r\n",
							calib_val_i);
					paramSD.WindowSize2 = calib_val_i;
					microsd_update_calibfile();

				} else {
					chprintf(chp,
							"Saving WindowSize2 error: value is greater than FILTER_BUFFER_SIZE\r\n");
				}
			} else {
				chprintf(chp, "Error: no value\r\n");
			}

		} else if (strcmp(argv[0], "WindowSize3") == 0) {
			if (strlen(argv[1]) != 0) {
				calib_val_i = atoi(argv[1]);

				if (calib_val_i <= FILTER_BUFFER_SIZE) {
					//chprintf(chp, "Writing new calib number: %f\r\n", calib_val);
					eeprom_write(EEPROM_MATH_WINSIZE3_CORRECTION,
							(uint8_t*) &calib_val_i, 1);
					chprintf(chp, "Saved WindowSize3 value: %d\r\n",
							calib_val_i);
					paramSD.WindowSize3 = calib_val_i;
					microsd_update_calibfile();

				} else {
					chprintf(chp,
							"Saving WindowSize3 error: value is greater than FILTER_BUFFER_SIZE\r\n");
				}
			} else {
				chprintf(chp, "Error: no value\r\n");
			}

		}
#ifdef USE_BLE_MODULE
		else if (strcmp(argv[0], "rudder_left") == 0) {
			if (strlen(argv[1]) != 0) {
				calib_val = atof(argv[1]);

				if ((calib_val <= 180) && (calib_val >= -180)) {
					eeprom_write(EEPROM_RUDDER_CALIB_NATIVE_LEFT,
							(uint8_t*) &r_rudder->native, 4);
					chThdSleepMilliseconds(5);
					eeprom_write(EEPROM_RUDDER_CALIB_DEGREES_LEFT,
												(uint8_t*) &calib_val, 4);
					chprintf(chp, "Saved left rudder value: %f native and %f degrees\r\n",
							r_rudder->native, calib_val);
					r_rudder->min_native = r_rudder->native;
					r_rudder_dots->x1 = r_rudder->native;
					r_rudder_dots->y1 = calib_val;
									r_rudder->min_degrees = calib_val;
					calculate_polynom_coefs(r_rudder_dots, r_rudder_coefs);
					microsd_update_calibfile();

				} else {
					chprintf(chp,
							"Saving left rudder value error: value not valid\r\n");
				}
			}

		} else if (strcmp(argv[0], "rudder_center") == 0) {
			if (strlen(argv[1]) != 0) {
				calib_val = atof(argv[1]);

				if ((calib_val <= 180) && (calib_val >= -180)) {
					eeprom_write(EEPROM_RUDDER_CALIB_NATIVE_CENTER,
							(uint8_t*) &r_rudder->native, 4);
					chThdSleepMilliseconds(5);
					eeprom_write(EEPROM_RUDDER_CALIB_DEGREES_CENTER,
												(uint8_t*) &calib_val, 4);
					chprintf(chp, "Saved central rudder value: %f native and %f degrees\r\n",
							r_rudder->native, calib_val);
					r_rudder_dots->x2 = r_rudder->native;
					r_rudder_dots->y2 = calib_val;
					calculate_polynom_coefs(r_rudder_dots, r_rudder_coefs);
					microsd_update_calibfile();

				} else {
					chprintf(chp,
							"Saving center rudder value error: value not valid\r\n");
				}
			}

		}else if (strcmp(argv[0], "rudder_right") == 0) {
			if (strlen(argv[1]) != 0) {
				calib_val = atof(argv[1]);

				if ((calib_val <= 180) && (calib_val >= -180)) {
					eeprom_write(EEPROM_RUDDER_CALIB_NATIVE_RIGHT,
							(uint8_t*) &r_rudder->native, 4);
					chThdSleepMilliseconds(5);
					eeprom_write(EEPROM_RUDDER_CALIB_DEGREES_RIGHT,
												(uint8_t*) &calib_val, 4);
					chprintf(chp, "Saved right rudder value: %f native and %f degrees\r\n",
							r_rudder->native, calib_val);
					r_rudder->max_native = r_rudder->native;
					r_rudder_dots->x1 = r_rudder->native;
					r_rudder_dots->y1 = calib_val;
									r_rudder->min_degrees = calib_val;
					calculate_polynom_coefs(r_rudder_dots, r_rudder_coefs);
					microsd_update_calibfile();

				} else {
					chprintf(chp,
							"Saving right rudder value error: value not valid\r\n");
				}
			}

		}
#endif

	}
}
#endif
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
	float deg;
	uint8_t temp;
	if (output->type != OUTPUT_SERVICE) {
		return;
	}
	if (argc != 0) {
		if (strcmp(argv[0], "help") == 0) {
			chprintf(chp, "Usage: - rudder calibrate|left|middle|right\n\r");
		} else if (strcmp(argv[0], "calibrate") == 0) {
			chprintf(chp, "Starting rudder calibration procedure\r\n");
			output->type = OUTPUT_RUDDER_CALIB;
		} else if (strcmp(argv[0], "left") == 0) {
			if (strlen(argv[1]) != 0) {
				deg = atof(argv[1]);
				eeprom_write(EEPROM_RUDDER_CALIB_NATIVE_LEFT,
						(uint8_t*) &rudder->native, 4);

				chThdSleepMilliseconds(5);
				eeprom_write(EEPROM_RUDDER_CALIB_DEGREES_LEFT, (uint8_t*) &deg,
						4);
				chThdSleepMilliseconds(5);
				temp = 1;
				eeprom_read(EEPROM_RUDDER_CALIB_FLAG_ADDR, &temp, 1);
				rudder->min_native = rudder->native;
				dots->x1 = rudder->native;
				dots->y1 = deg;
				rudder->min_degrees = deg;
				adc_update_rudder_struct(rudder);
				chprintf(chp, "Saved left position\r\n");
			} else {
				chprintf(chp, "Error: invalid value\r\n");
			}
		} else if (strcmp(argv[0], "center") == 0) {
			if (strlen(argv[1]) != 0) {
				deg = atof(argv[1]);
				eeprom_write(EEPROM_RUDDER_CALIB_NATIVE_CENTER,
						(uint8_t*) &rudder->native, 4);
				chThdSleepMilliseconds(5);
				eeprom_write(EEPROM_RUDDER_CALIB_DEGREES_CENTER,
						(uint8_t*) &deg, 4);
				chThdSleepMilliseconds(5);
				temp = 1;
				eeprom_read(EEPROM_RUDDER_CALIB_FLAG_ADDR, &temp, 1);
				dots->x2 = rudder->native;
				dots->y2 = deg;
				adc_update_rudder_struct(rudder);
				chprintf(chp, "Saved center position\r\n");
			} else {
				chprintf(chp, "Error: invalid value\r\n");
			}
		} else if (strcmp(argv[0], "right") == 0) {
			if (strlen(argv[1]) != 0) {
				deg = atof(argv[1]);
				eeprom_write(EEPROM_RUDDER_CALIB_NATIVE_RIGHT,
						(uint8_t*) &rudder->native, 4);

				chThdSleepMilliseconds(5);
				eeprom_write(EEPROM_RUDDER_CALIB_DEGREES_RIGHT, (uint8_t*) &deg,
						4);
				chThdSleepMilliseconds(5);
				temp = 1;
				eeprom_read(EEPROM_RUDDER_CALIB_FLAG_ADDR, &temp, 1);
				rudder->max_native = rudder->native;
				dots->x3 = rudder->native;
				dots->y3 = deg;
				rudder->max_degrees = deg;
				adc_update_rudder_struct(rudder);
				chprintf(chp, "Saved right position\r\n");
			} else {
				chprintf(chp, "Error: invalid value\r\n");
			}
		} else if (strcmp(argv[0], "get") == 0) {
			eeprom_read(EEPROM_RUDDER_CALIB_DEGREES_LEFT, (uint8_t*) &deg, 4);
			chprintf(chp, "Rudder left deg: %f\r\n", deg);
			eeprom_read(EEPROM_RUDDER_CALIB_NATIVE_LEFT,
					(uint8_t*) &deg, 4);
			chprintf(chp, "Rudder left native: %f\r\n", deg);

			eeprom_read(EEPROM_RUDDER_CALIB_NATIVE_CENTER,
					(uint8_t*) &deg, 4);
			chprintf(chp, "Rudder center native: %f\r\n", deg);
			eeprom_read(EEPROM_RUDDER_CALIB_DEGREES_CENTER, (uint8_t*) &deg, 4);
			chprintf(chp, "Rudder center native: %f\r\n", deg);
			eeprom_read(EEPROM_RUDDER_CALIB_NATIVE_RIGHT,
					(uint8_t*) &deg, 4);
			chprintf(chp, "Rudder right native: %f\r\n", deg);
			eeprom_read(EEPROM_RUDDER_CALIB_DEGREES_RIGHT, (uint8_t*) &deg, 4);
			chprintf(chp, "Rudder right native: %f\r\n", deg);
			chprintf(chp, "Dots x1 x2 x3: %f %f %f\r\n", dots->x1, dots->x2, dots->x3);
			chprintf(chp, "Dots y1 y2 y3: %f %f %f\r\n", dots->y1, dots->y2, dots->y3);
			chprintf(chp, "Polynom a b c: %f %f %f\r\n", coefs->a, coefs->b, coefs->c);

		} else if (strcmp(argv[0], "reset") == 0) {
			deg = 2000.0;
			eeprom_write(EEPROM_RUDDER_CALIB_NATIVE_CENTER,
					(uint8_t*) &deg, 4);
			chThdSleepMilliseconds(5);
			deg = 0.0;
			eeprom_write(EEPROM_RUDDER_CALIB_DEGREES_CENTER, (uint8_t*) &deg,
					4);
			chThdSleepMilliseconds(5);
			deg = 4000.0;
			eeprom_write(EEPROM_RUDDER_CALIB_NATIVE_RIGHT,
					(uint8_t*) &deg, 4);
			chThdSleepMilliseconds(5);
			deg = 90.0;
			eeprom_write(EEPROM_RUDDER_CALIB_DEGREES_RIGHT, (uint8_t*) &deg, 4);
			chThdSleepMilliseconds(5);
			deg = 100.0;
			eeprom_write(EEPROM_RUDDER_CALIB_NATIVE_LEFT,
					(uint8_t*) &deg, 4);
			chThdSleepMilliseconds(5);
			deg = -90.0;
			eeprom_write(EEPROM_RUDDER_CALIB_DEGREES_LEFT, (uint8_t*) &deg,
					4);
			chThdSleepMilliseconds(5);
			adc_update_rudder_struct(rudder);
			chprintf(chp, "Rudder calibration set to default\r\n");
		}
	}
}

#endif

#ifdef SD_SENSOR_BOX_LAG
void cmd_lag(BaseSequentialStream* chp, int argc, char* argv[]) {
	float calib_val;
	if (output->type != OUTPUT_SERVICE) {
		return;
	}
	if (argc != 0) {
		if (strcmp(argv[0], "help") == 0) {
			chprintf(chp,
					"Usage: - lag write <float> - float number which\t\n\tmultiply hz measures.\n\r");
		} else if (strcmp(argv[0], "write") == 0) {
			if (strlen(argv[1]) != 0){
				calib_val = atof(argv[1]);
				chprintf(chp, "Writing new calib number: %f\r\n", calib_val);
				eeprom_write(EEPROM_LAG_CALIB_NUMBER, (uint8_t*)&calib_val, 4);
				lag->calib_num = calib_val;
			}
		} else if (strcmp(argv[0], "calibrate") == 0){
			eeprom_read(EEPROM_LAG_CALIB_NUMBER, (uint8_t*)&calib_val, 4);
			chprintf(chp, "Readed calib number: %f\r\n", calib_val);

		}
	}
}
#endif


#ifdef USE_LAG_MODULE
#endif
