#include "calibration.h"

#include "config.h"
#include <hal.h>
#include "shellconf.h"
#include "math.h"
#include <shell.h>
#include <string.h>
#include <stdlib.h>
#include "microsd.h"
#include "xbee.h"
#include "eeprom.h"
#include "lag.h"
#include "bmx160_i2c.h"
#include "sailDataMath.h"

#ifdef USE_ADC_MODULE
#include "adc.h"

#endif

#ifdef USE_BLE_MODULE
extern lag_t *r_lag;
extern rudder_t *r_rudder;
#endif

#ifdef USE_TENSO_MODULE
#include "tenso.h"
#endif

#if defined(SD_SENSOR_BOX) || defined(USE_RUDDER_MODULE)
#include "adc.h"
extern dots_t *r_rudder_dots;
extern coefs_t *r_rudder_coefs;
#endif

extern bmx160_t bmx160;
extern struct ch_semaphore usart1_semaph;
extern xbee_remote_dev_t remote_dev[NUM_OF_SPORTSMAN_DEVICES + NUM_OF_BOUY_DEVICES];

calib_parameters_t calibrations;

void cmd_load_math_cal(BaseSequentialStream* chp, int argc, char* argv[]) {
	float calib_val;
	int16_t calib_val_i;
	if (argc != 0) {
		if (strcmp(argv[0], "help") == 0) {
			chprintf(chp,
					"Usage: - load_math_calib sportsman <NUM> CompassCorrection|HSPCorrection|HeelCorrection|MagneticDeclanation|PitchCorrection|RudderCorrection|WindCorrection|WindowSize1|WindowSize2|WindowSize3 <float>\n\r");
		} else {
			calib_parse_shell_command(chp, &argv[1]);
		}

	}
}

void cmd_get_math_cal(BaseSequentialStream* chp, int argc, char* argv[]) {

	if (argc != 0) {
		if (strcmp(argv[0], "help") == 0) {
			chprintf(chp, "Usage: - get_math_calib sportsman <NUM> \n\r");
		} else {
			uint8_t dev_num = atoi(argv[1]);
			xbee_get_calib_request_to_remote_dev(dev_num);
			chThdSleepMilliseconds(500);
			calib_print_calib_to_shell(chp, dev_num);
		}
	}
}

void calib_print_calib_to_shell(BaseSequentialStream* chp, uint8_t dev_num) {
	calib_parameters_t *cal;
	if (dev_num == 0) {
		cal = &calibrations;
	} else {
		cal = &remote_dev[dev_num].calibrations;
	}

	chSemWait(&usart1_semaph);
	chprintf(chp,
			"\r\n{\"msg_type\":\"math_calib_data\",\r\n\t\t\"boat_%d\":{\r\n\t\t\t",
			dev_num);
	chprintf(chp, "\"CompassCorrection\":%f,\r\n\t\t\t",
			cal->CompassCorrection);
	chprintf(chp, "\"HSPCorrection\":%f,\r\n\t\t\t", cal->HSPCorrection);
	chprintf(chp, "\"HeelCorrection\":%f,\r\n\t\t\t", cal->HeelCorrection);
	chprintf(chp, "\"MagneticDeclanation\":%f,\r\n\t\t\t",
			cal->MagneticDeclanation);
	chprintf(chp, "\"PitchCorrection\":%f,\r\n\t\t\t", cal->PitchCorrection);
	chprintf(chp, "\"RudderCorrection\":%f,\r\n\t\t\t", cal->RudderCorrection);
	chprintf(chp, "\"WindCorrection\":%f,\r\n\t\t\t", cal->WindCorrection);
	chprintf(chp, "\"WindowSize1\":%d,\r\n\t\t\t", cal->WindowSize1);
	chprintf(chp, "\"WindowSize2\":%d,\r\n\t\t\t", cal->WindowSize2);
	chprintf(chp, "\"WindowSize3\":%d,\r\n\t\t\t", cal->WindowSize3);
	chprintf(chp, "\"RudderLeftNative\":%d,\r\n\t\t\t",
			(uint16_t) cal->rudder_calib.min_native);
	chprintf(chp, "\"RudderLeftDegrees\":%f,\r\n\t\t\t",
			cal->rudder_calib.min_degrees);
	chprintf(chp, "\"RudderCenterNative\":%d,\r\n\t\t\t",
			(uint16_t) cal->rudder_calib.center_native);
	chprintf(chp, "\"RudderCenterDegrees\":%f,\r\n\t\t\t",
			cal->rudder_calib.center_degrees);
	chprintf(chp, "\"RudderRightNative\":%d,\r\n\t\t\t",
			(uint16_t) cal->rudder_calib.max_native);
	chprintf(chp, "\"RudderRightDegrees\":%f,\r\n\t\t\t",
			cal->rudder_calib.max_degrees);
	chprintf(chp, "\"CompassRawOffsetX\":%f,\r\n\t\t\t", bmx160.mag_offset.x);
	chprintf(chp, "\"CompassRawOffsetY\":%f,\r\n\t\t\t", bmx160.mag_offset.y);
	chprintf(chp, "\"CompassRawOffsetZ\":%f\r\n\t\t\t", bmx160.mag_offset.z);
	chprintf(chp, "}\r\n\t}\r\n");
	chSemSignal(&usart1_semaph);

}

void calib_init_params(void) {
	EEPROM_READ(MATH_MEMORY.MATH_COMPASS_CORRECTION, (uint8_t*)&calibrations.CompassCorrection);
	EEPROM_READ(MATH_MEMORY.MATH_HSP_CORRECTION, (uint8_t*)&calibrations.HSPCorrection);
	EEPROM_READ(MATH_MEMORY.MATH_HEEL_CORRECTION, (uint8_t*)&calibrations.HeelCorrection);
	EEPROM_READ(MATH_MEMORY.MATH_DECLANATION_CORRECTION, (uint8_t*)&calibrations.MagneticDeclanation);
	EEPROM_READ(MATH_MEMORY.MATH_PITCH_CORRECTION, (uint8_t*)&calibrations.PitchCorrection);
	EEPROM_READ(MATH_MEMORY.MATH_RUDDER_CORRECTION, (uint8_t*)&calibrations.RudderCorrection);
	EEPROM_READ(MATH_MEMORY.MATH_WIND_CORRECTION, (uint8_t*)&calibrations.WindCorrection);
	EEPROM_READ(MATH_MEMORY.MATH_WINSIZE1_CORRECTION, (uint8_t*)&calibrations.WindowSize1);
	EEPROM_READ(MATH_MEMORY.MATH_WINSIZE2_CORRECTION, (uint8_t*)&calibrations.WindowSize2);
	EEPROM_READ(MATH_MEMORY.MATH_WINSIZE3_CORRECTION, (uint8_t*)&calibrations.WindowSize3);
}

int8_t calib_update_compass_correction(uint8_t dev_num, float calib_val)
{
	chprintf(SHELL_IFACE, "input compass corr: %f, devnum %d", calib_val, dev_num);
	if (dev_num == 0) {
	EEPROM_WRITE(MATH_MEMORY.MATH_COMPASS_CORRECTION, (uint8_t*) &calib_val);
	//chprintf(chp, "input compass corr: %f", calib_val);
	calibrations.CompassCorrection = calib_val;
	microsd_update_calibfile();
	} else {
		xbee_write_calibration_to_remote_dev(dev_num, RF_CALIB_COMPASS, calib_val);
	}
	return 0;
}

int8_t calib_update_hsp_correction(uint8_t dev_num, float calib_val)
{
	if (dev_num == 0) {
	EEPROM_WRITE(MATH_MEMORY.MATH_HSP_CORRECTION, (uint8_t*) &calib_val);
	calibrations.CompassCorrection = calib_val;
	microsd_update_calibfile();
	}else {
		xbee_write_calibration_to_remote_dev(dev_num, RF_CALIB_HSP, calib_val);
	}
	return 0;
}

int8_t calib_update_heel_correction(uint8_t dev_num, float calib_val)
{
	if (dev_num == 0) {
	EEPROM_WRITE(MATH_MEMORY.MATH_HEEL_CORRECTION, (uint8_t*) &calib_val);
	calibrations.HeelCorrection = calib_val;
	microsd_update_calibfile();
	} else {
		xbee_write_calibration_to_remote_dev(dev_num, RF_CALIB_HEEL, calib_val);
	}
	return 0;
}

int8_t calib_update_magnetic_declanation(uint8_t dev_num, float calib_val)
{
	if (dev_num == 0) {
	EEPROM_WRITE(MATH_MEMORY.MATH_DECLANATION_CORRECTION, (uint8_t*) &calib_val);
	calibrations.MagneticDeclanation = calib_val;
	microsd_update_calibfile();
	} else {
		xbee_write_calibration_to_remote_dev(dev_num, RF_CALIB_MAGN, calib_val);
	}
	return 0;
}

int8_t calib_update_pitch_correction(uint8_t dev_num, float calib_val)
{
	if (dev_num == 0) {
	EEPROM_WRITE(MATH_MEMORY.MATH_PITCH_CORRECTION, (uint8_t*) &calib_val);
	calibrations.PitchCorrection = calib_val;
	microsd_update_calibfile();
	} else {
		xbee_write_calibration_to_remote_dev(dev_num, RF_CALIB_PITCH, calib_val);
	}
	return 0;
}

int8_t calib_update_rudder_correction(uint8_t dev_num, float calib_val)
{
	if (dev_num == 0) {
	EEPROM_WRITE(MATH_MEMORY.MATH_RUDDER_CORRECTION, (uint8_t*) &calib_val);
	calibrations.RudderCorrection = calib_val;
	microsd_update_calibfile();
	} else {
		xbee_write_calibration_to_remote_dev(dev_num, RF_CALIB_RUDDER, calib_val);
	}
	return 0;
}

int8_t calib_update_wind_correction(uint8_t dev_num, float calib_val)
{
	if (dev_num == 0) {
	EEPROM_WRITE(MATH_MEMORY.MATH_WIND_CORRECTION, (uint8_t*) &calib_val);
	calibrations.WindCorrection = calib_val;
	microsd_update_calibfile();
	} else {
		xbee_write_calibration_to_remote_dev(dev_num, RF_CALIB_WIND, calib_val);
	}
	return 0;
}

int8_t calib_update_window_size_1(uint8_t dev_num, uint8_t calib_val_i)
{
	if (dev_num == 0) {
	EEPROM_WRITE(MATH_MEMORY.MATH_WINSIZE1_CORRECTION, (uint8_t*) &calib_val_i);
	calibrations.WindowSize1 = calib_val_i;
	microsd_update_calibfile();
	} else {
		xbee_write_calibration_to_remote_dev(dev_num, RF_CALIB_WINDOW_1, (float) calib_val_i);
	}
	return 0;
}

int8_t calib_update_window_size_2(uint8_t dev_num, uint8_t calib_val_i)
{
	if (dev_num == 0) {
	EEPROM_WRITE(MATH_MEMORY.MATH_WINSIZE3_CORRECTION, (uint8_t*) &calib_val_i);
	calibrations.WindowSize1 = calib_val_i;
	microsd_update_calibfile();
	} else {
		xbee_write_calibration_to_remote_dev(dev_num, RF_CALIB_WINDOW_2, (float) calib_val_i);
	}
	return 0;
}
int8_t calib_update_window_size_3(uint8_t dev_num, uint8_t calib_val_i)
{
	if (dev_num == 0) {
	EEPROM_WRITE(MATH_MEMORY.MATH_WINSIZE3_CORRECTION, (uint8_t*) &calib_val_i);
	calibrations.WindowSize1 = calib_val_i;
	microsd_update_calibfile();
	} else {
		xbee_write_calibration_to_remote_dev(dev_num, RF_CALIB_WINDOW_3, (float) calib_val_i);
	}
	return 0;
}

int8_t calib_update_rudder_left(uint8_t dev_num, float calib_val)
{
	if (dev_num == 0) {
	EEPROM_WRITE(RUDDER_MEMORY.RUDDER_CALIB_NATIVE_LEFT, (uint8_t*) &r_rudder->native);
	EEPROM_WRITE(RUDDER_MEMORY.RUDDER_CALIB_DEGREES_LEFT, (uint8_t*) &calib_val);
	calibrations.rudder_calib.min_native = r_rudder->native;
	calibrations.rudder_dots.x1 = r_rudder->native;
	calibrations.rudder_dots.y1 = calib_val;
	calibrations.rudder_calib.min_degrees = calib_val;
	calculate_polynom_coefs(&calibrations.rudder_dots, &calibrations.rudder_coefs);
	microsd_update_calibfile();
	} else {
		xbee_write_calibration_to_remote_dev(dev_num, RF_CALIB_RUDDER_LEFT, calib_val);
	}
	return 0;
}

int8_t calib_update_rudder_center(uint8_t dev_num, float calib_val)
{
	if (dev_num == 0) {
	EEPROM_WRITE(RUDDER_MEMORY.RUDDER_CALIB_NATIVE_CENTER, (uint8_t*) &r_rudder->native);
	EEPROM_WRITE(RUDDER_MEMORY.RUDDER_CALIB_DEGREES_CENTER, (uint8_t*) &calib_val);
	calibrations.rudder_calib.center_native = r_rudder->native;
	calibrations.rudder_dots.x2 = r_rudder->native;
	calibrations.rudder_dots.y2 = calib_val;
	calibrations.rudder_calib.center_degrees = calib_val;
	calculate_polynom_coefs(&calibrations.rudder_dots, &calibrations.rudder_coefs);
	microsd_update_calibfile();
	} else {
		xbee_write_calibration_to_remote_dev(dev_num, RF_CALIB_RUDDER_CENTER, calib_val);
	}
	return 0;
}

int8_t calib_update_rudder_right(uint8_t dev_num, float calib_val)
{
	if (dev_num == 0) {
	EEPROM_WRITE(RUDDER_MEMORY.RUDDER_CALIB_NATIVE_RIGHT, (uint8_t*) &r_rudder->native);
	EEPROM_WRITE(RUDDER_MEMORY.RUDDER_CALIB_DEGREES_RIGHT, (uint8_t*) &calib_val);
	calibrations.rudder_calib.max_native = r_rudder->native;
	calibrations.rudder_dots.x3 = r_rudder->native;
	calibrations.rudder_dots.y3 = calib_val;
	calibrations.rudder_calib.max_degrees = calib_val;
	calculate_polynom_coefs(&calibrations.rudder_dots, &calibrations.rudder_coefs);
	microsd_update_calibfile();
	} else {
		xbee_write_calibration_to_remote_dev(dev_num, RF_CALIB_RUDDER_RIGHT, calib_val);
	}
	return 0;
}

void calib_parse_shell_command(BaseSequentialStream* chp, char* argv[]) {
	float calib_val;
	uint32_t calib_val_i;
	uint8_t dev_num = atoi(argv[0]);
	if (strcmp(argv[1], "CompassCorrection") == 0) {
		if (strlen(argv[2]) != 0) {
			calib_val = atof(argv[2]);
			calib_update_compass_correction(dev_num, calib_val);
			chprintf(chp, "Saved CompassCorrection value: %f\r\n", calib_val);
		} else {
			chprintf(chp, "Error: no value\r\n");
		}
	} else if (strcmp(argv[1], "HSPCorrection") == 0) {
		if (strlen(argv[2]) != 0) {
			calib_val = atof(argv[2]);
			calib_update_hsp_correction(dev_num, calib_val);
			chprintf(chp, "Saved HSPCorrection value: %f\r\n", calib_val);

		} else {
			chprintf(chp, "Error: no value\r\n");
		}
	} else if (strcmp(argv[1], "HeelCorrection") == 0) {
		if (strlen(argv[2]) != 0) {
			calib_val = atof(argv[2]);
			calib_update_heel_correction(dev_num, calib_val);

			chprintf(chp, "Saved HeelCorrection value: %f\r\n", calib_val);

		} else {
			chprintf(chp, "Error: no value\r\n");
		}
	} else if (strcmp(argv[1], "MagneticDeclanation") == 0) {
		if (strlen(argv[2]) != 0) {
			calib_val = atof(argv[2]);
			calib_update_magnetic_declanation(dev_num, calib_val);
			chprintf(chp, "Saved MagneticDeclanation value: %f\r\n", calib_val);
		} else {
			chprintf(chp, "Error: no value\r\n");
		}
	} else if (strcmp(argv[1], "PitchCorrection") == 0) {
		if (strlen(argv[2]) != 0) {
			calib_val = atof(argv[2]);
			calib_update_pitch_correction(dev_num, calib_val);
			chprintf(chp, "Saved PitchCorrection value: %f\r\n", calib_val);
		} else {
			chprintf(chp, "Error: no value\r\n");
		}
	} else if (strcmp(argv[1], "RudderCorrection") == 0) {
		if (strlen(argv[2]) != 0) {
			calib_val = atof(argv[2]);
			calib_update_rudder_correction(dev_num, calib_val);
			chprintf(chp, "Saved RudderCorrection value: %f\r\n", calib_val);

		} else {
			chprintf(chp, "Error: no value\r\n");
		}
	} else if (strcmp(argv[1], "WindCorrection") == 0) {
		if (strlen(argv[2]) != 0) {
			calib_val = atof(argv[2]);
			calib_update_wind_correction(dev_num, calib_val);
			chprintf(chp, "Saved WindCorrection value: %f\r\n", calib_val);

		} else {
			chprintf(chp, "Error: no value\r\n");
		}
	} else if (strcmp(argv[1], "WindowSize1") == 0) {
		if (strlen(argv[2]) != 0) {
			calib_val_i = atoi(argv[2]);
			if (calib_val_i <= FILTER_BUFFER_SIZE) {
				calib_update_window_size_1(dev_num, calib_val);
				chprintf(chp, "Saved WindowSize1 value: %d\r\n", calib_val_i);
			} else {
				chprintf(chp,
						"Saving WindowSize1 error: value is greater than FILTER_BUFFER_SIZE\r\n");
			}
		} else {
			chprintf(chp, "Error: no value %f\r\n");
		}

	} else if (strcmp(argv[1], "WindowSize2") == 0) {
		if (strlen(argv[2]) != 0) {
			calib_val_i = atoi(argv[2]);
			if (calib_val_i <= FILTER_BUFFER_SIZE) {
				calib_update_window_size_2(dev_num, calib_val);
				chprintf(chp, "Saved WindowSize2 value: %d\r\n", calib_val_i);
			} else {
				chprintf(chp,
						"Saving WindowSize2 error: value is greater than FILTER_BUFFER_SIZE\r\n");
			}
		} else {
			chprintf(chp, "Error: no value\r\n");
		}

	} else if (strcmp(argv[1], "WindowSize3") == 0) {
		if (strlen(argv[2]) != 0) {
			calib_val_i = atoi(argv[2]);
			if (calib_val_i <= FILTER_BUFFER_SIZE) {
				calib_update_window_size_3(dev_num, calib_val);
				chprintf(chp, "Saved WindowSize3 value: %d\r\n", calib_val_i);
			} else {
				chprintf(chp,
						"Saving WindowSize3 error: value is greater than FILTER_BUFFER_SIZE\r\n");
			}
		} else {
			chprintf(chp, "Error: no value\r\n");
		}
		/*
		 } else if (strcmp(argv[1], "CompassRawOffsetX") == 0) {
		 if (strlen(argv[2]) != 0) {
		 calib_val = atof(argv[2]);

		 chprintf(chp, "Saved CompassRawOffsetX value: %f\r\n", calib_val);
		 bmx160.mag_offset.x = calib_val;
		 } else {
		 chprintf(chp, "Error: no value\r\n");
		 }
		 } else if (strcmp(argv[1], "CompassRawOffsetY") == 0) {
		 if (strlen(argv[2]) != 0) {
		 calib_val = atof(argv[2]);
		 //chprintf(chp, "Writing new calib number: %f\r\n", calib_val);
		 EEPROM_WRITE(MAGN_MEMORY.MAGN_Y_OFFSET, (uint8_t*) &calib_val);

		 chprintf(chp, "Saved CompassRawOffsetY value: %f\r\n", calib_val);
		 bmx160.mag_offset.y = calib_val;
		 } else {
		 chprintf(chp, "Error: no value\r\n");
		 }
		 } else if (strcmp(argv[1], "CompassRawOffsetZ") == 0) {
		 if (strlen(argv[2]) != 0) {
		 calib_val = atof(argv[2]);
		 EEPROM_WRITE(MAGN_MEMORY.MAGN_Z_OFFSET, (uint8_t*) &calib_val);

		 chprintf(chp, "Saved CompassRawOffsetZ value: %f\r\n", calib_val);
		 bmx160.mag_offset.z = calib_val;
		 } else {
		 chprintf(chp, "Error: no value\r\n");
		 }
		 }
		 */
	} else if (strcmp(argv[1], "rudder_left") == 0) {
		if (strlen(argv[2]) != 0) {
			calib_val = atof(argv[2]);

			if ((calib_val <= 180) && (calib_val >= -180)) {
				calib_update_rudder_left(dev_num, calib_val);
				chprintf(chp,
						"Saved left rudder value: %f native and %f degrees\r\n",
						calibrations.rudder_calib.native, calib_val);
			} else {
				chprintf(chp,
						"Saving left rudder value error: value not valid\r\n");
			}
		}

	} else if (strcmp(argv[1], "rudder_center") == 0) {
		if (strlen(argv[2]) != 0) {
			calib_val = atof(argv[2]);
			if ((calib_val <= 180) && (calib_val >= -180)) {
				calib_update_rudder_center(dev_num, calib_val);
				chprintf(chp,
						"Saved central rudder value: %f native and %f degrees\r\n",
						calibrations.rudder_calib.native, calib_val);
			} else {
				chprintf(chp,
						"Saving center rudder value error: value not valid\r\n");
			}
		}

	} else if (strcmp(argv[1], "rudder_right") == 0) {
		if (strlen(argv[2]) != 0) {
			calib_val = atof(argv[2]);

			if ((calib_val <= 180) && (calib_val >= -180)) {
				calib_update_rudder_right(dev_num, calib_val);
				chprintf(chp,
						"Saved right rudder value: %f native and %f degrees\r\n",
						calibrations.rudder_calib.native, calib_val);
			} else {
				chprintf(chp,
						"Saving right rudder value error: value not valid\r\n");
			}
		}

	}
}
