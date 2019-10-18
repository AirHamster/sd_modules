#include "config.h"
#include "sd_shell_cmds.h"
#include <hal.h>
#include "shellconf.h"
#include "math.h"
#include <shell.h>
#include <string.h>
#include <stdlib.h>
#ifdef USE_MICROSD_MODULE
#include "microsd.h"
extern thread_reference_t microsd_trp;
#endif
#ifdef USE_XBEE_868_MODULE
#include "xbee.h"
#endif
#ifdef USE_UBLOX_GPS_MODULE
#include "neo-m8.h"
#include "neo_ubx.h"
extern ubx_nav_pvt_t *pvt_box;
extern ubx_nav_odo_t *odo_box;
#endif
#ifdef USE_BNO055_MODULE
#include "bno055_i2c.h"
#include "bno055.h"
extern bno055_t *bno055;
#endif
#ifdef USE_WINDSENSOR_MODULE
#include "windsensor.h"
extern windsensor_t *wind;
#endif
#ifdef USE_SERVICE_MODE
#include "service_mode.h"
#endif
#ifdef USE_BLE_MODULE
#include "nina-b3.h"
#include "lag.h"
#include "adc.h"
extern ble_peer_t *peer;
#endif
#ifdef SD_SENSOR_BOX_RUDDER
extern ble_charac_t *ble_rudder;
extern rudder_t *rudder;
#endif
#ifdef SD_SENSOR_BOX_LAG
#include "lag.h"
extern ble_charac_t *ble_lag;
extern lag_t *lag;
#endif
#ifdef USE_ADC_MODULE
#include "adc.h"

#endif

#ifdef USE_HMC5883_MODULE
#include "hmc5883_i2c.h"
extern hmc5883_t *hmc5883;
#endif

#ifdef USE_HMC6343_MODULE
#include "hmc6343_i2c.h"
extern hmc6343_t *hmc6343;
#endif

extern lag_t *r_lag;
extern rudder_t *r_rudder;

#ifdef SD_MODULE_TRAINER
#ifdef USE_MATH_MODULE
#include "sailDataMath.h"
extern float lastFilterValues[10][FILTER_BUFFER_SIZE];
extern float windAngleTarget;
extern float lastSensorValues[SIZE_BUFFER_VALUES];
extern float hullSpeedTarget;
extern float velocityMadeGoodTarget;
#endif
#endif

extern struct ch_semaphore usart1_semaph;
extern ble_charac_t *thdg;
extern ble_charac_t *rdr;
extern ble_charac_t *twd;
extern ble_charac_t *tws;
extern ble_charac_t *twa;
extern ble_charac_t *bs;
extern ble_charac_t *twa_tg;
extern ble_charac_t *bs_tg;
extern ble_charac_t *hdg;
extern ble_charac_t *heel;
extern uint32_t __ram0_end__;
output_t *output;
char *complete_buffer[16];
char history_buffer[128];
const int history_size = 128;
static void send_json(void);
thread_reference_t output_trp = NULL;
static THD_WORKING_AREA(output_thread_wa, 1024*4);
static THD_FUNCTION(output_thread, arg);
static const ShellCommand commands[] = {
		{ "start", cmd_start },
		{ "c", cmd_c },
		{ "boot", cmd_boot },
		{ "reset", cmd_reset },
#ifdef USE_SERVICE_MODE
		{ "service", cmd_service },
#ifdef SD_MODULE_TRAINER
#ifdef USE_MATH_MODULE
		{ "load_math_calib", cmd_load_math_cal },
		{ "get_math_calib", cmd_get_math_cal },
#endif
#endif // SD_MODULE_TRAINER
#ifdef USE_BNO055_MODULE
		{ "gyro", cmd_gyro },
#endif //USE_BNO055_MODULE
#ifdef USE_MICROSD_MODULE
		{ "microsd", cmd_microsd },
		{ "tree", cmd_tree },
		{ "cat", cmd_cat },
		{ "mkfs", cmd_mkfs },
#endif //USE_MICROSD_MODULE
#ifdef USE_BLE_MODULE
		{ "ble", cmd_ble },
#endif //USE_BLE_MODULE
#ifdef SD_SENSOR_BOX_LAG
		{ "lag", cmd_lag },
#endif
#endif //USE_SERVICE_MODE

#ifdef USE_XBEE_868_MODULE
		{ "xbee", cmd_xbee },
#endif
#ifdef USE_MICROSD_MODULE
		{ "tree", cmd_tree },
		{ "mount", cmd_mount },
		{ "free", cmd_free },
		{ "open", cmd_open },
		{ "write", cmd_write },
#endif
#ifdef USE_RUDDER_MODULE
		{"rudder", cmd_rudder },
#endif
		{ NULL, NULL }
};

static const ShellConfig shell_cfg1 = { (BaseSequentialStream*) &SHELL_SD,
		commands, history_buffer, 32, complete_buffer };

thread_t *cmd_init(void) {
	return chThdCreateFromHeap(NULL, SHELL_WA_SIZE, "shell", NORMALPRIO + 4,
			shellThread, (void *) &shell_cfg1);
}

void start_json_module(void){
	chThdCreateStatic(output_thread_wa, sizeof(output_thread_wa), NORMALPRIO, output_thread, NULL);
}

/*
 * Thread that outputs debug data which is needed
 */

static THD_FUNCTION(output_thread, arg) {
	(void)arg;
	uint8_t i = 0;
	chRegSetThreadName("Data output");
	systime_t prev = chVTGetSystemTime(); // Current system time.

	while (true) {
		//wdgReset(&WDGD1);
		palToggleLine(LINE_GREEN_LED);
		chThdSleepMilliseconds(5);
#ifdef USE_BNO055_MODULE
		switch (output->type){
		case OUTPUT_NONE:
			break;
		case OUTPUT_TEST:
			send_json();
	//		if (i++ == 10) {
		//		nina_send_all(peer);
		//		i = 0;
		//	}
			break;
		case OUTPUT_SERVICE:
			break;
		case OUTPUT_ALL_CALIB:
			output_all_calib();
			break;
		case OUTPUT_BLE:
		//	if (i++ == 10){
			//nina_send_all(peer);
		//	i = 0;
		//	}
			break;
		default:
			break;
		}
#endif

#ifdef USE_BLE_MODULE
		switch (output->type){
				case OUTPUT_NONE:
					break;
				case OUTPUT_TEST:
					if (i++ == 10) {
						copy_to_ble();
						nina_send_all(peer);
						i = 0;
					}
					break;
				case OUTPUT_RUDDER_CALIB:
							adc_print_rudder_info(rudder);
							break;
				case OUTPUT_SERVICE:
					break;
				case OUTPUT_BLE:
		/*			if (i++ == 10){
					nina_send_all(peer);
					i = 0;
					}*/
					break;
				default:
					break;
				}
#endif
#ifdef SD_SENSOR_BOX_RUDDER
		switch (output->ble){
		case OUTPUT_NONE:
			break;
		case OUTPUT_RUDDER_CALIB:
			adc_print_rudder_info(rudder);
			break;
		case OUTPUT_RUDDER_BLE:
			send_rudder_over_ble(rudder);
		default:
			break;
		}
#endif
#ifdef SD_SENSOR_BOX_LAG
		switch (output->ble) {
		case OUTPUT_NONE:
			break;
		case OUTPUT_LAG_CALIB:
			//adc_print_rudder_info(lag);
			break;
		case OUTPUT_LAG_BLE:
			send_lag_over_ble(lag);
		default:
			break;
		}
#endif
		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(100));
	}
}

#ifdef USE_BNO055_MODULE
uint8_t output_all_calib(void){
	chSemWait(&usart1_semaph);
	chprintf(SHELL_IFACE, "\r\n{\"msg_type\":\"calib_data\",\r\n\t\t\"boat_1\":{\r\n\t\t\t");
	chprintf(SHELL_IFACE, "\"yaw\":%d,\r\n\t\t\t", (uint16_t)bno055->d_euler_hpr.h);
	chprintf(SHELL_IFACE, "\"pitch\":%f,\r\n\t\t\t", bno055->d_euler_hpr.p);
	chprintf(SHELL_IFACE, "\"roll\":%f,\r\n\t\t\t", bno055->d_euler_hpr.r);
	chprintf(SHELL_IFACE, "\"magn_cal\":%d,\r\n\t\t\t", bno055->calib_stat.magn);
	chprintf(SHELL_IFACE, "\"accel_cal\":%d,\r\n\t\t\t", bno055->calib_stat.accel);
	chprintf(SHELL_IFACE, "\"gyro_cal\":%d,\r\n\t\t\t", bno055->calib_stat.gyro);
	chprintf(SHELL_IFACE, "\"sys_cal\":%d,\r\n\t\t\t", bno055->calib_stat.system);
	chprintf(SHELL_IFACE, "}\r\n\t}");
	chSemSignal(&usart1_semaph);
}
#endif

/*
void send_data(uint8_t stream){
	uint8_t databuff[34];
	int32_t spdi = 0;
	double spd;
	double dlat, dlon;
	spd = (float)(pvt_box->gSpeed * 0.0036);
	spdi = (int32_t)(spd);
	tx_box->lat = pvt_box->lat;
	tx_box->lon = pvt_box->lon;
	tx_box->hour = pvt_box->hour;
	tx_box->min = pvt_box->min;
	tx_box->sec = pvt_box->sec;
	tx_box->dist = (uint16_t)odo_box->distance;
	tx_box->sat = pvt_box->numSV;
	tx_box->speed = spd;
	tx_box->headMot = pvt_box->headMot;
	tx_box->headVeh = pvt_box->headVeh;

		databuff[0] = RF_GPS_PACKET;
		databuff[1] = (uint8_t)(tx_box->lat >> 24);
		databuff[2] = (uint8_t)(tx_box->lat >> 16 );
		databuff[3] = (uint8_t)(tx_box->lat >> 8);
		databuff[4] = (uint8_t)(tx_box->lat);
		databuff[5] = (uint8_t)(tx_box->lon >> 24);
		databuff[6] = (uint8_t)(tx_box->lon >> 16);
		databuff[7] = (uint8_t)(tx_box->lon >> 8);
		databuff[8] = (uint8_t)(tx_box->lon);
		databuff[9] = tx_box->hour;
		databuff[10] = tx_box->min;
		databuff[11] = tx_box->sec;
		databuff[12] = tx_box->sat;
		databuff[13] = (uint8_t)(tx_box->dist >> 8);
		databuff[14] = (uint8_t)(tx_box->dist);

		memcpy(&databuff[15], &tx_box->speed, sizeof(tx_box->speed));

		databuff[19] = (uint8_t)(tx_box->yaw >> 8);
		databuff[20] = (uint8_t)(tx_box->yaw);

		memcpy(&databuff[21], &tx_box->pitch, sizeof(tx_box->pitch));

		memcpy(&databuff[25], &tx_box->roll, sizeof(tx_box->roll));
		databuff[29] = tx_box->bat;

		databuff[30] = (uint8_t)(tx_box->headMot >> 24);
		databuff[31] = (uint8_t)(tx_box->headMot >> 16);
		databuff[32] = (uint8_t)(tx_box->headMot >> 8);
		databuff[33] = (uint8_t)(tx_box->headMot);

	xbee_send_rf_message(xbee, databuff, 34);
}
*/
int32_t convert_to_ble_type(float value){
	int32_t val;
	int16_t cel;
	uint8_t drob;

	if(value < 0.0){
		value = value * -1;
		cel = (int16_t)(value);
		drob = (uint8_t)((value - (float)cel) * 100);
		//chprintf(SHELL_IFACE, "cel %x\r\n", cel);
		//	chprintf(SHELL_IFACE, "drob %x\r\n", drob);
		cel = cel * -1;
		val = cel << 8 | drob;
		val &= 0xFFFFFF;
	}else{
		cel = (int16_t)(value);
	drob = (uint8_t)((value - (float)cel) * 100);
	val = cel << 8 | drob;
	}



	//chprintf(SHELL_IFACE, "val %x\r\n", val);
	return val;
}

void copy_to_ble(void){
#ifdef SD_MODULE_TRAINER
#ifdef USE_MATH_MODULE
	/*
	thdg->value = convert_to_ble_type(lastFilterValues[1][FILTER_BUFFER_SIZE - 1]);
	twd->value = convert_to_ble_type(lastFilterValues[4][FILTER_BUFFER_SIZE - 1]);
	tws->value = convert_to_ble_type(lastFilterValues[5][FILTER_BUFFER_SIZE - 1]);
	twa->value = convert_to_ble_type(lastFilterValues[6][FILTER_BUFFER_SIZE - 1]);
	twa_tg->value = convert_to_ble_type(windAngleTarget);
	bs_tg->value = convert_to_ble_type(hullSpeedTarget);

	hdg->value = convert_to_ble_type(fmod(lastSensorValues[HDM] + 3600.0, 360.0));
	//hdg->value = convert_to_ble_type(lastFilterValues[0][FILTER_BUFFER_SIZE - 1]);
	heel->value = convert_to_ble_type(lastSensorValues[HEEL]);
	*/
	thdg->value = convert_to_ble_type(lastSensorValues[HDT]);
		twd->value = convert_to_ble_type(lastSensorValues[TWD]);
		tws->value = convert_to_ble_type(lastSensorValues[TWS]);
		twa->value = convert_to_ble_type(lastSensorValues[TWA]);
		twa_tg->value = convert_to_ble_type(lastSensorValues[TWA_TGT]);
		bs_tg->value = convert_to_ble_type(lastSensorValues[VMG_TGT]);
		bs->value = convert_to_ble_type((float)(pvt_box->gSpeed * 0.0036 / 1.852));
		heel->value = convert_to_ble_type(bno055->d_euler_hpr.h);
		hdg->value = convert_to_ble_type(fmod(lastSensorValues[HDM] + 3600.0, 360.0));
		//hdg->value = convert_to_ble_type(lastFilterValues[0][FILTER_BUFFER_SIZE - 1]);
		heel->value = convert_to_ble_type(lastSensorValues[HEEL]);
#endif
	//heel->value = convert_to_ble_type(bno055->d_euler_hpr.r);

	//hdg->value = convert_to_ble_type(lastFilterValues[0][0]);
	//heel->value = convert_to_ble_type(lastFilterValues[2][0]);

#endif
}


void send_json(void)
{

	//return;
	chSemWait(&usart1_semaph);
	chprintf(SHELL_IFACE, "\r\n{\"msg_type\":\"boats_data\",\r\n\t\t\"boat_1\":{\r\n\t\t\t");
#ifdef USE_UBLOX_GPS_MODULE
		chprintf(SHELL_IFACE, "\"hour\":%d,\r\n\t\t\t", pvt_box->hour);
		chprintf(SHELL_IFACE, "\"min\":%d,\r\n\t\t\t", pvt_box->min);
		chprintf(SHELL_IFACE, "\"sec\":%d,\r\n\t\t\t", pvt_box->sec);
		chprintf(SHELL_IFACE, "\"lat\":%f,\r\n\t\t\t", pvt_box->lat / 10000000.0f);
		chprintf(SHELL_IFACE, "\"lon\":%f,\r\n\t\t\t", pvt_box->lon / 10000000.0f);
		chprintf(SHELL_IFACE, "\"speed\":%f,\r\n\t\t\t", (float)(pvt_box->gSpeed * 0.0036));
		chprintf(SHELL_IFACE, "\"dist\":%d,\r\n\t\t\t", (uint16_t)odo_box->distance);
#endif
#ifdef USE_BNO055_MODULE
		//chprintf(SHELL_IFACE, "\"yaw\":%d,\r\n\t\t\t", (uint16_t)bno055->d_euler_hpr.h);
		//chprintf(SHELL_IFACE, "\"yaw\":%d,\r\n\t\t\t", (uint16_t)hmc5883->yaw);
		chprintf(SHELL_IFACE, "\"yaw\":%d,\r\n\t\t\t", (uint16_t)hmc6343->yaw);
		chprintf(SHELL_IFACE, "\"pitch\":%f,\r\n\t\t\t", bno055->d_euler_hpr.p);
		chprintf(SHELL_IFACE, "\"roll\":%f,\r\n\t\t\t", bno055->d_euler_hpr.r);
	//	chprintf(SHELL_IFACE, "\"magn_cal\":%d,\r\n\t\t\t", bno055->calib_stat.magn);
	//	chprintf(SHELL_IFACE, "\"accel_cal\":%d,\r\n\t\t\t", bno055->calib_stat.accel);
	//	chprintf(SHELL_IFACE, "\"gyro_cal\":%d,\r\n\t\t\t", bno055->calib_stat.gyro);
	//	chprintf(SHELL_IFACE, "\"sys_cal\":%d,\r\n\t\t\t", bno055->calib_stat.system);
#endif
#ifdef USE_UBLOX_GPS_MODULE
		chprintf(SHELL_IFACE, "\"headMot\":%d,\r\n\t\t\t", (uint16_t)(pvt_box->headMot / 100000));
		chprintf(SHELL_IFACE, "\"sat\":%d,\r\n\t\t\t", pvt_box->numSV);
#endif
		chprintf(SHELL_IFACE, "\"rudder\":%f,\r\n\t\t\t", r_rudder->degrees);
		chprintf(SHELL_IFACE, "\"log\":%f,\r\n\t\t\t", r_lag->meters);
#ifdef USE_WINDSENSOR_MODULE
		chprintf(SHELL_IFACE, "\"wind_dir\":%d,\r\n\t\t\t", wind->direction);
		chprintf(SHELL_IFACE, "\"wind_spd\":%f,\r\n\t\t\t", wind->speed);
#endif
		chprintf(SHELL_IFACE, "\"rssi\":%d,\r\n\t\t\t", 0);
		//	chprintf(SHELL_IFACE, "\"accel_raw\":%d; %d; %d,\r\n\t\t\t", bno055->accel_raw.x, bno055->accel_raw.y, bno055->accel_raw.z);
	//	chprintf(SHELL_IFACE, "\"gyro_raw\":%d; %d; %d,\r\n\t\t\t", bno055->gyro_raw.x, bno055->gyro_raw.y, bno055->gyro_raw.z);
	//	chprintf(SHELL_IFACE, "\"magn_raw\":%d; %d; %d,\r\n\t\t\t", bno055->mag_raw.x, bno055->mag_raw.y, bno055->mag_raw.z);

		chprintf(SHELL_IFACE, "\"bat\":0\r\n\t\t\t");
		chprintf(SHELL_IFACE, "}\r\n\t}");
		chSemSignal(&usart1_semaph);
}

#ifdef SD_SENSOR_BOX_LAG
void send_lag_over_ble(lag_t *lag){
	/*uint16_t spd_cel;
	uint8_t spd_drob;
	spd_cel = (uint16_t)lag->meters;
	spd_drob = (uint8_t)((lag->meters - (float)spd_cel) * 100);
	chprintf(SHELL_IFACE, "Deg %d %d %4x%2x  ", spd_cel, spd_drob, spd_cel, spd_drob);*/
	int32_t val;
	val = convert_to_ble_type(lag->meters);
	nina_notify(ble_lag, val);
}
#endif

#ifdef SD_SENSOR_BOX_RUDDER
void send_rudder_over_ble(rudder_t *rudder){
	int32_t val;
	/*uint16_t degrees_cel;
	uint8_t degrees_drob;
	degrees_cel = (uint16_t)rudder->degrees;
	degrees_drob = (uint8_t)((rudder->degrees - (float)degrees_cel) * 100);
	chprintf(SHELL_IFACE, "Deg %d %d %4x%2x", degrees_cel, degrees_drob, degrees_cel, degrees_drob);*/
	val = convert_to_ble_type(rudder->degrees);
	nina_notify(ble_rudder, val);
}
#endif

void cmd_start(BaseSequentialStream* chp, int argc, char* argv[]) {
	if (argc != 0) {
		if (strcmp(argv[0], "test") == 0) {
			toggle_test_output();
			chprintf(chp, "Test started\n\r");
			return;
		}
		if (strcmp(argv[0], "gps") == 0) {
			toggle_gps_output();
			chprintf(chp, "GPS started\n\r");
			return;
		}
		if (strcmp(argv[0], "ypr") == 0) {
			toggle_ypr_output();
			return;
		}
		if (strcmp(argv[0], "gyro") == 0) {
			toggle_gyro_output();
			return;
		}
	}
	chprintf(chp, "Usage: start test|gps|ypr|gyro\n\r");
}

void cmd_c(BaseSequentialStream* chp, int argc, char* argv[]) {
	(void) argc;
	(void) argv;
	stop_all_tests();
	chprintf(chp, "Stopped all outputs\n\r");
}

void cmd_reset(BaseSequentialStream* chp, int argc, char* argv[]){
	(void) argc;
	(void) argv;
	chprintf(chp, "\r\nReset system");
	chThdSleepMilliseconds(500);
	chprintf(chp, ".");
	chThdSleepMilliseconds(500);
	chprintf(chp, ".");
	chThdSleepMilliseconds(500);
	chprintf(chp, ".");
	chThdSleepMilliseconds(500);
	chprintf(chp, "\r\n");
	NVIC_SystemReset();
}

void cmd_boot(BaseSequentialStream* chp, int argc, char* argv[]) {
	(void) argc;
	(void) argv;
	chprintf(chp, "Entering bootloader after system reset");
	chThdSleepMilliseconds(500);
	chprintf(chp, ".");
	chThdSleepMilliseconds(500);
	chprintf(chp, ".");
	chThdSleepMilliseconds(500);
	chprintf(chp, ".");
	chThdSleepMilliseconds(500);
	chprintf(chp, "\r\n");

	// *((unsigned long *)(SYMVAL(__ram0_end__) - 4)) = 0xDEADBEEF;

	 //*((unsigned long *) BKPSRAM_BASE) = 0xDEADBEEF;
	 RTC->BKP0R = MAGIC_BOOTLOADER_NUMBER;	// set magic flag => reset handler will jump into boot loader

	 if (RTC->BKP0R == MAGIC_BOOTLOADER_NUMBER) {
	 chprintf(chp, "Writed to the end of RAM %x, reset\r\n", RTC->BKP0R);
	 chThdSleepMilliseconds(500);
	 NVIC_SystemReset();
	 }else{
		 chprintf(chp, "Comparsion failed\r\n");
	 }

}

void cmd_ublox(BaseSequentialStream* chp, int argc, char* argv[]) {
	if (argc != 0) {
		if (strcmp(argv[0], "cog_lpf") == 0) {
			chprintf(chp, "Cog stat, argc %d\r\n", argc);
			if (argc == 2) {
				if (strcmp(argv[1], "on") == 0) {
					neo_toggle_cog_lpf(true);
				} else if (strcmp(argv[1], "off") == 0) {
					neo_toggle_cog_lpf(false);
				} else if (strcmp(argv[1], "status") == 0) {
					neo_get_lpf_status();
				} else if (strcmp(argv[1], "help") == 0) {
					chprintf(chp,
							"Usage: ublox cog_lpf on/off | status | max_speed | max_pos_acc | lp_gain\r\n");
				} else {
					chprintf(chp, "Usage: ublox cog_lpf on|off\r\n");
				}
			} else if (argc == 3) {
				if (strcmp(argv[1], "max_speed") == 0) {
					neo_set_cog_speed(argv[2]);
				} else if (strcmp(argv[1], "max_pos_acc") == 0) {
					neo_set_cog_pos_acc(argv[2]);
				} else if (strcmp(argv[1], "lp_gain") == 0) {
					neo_set_cog_gain(argv[2]);
				} else {
					chprintf(chp,
							"Usage: ublox cog_lpf on/off | status | max_speed | max_pos_acc | lp_gain\r\n");
				}
			} else {
				chprintf(chp,
						"Usage: ublox cog_lpf on/off | status | max_speed | max_pos_acc | lp_gain\r\n");
			}
			return;
		}
		if (strcmp(argv[0], "vel_lpf") == 0) {
			if (argc == 2) {
				if (strcmp(argv[1], "on") == 0) {
					neo_toggle_vel_lpf(true);
				} else if (strcmp(argv[1], "off") == 0) {
					neo_toggle_vel_lpf(false);
				}
			} else if (argc == 3) {
				if (strcmp(argv[1], "lp_gain") == 0) {
					neo_set_vel_gain(argv[2]);
				} else {
					chprintf(chp, "Usage: ublox vel_lpf lp_gain");
				}
			} else {
				neo_get_lpf_status();
			}
			return;
		}
		if (strcmp(argv[0], "slas") == 0) {
			if (argc == 2) {
				if (strcmp(argv[1], "on") == 0) {
					neo_toggle_slas(true);
				} else if (strcmp(argv[1], "off") == 0) {
					neo_toggle_slas(false);
				} else if (strcmp(argv[1], "status") == 0) {
					neo_get_slas_sbas_status();
				}
			} else {
				chprintf(chp, "Usage: ublox slas on/off | status\r\n");
			}
			return;
		}
		if (strcmp(argv[0], "sbas") == 0) {
			if (argc == 2) {
				if (strcmp(argv[1], "on") == 0) {
					neo_toggle_sbas(true);
				} else if (strcmp(argv[1], "off") == 0) {
					neo_toggle_sbas(false);
				} else if (strcmp(argv[1], "status") == 0) {
					neo_get_slas_sbas_status();
				}
			} else {
				chprintf(chp, "Usage: ublox sbas on/off | status\r\n");
			}
			return;
		}
		if (strcmp(argv[0], "rtk") == 0) {
			if (argc == 2) {
				if (strcmp(argv[1], "on") == 0) {
					neo_toggle_rtk(true);
				} else if (strcmp(argv[1], "off") == 0) {
					neo_toggle_rtk(false);
				} else if (strcmp(argv[1], "status") == 0) {
					neo_get_rtk_status();
				}
			} else {
				chprintf(chp, "Usage: ublox sbas on/off | status\r\n");
			}
			return;
		}
		if (strcmp(argv[0], "stat") == 0) {
			neo_get_full_status();
			return;
		}
		if (strcmp(argv[0], "rate") == 0) {
			if (argc == 2) {
				if (strcmp(argv[1], "status") == 0) {
					neo_get_rate_status();
				}
			} else if (argc == 3) {
				if (strcmp(argv[1], "set") == 0) {
					neo_set_rate_ms(argv[2]);
				}
			} else {
				chprintf(chp, "Usage: ublox rate status | set < ms >\r\n");
			}
			return;
		}
	}
	chprintf(chp, "Usage: ublox lpf|slas|sbas|rtk|stat\n\r");
}

#ifdef USE_XBEE_868_MODULE
void cmd_xbee(BaseSequentialStream* chp, int argc, char* argv[]) {
	if (argc != 0) {
		if (strcmp(argv[0], "addr") == 0) {
			xbee_get_addr();
			return;
		}
		if (strcmp(argv[0], "dest") == 0) {
			if (argc == 2) {
				//xbee_set_dest(atoi(argv[1]));
			}
			//xbee_get_dest();
			return;
		}
		if (strcmp(argv[0], "mesh") == 0) {
			return;
		}
		if (strcmp(argv[0], "rssi") == 0) {
			xbee_get_rssi();
			return;
		}
		if (strcmp(argv[0], "stat") == 0) {
			xbee_get_stat();
			return;
		}
		if (strcmp(argv[0], "ping") == 0) {
			xbee_get_ping();
			return;
		}
		if (strcmp(argv[0], "channels") == 0) {
			xbee_get_channels();
			return;
		}
		if (strcmp(argv[0], "lb") == 0) {
			if (argc == 2) {
				xbee_set_loopback(argv);
			} else {
				xbee_get_lb_status();
			}
			return;
		}
	}
	chprintf(chp, "Usage: xbee addr|dest|mesh|rssi|ping|stat|lb\n\r");
}
#endif
void toggle_test_output(void) {
	output->type = OUTPUT_TEST;
	output->ble = OUTPUT_TEST;
#ifdef USE_BNO055_MODULE
	bno055->read_type = OUTPUT_TEST;
#endif
	output->service = 0;
	output->test = (~output->test) & 0x01;
}

void toggle_ble_output(void) {
	output->service = 0;
	output->type = OUTPUT_BLE;
}

void toggle_gps_output(void) {
	output->service = 0;
	output->gps = (~output->gps) & 0x01;
}

void toggle_ypr_output(void) {
	output->service =  0;
	output->ypr = (~output->ypr) & 0x01;
}

void toggle_gyro_output(void) {
	output->service = 0;
	output->gyro = (~output->gyro) & 0x01;
}



void stop_all_tests(void) {
	if (output->type == OUTPUT_ALL_CALIB){
		output->type = OUTPUT_SERVICE;
	}else{
		output->type = OUTPUT_NONE;
		output->ble = OUTPUT_NONE;
#ifdef USE_BNO055_MODULE
		bno055->read_type = OUTPUT_NONE;
#endif
	}
}


