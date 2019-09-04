#include "config.h"
#include "sd_shell_cmds.h"
#include <hal.h>
#include "shellconf.h"
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
extern struct ch_semaphore usart1_semaph;

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
		{ "service", cmd_service },
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
#ifdef USE_ADC_MODULE
		//{"adc", cmd_adc },
#endif
		{ NULL, NULL }
};

static const ShellConfig shell_cfg1 = { (BaseSequentialStream*) &SHELL_SD,
		commands, history_buffer, 32, complete_buffer };

thread_t *cmd_init(void) {
	return chThdCreateFromHeap(NULL, SHELL_WA_SIZE, "shell", NORMALPRIO,
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

	chRegSetThreadName("Data output");
	systime_t prev = chVTGetSystemTime(); // Current system time.

	while (true) {
		//wdgReset(&WDGD1);
		palToggleLine(LINE_GREEN_LED);
		chThdSleepMilliseconds(5);
		send_json();
		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(100));
	}
}
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

void send_json(void)
{
	chSemWait(&usart1_semaph);
	chprintf((BaseSequentialStream*)&SD1, "\r\n{\"msg_type\":\"boats_data\",\r\n\t\t\"boat_1\":{\r\n\t\t\t");
#ifdef USE_UBLOX_GPS_MODULE
		chprintf((BaseSequentialStream*)&SD1, "\"hour\":%d,\r\n\t\t\t", pvt_box->hour);
		chprintf((BaseSequentialStream*)&SD1, "\"min\":%d,\r\n\t\t\t", pvt_box->min);
		chprintf((BaseSequentialStream*)&SD1, "\"sec\":%d,\r\n\t\t\t", pvt_box->sec);
		chprintf((BaseSequentialStream*)&SD1, "\"lat\":%f,\r\n\t\t\t", pvt_box->lat / 10000000.0f);
		chprintf((BaseSequentialStream*)&SD1, "\"lon\":%f,\r\n\t\t\t", pvt_box->lon / 10000000.0f);
		chprintf((BaseSequentialStream*)&SD1, "\"speed\":%f,\r\n\t\t\t", (float)(pvt_box->gSpeed * 0.0036));
		chprintf((BaseSequentialStream*)&SD1, "\"dist\":%d,\r\n\t\t\t", (uint16_t)odo_box->distance);
#endif
#ifdef USE_BNO055_MODULE
		chprintf((BaseSequentialStream*)&SD1, "\"yaw\":%d,\r\n\t\t\t", (uint16_t)bno055->d_euler_hpr.h);
		chprintf((BaseSequentialStream*)&SD1, "\"pitch\":%f,\r\n\t\t\t", bno055->d_euler_hpr.p);
		chprintf((BaseSequentialStream*)&SD1, "\"roll\":%f,\r\n\t\t\t", bno055->d_euler_hpr.r);
#endif
#ifdef USE_UBLOX_GPS_MODULE
		chprintf((BaseSequentialStream*)&SD1, "\"headMot\":%d,\r\n\t\t\t", (uint16_t)(pvt_box->headMot / 100000));
		chprintf((BaseSequentialStream*)&SD1, "\"sat\":%d,\r\n\t\t\t", pvt_box->numSV);
#endif
#ifdef USE_WINDSENSOR_MODULE
		chprintf((BaseSequentialStream*)&SD1, "\"wind_dir\":%d,\r\n\t\t\t", wind->direction);
		chprintf((BaseSequentialStream*)&SD1, "\"wind_spd\":%f,\r\n\t\t\t", wind->speed);
#endif
		chprintf((BaseSequentialStream*)&SD1, "\"rssi\":%d,\r\n\t\t\t", 0);
		//	chprintf((BaseSequentialStream*)&SD1, "\"accel_raw\":%d; %d; %d,\r\n\t\t\t", bno055->accel_raw.x, bno055->accel_raw.y, bno055->accel_raw.z);
	//	chprintf((BaseSequentialStream*)&SD1, "\"gyro_raw\":%d; %d; %d,\r\n\t\t\t", bno055->gyro_raw.x, bno055->gyro_raw.y, bno055->gyro_raw.z);
	//	chprintf((BaseSequentialStream*)&SD1, "\"magn_raw\":%d; %d; %d,\r\n\t\t\t", bno055->mag_raw.x, bno055->mag_raw.y, bno055->mag_raw.z);
	//	chprintf((BaseSequentialStream*)&SD1, "\"magn_cal\":%d,\r\n\t\t\t", bno055->magn_cal);
	//	chprintf((BaseSequentialStream*)&SD1, "\"accel_cal\":%d,\r\n\t\t\t", bno055->accel_cal);
	//	chprintf((BaseSequentialStream*)&SD1, "\"gyro_cal\":%d,\r\n\t\t\t", bno055->gyro_cal);
		chprintf((BaseSequentialStream*)&SD1, "\"bat\":0\r\n\t\t\t");
		chprintf((BaseSequentialStream*)&SD1, "}\r\n\t}");
		chSemSignal(&usart1_semaph);
}

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
	output->service = 0;
	output->test = (~output->test) & 0x01;
}

void toggle_gps_output(void) {
	output->service = 0;
	output->gps = (~output->gps) & 0x01;
}

void toggle_ypr_output(void) {
	output->service = 1;
	output->ypr = (~output->ypr) & 0x01;
}

void toggle_gyro_output(void) {
	output->service = 1;
	output->gyro = (~output->gyro) & 0x01;
}

void stop_all_tests(void) {
	output->test = 0;
	output->gps = 0;
	output->ypr = 0;
	output->gyro = 0;
}


