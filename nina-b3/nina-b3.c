/*
 * nina-b3.c
 *
 *  Created on: Apr 26, 2019
 *      Author: a-h
 */

#include "nina-b3.h"
#include "config.h"
#include <stdlib.h>
#include <string.h>
#include "stdint.h"
#include "math.h"
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"
#include "sd_shell_cmds.h"
#include "lag.h"
#include "adc.h"
extern struct ch_semaphore usart1_semaph;
#ifdef SD_MODULE_TRAINER
	ble_charac_t *thdg;
	ble_charac_t *rdr;
	ble_charac_t *twd;
	ble_charac_t *tws;
	ble_charac_t *twa;
	ble_charac_t *bs;
	ble_charac_t *twa_tg;
	ble_charac_t *bs_tg;
	ble_charac_t *hdg;
	ble_charac_t *heel;
	ble_charac_t *charac_array[NUM_OF_CHARACTS];

	ble_remote_t *remote_lag;
	ble_remote_t *remote_rudder;
	ble_remote_t *remote_wind;
#endif

#ifdef SD_SENSOR_BOX_RUDDER
	ble_charac_t *ble_rudder;
	ble_charac_t *charac_array[NUM_OF_CHARACTS];
#endif

#ifdef SD_SENSOR_BOX_LAG
	ble_charac_t *ble_lag;
	ble_charac_t *charac_array[NUM_OF_CHARACTS];
#endif

#ifdef SD_SENSOR_BOX_WIND

#endif
	extern output_t *output;
	lag_t *r_lag;
	rudder_t *r_rudder;
	ble_temp_charac_t *charac_temporary;
ble_peer_t *peer;
ble_t *ble;
static const SerialConfig nina_config =
		{ 115200, 0, USART_CR2_STOP1_BITS, 0 };

static THD_WORKING_AREA(ble_parsing_thread_wa, 4096);
static THD_FUNCTION(ble_parsing_thread, arg);
static THD_WORKING_AREA(ble_thread_wa, 4096);
static THD_FUNCTION(ble_thread, arg);

void start_ble_module(void){
	chThdCreateStatic(ble_parsing_thread_wa, sizeof(ble_parsing_thread_wa), NORMALPRIO + 1,
			ble_parsing_thread, NULL);
	chThdCreateStatic(ble_thread_wa, sizeof(ble_thread_wa), NORMALPRIO + 1,
				ble_thread, NULL);
}

/*
 * Thread to process data collection and filtering from NEO-M8P
 */

static THD_FUNCTION(ble_parsing_thread, arg) {
	(void) arg;
	uint8_t token;
	int8_t megastring[256];
	uint16_t i = 0;
	uint8_t str_flag = 0;
	chRegSetThreadName("BLE Parsing Thread");
	sdStart(&NINA_IF, &nina_config);
	//systime_t prev = chVTGetSystemTime(); // Current system time.
	while (true) {
		token = sdGet(&NINA_IF);
		megastring[i] = token;
		i++;
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*) &SD1, "%c", token);
		chSemSignal(&usart1_semaph);
		if (token == '\r' || token == '+'){
			str_flag = 1;
		}else if ((token == '\n') && (str_flag == 1)){
			nina_parse_command(megastring);
			memset(megastring, 0, 256);
			i = 0;
			str_flag = 0;
		}
		if (i == 256){
			i = 0;
		}

	//	palToggleLine(LINE_ORANGE_LED);
		//prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(500));
	}
}

static THD_FUNCTION(ble_thread, arg) {
	(void) arg;
	uint8_t token;
	chRegSetThreadName("BLE Conrol Thread");
	nina_fill_memory();
	nina_init_services();
	//systime_t prev = chVTGetSystemTime(); // Current system time.
	while (true) {
		chThdSleepMilliseconds(1000);
	//	palToggleLine(LINE_ORANGE_LED);
		//prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(500));
	}
}

uint8_t nina_parse_command(int8_t *strp) {
	uint8_t scan_res;
	uint8_t scanned_vals[32];
	uint32_t val, scanned_32;
	int8_t addr[16] = {0};
	//chprintf((BaseSequentialStream*) &SD1, "Parsing %s\r\n", strp);
	scan_res = sscanf(strp, "+UBTGSN:%d,%d,%x\r", &scanned_vals[0], &scanned_vals[1],	&val);
	if (scan_res == 1) {

		return 1;
	}

	scan_res = sscanf(strp, "+UBTGCHA:%d,%d\r", &scanned_vals[0],
			&scanned_vals[1]);
	if (scan_res == 2) {
		charac_temporary->cccd_handle = scanned_vals[0];
		charac_temporary->value_handle = scanned_vals[1];
		charac_temporary->parsed = 1;
		chprintf((BaseSequentialStream*) &SD1, "Scanned charac descript %d %d\r\n", scanned_vals[0], scanned_vals[1]);
		return 1;
	}

	scan_res = sscanf(strp, "+UBTGSER:%d,%d\r", &scanned_vals[0]);
	if (scan_res == 1) {

		return 1;
	}

	scan_res = sscanf(strp, "+UBTGCHA:%d,%d\r", &scanned_vals[0],
			&scanned_vals[1]);
	if (scan_res == 2) {

		return 1;
	}
	scan_res = sscanf(strp, "+UUBTGN:%d,%d,%x\r", &scanned_vals[0],
				&scanned_vals[1], &scanned_32);
		if (scan_res == 3) {
			nina_parse_notification(scanned_vals[0],
				scanned_vals[1], scanned_32);
			return 1;
		}
/*	if (strstr(strp, "UUBTACLC:") != NULL){
		chprintf((BaseSequentialStream*) &SD1, "Scanned %s\r\n", strp+12);
		strncpy(addr, strp+(strlen(strp)-1), 12);
		chprintf((BaseSequentialStream*) &SD1, "Copied %s\r\n", addr);
		scanned_vals[0] = atoi(strp+12);
		chprintf((BaseSequentialStream*) &SD1, "scanned vla 0 %d\r\n", scanned_vals[0]);
		scanned_vals[1] = atoi(strp+11);
				chprintf((BaseSequentialStream*) &SD1, "scanned vla 1 %d\r\n", scanned_vals[1]);
		return 1;
	}*/
	scan_res = sscanf(strp, "+UUBTACLC:%d,%d,%12sp\r\n", &scanned_vals[0], &scanned_vals[1], addr);
	if (scan_res == 3) {
		chprintf((BaseSequentialStream*) &SD1, "Scanned connected to dev %d %d %s\r\n", scanned_vals[0], scanned_vals[1], addr);
		nina_register_remote_dev(scanned_vals[0], scanned_vals[1], addr);
		return 1;
	}

	scan_res = sscanf(strp, "+UUBTACLC:%d,%d,%12sr\r\n", &scanned_vals[0], &scanned_vals[1], addr);
	//chprintf((BaseSequentialStream*) &SD1, "Scanned %d\r\n", scan_res);
	if (scan_res == 3) {
		chprintf((BaseSequentialStream*) &SD1, "Scanned connected dev %d %d %s\r\n", scanned_vals[0], scanned_vals[1], addr);
		nina_register_peer(scanned_vals[0], scanned_vals[1], addr);
		return 1;
	}


	scan_res = sscanf(strp, "+UUBTACLD:%d\r", &scanned_vals[0]);
	if (scan_res == 1) {
		nina_unregister_peer(scanned_vals[0]);
		return 1;
	}

	scan_res = sscanf(strp, "+UUBTB:%xr,%d\r", &addr, &scanned_vals[0]);
	if (scan_res == 2) {

		return 1;
	}
	if (strstr(strp, "OK") != NULL){

		return 1;
	}
	if (strstr(strp, "ERROR") != NULL){
		return 1;
	}
	return -1;
}

void nina_parse_notification(uint8_t conn_handle, uint8_t val_handle, uint32_t value){
#ifdef SD_MODULE_TRAINER
	if ((remote_lag->is_connected == 1) && (remote_lag->conn_handle == conn_handle)){
		r_lag->meters = (float)((value >> 8) + ((float)(value & 0xFF) / 100.0));
		bs->value = value;
		//chprintf((BaseSequentialStream*) &SD1, "R lag %f\r\n", r_lag->meters);
	}else if ((remote_rudder->is_connected == 1) && (remote_rudder->conn_handle == conn_handle)){
		r_rudder->degrees = (float)((value >> 8) + ((float)(value & 0xFF) / 100.0));
		rdr->value = value;
	//	chSemWait(&usart1_semaph);
	//	chprintf((BaseSequentialStream*) &SD1, "R rdr %f\r\n", r_rudder->degrees);
	//	chSemSignal(&usart1_semaph);
	}
#endif
}

void nina_connect(uint8_t *addr, uint8_t type){
	chprintf(NINA_IFACE, "AT+UBTACLC=%s,%d\r", addr, type);
}

void nina_register_peer(uint8_t conn_handle, uint8_t type, int8_t *addr){
	peer->conn_handle = conn_handle;
	peer->type = type;
	memcpy(peer->addr, addr, 12);
	peer->is_connected = 1;
	chprintf((BaseSequentialStream*) &SD1, "Connected %d %d %s\r\n", peer->conn_handle, peer->type, peer->addr);
#ifdef SD_MODULE_TRAINER
	output->type = OUTPUT_BLE;
#endif
#ifdef SD_SENSOR_BOX_RUDDER
	output->type = OUTPUT_RUDDER_BLE;
#endif
#ifdef SD_SENSOR_BOX_LAG
	output->type = OUTPUT_LAG_BLE;
#endif

}

void nina_unregister_peer(uint8_t conn_handle){
	(void)conn_handle;
	chprintf((BaseSequentialStream*) &SD1, "Disconnected %d %d %s\r\n", peer->conn_handle, peer->type, peer->addr);
	peer->is_connected = 0;
	peer->conn_handle = 0;
	peer->type = 0;
	memset(peer->addr, 0, 12);
	output->type = OUTPUT_NONE;
}

void nina_register_remote_dev(uint8_t conn_handle, uint8_t type, int8_t *addr){
#ifdef SD_MODULE_TRAINER
	if (strcmp(addr, "CCF95781688F") == 0){
		remote_lag->conn_handle = conn_handle;

	remote_lag->is_connected = 1;
	remote_lag->type = type;
	chprintf((BaseSequentialStream*) &SD1, "Connected to %d %d %s\r\n", remote_lag->conn_handle, remote_lag->type, addr);
	nina_get_remote_characs(remote_lag->conn_handle, 0x4A01);
	}
	if (strcmp(addr, "CCF957816647") == 0){
			remote_rudder->conn_handle = conn_handle;

		remote_rudder->is_connected = 1;
		remote_rudder->type = type;
		chprintf((BaseSequentialStream*) &SD1, "Connected to %d %d %s\r\n", remote_rudder->conn_handle, remote_rudder->type, addr);
		nina_get_remote_characs(remote_lag->conn_handle, 0x5A01);
		}
	else{
		nina_register_peer(conn_handle, type, addr);
	}
#else
	nina_register_peer(conn_handle, type, addr);
#endif
}

void nina_get_remote_characs(uint16_t handle, uint16_t uuid){
	//chprintf(NINA_IFACE, "AT+UBTGDP=%d\r", handle);
	//chThdSleepMilliseconds(300);
	//chprintf(NINA_IFACE, "AT+UBTGDCS=%d,%d,%d\r", handle, 30, 65535);
	//chThdSleepMilliseconds(300);
	//chprintf(NINA_IFACE, "AT+UBTGDCD=%d,%d,%d\r", handle, 32, 65535);
	//chThdSleepMilliseconds(300);
	chprintf(NINA_IFACE, "AT+UBTGWC=%d,%d,%d\r", handle, 33, 1);
}

void nina_unregister_remote_dev(uint8_t conn_handle){
	(void)conn_handle;
#ifdef SD_MODULE_TRAINER
	if (remote_lag->conn_handle == conn_handle) {
		chprintf((BaseSequentialStream*) &SD1, "Disconnected to %d %d %s\r\n",
				remote_lag->conn_handle, remote_lag->type);
		remote_lag->conn_handle = 0;

		remote_lag->is_connected = 0;
		remote_lag->type = 0;
	}
	if (remote_rudder->conn_handle == conn_handle) {
		chprintf((BaseSequentialStream*) &SD1, "Disconnected to %d %d %s\r\n",
				remote_rudder->conn_handle, remote_rudder->type);
		remote_rudder->conn_handle = 0;

		remote_rudder->is_connected = 0;
		remote_rudder->type = 0;
	}
#endif
}

void nina_fill_memory(void){
	charac_temporary = calloc(1, sizeof(ble_temp_charac_t));
	peer = calloc(1, sizeof(ble_peer_t));
	r_lag = calloc(1, sizeof(lag_t));
	r_rudder = calloc(1, sizeof(rudder_t));
#ifdef SD_MODULE_TRAINER
	thdg = calloc(1, sizeof(ble_charac_t));
	rdr = calloc(1, sizeof(ble_charac_t));
	twd = calloc(1, sizeof(ble_charac_t));
	tws = calloc(1, sizeof(ble_charac_t));
	twa = calloc(1, sizeof(ble_charac_t));
	bs = calloc(1, sizeof(ble_charac_t));
	twa_tg = calloc(1, sizeof(ble_charac_t));
	bs_tg = calloc(1, sizeof(ble_charac_t));
	hdg = calloc(1, sizeof(ble_charac_t));
	heel = calloc(1, sizeof(ble_charac_t));
	remote_lag = calloc(1, sizeof(ble_remote_t));
	remote_rudder = calloc(1, sizeof(ble_remote_t));
	remote_wind = calloc(1, sizeof(ble_remote_t));

	charac_array[0] = thdg;
	charac_array[1] = rdr;
	charac_array[2] = twd;
	charac_array[3] = tws;
	charac_array[4] = twa;
	charac_array[5] = bs;
	charac_array[6] = twa_tg;
	charac_array[7] = bs_tg;
	charac_array[8] = hdg;
	charac_array[9] = heel;

#endif

#ifdef SD_SENSOR_BOX_RUDDER
	ble_rudder = calloc(1, sizeof(ble_charac_t));
	charac_array[0] = ble_rudder;
#endif

#ifdef SD_SENSOR_BOX_LAG
	ble_lag = calloc(1, sizeof(ble_charac_t));
	charac_array[0] = ble_lag;
#endif

#ifdef SD_SENSOR_BOX_WIND

#endif
}
void nina_send_at(void){
	chprintf(NINA_IFACE, "AT\r");
}

void nina_init_module(void){
	nina_init_services();
}

uint8_t nina_wait_response(int8_t *at_command){

return NINA_SUCCESS;
}

//AT+UBTGCHA=3A01,10,1,1,0F00FF,0,3
uint8_t nina_add_charac(ble_charac_t *charac, uint16_t uuid, uint8_t properties,
		uint8_t sec_read, uint8_t sec_write, uint32_t def_val,
		uint8_t read_auth, uint8_t max_len) {
	uint8_t timeout = 0;
	charac->max_lenth = max_len;
	charac->properties = properties;
	charac->read_auth = read_auth;
	charac->security_read = sec_read;
	charac->security_write = sec_write;
	charac->uuid = uuid;
	charac->value = def_val;
	chprintf(NINA_IFACE, "AT+UBTGCHA=%x,%d,%d,%d,%06x,%d,%d\r", uuid, properties,
			sec_read, sec_write, def_val, read_auth, max_len);
	while(timeout++ < 100){
		if (charac_temporary->parsed == 1){
			charac->cccd_handle = charac_temporary->cccd_handle;
			charac->value_handle = charac_temporary->value_handle;
			charac_temporary->parsed = 0;
		}else{
			chThdSleepMilliseconds(5);
		}
	}
	timeout = 0;
	//nina_wait_charac_handlers(charac);
}

void nina_notify(ble_charac_t *ble_rudder, int16_t cel, uint8_t drob){
	chprintf(NINA_IFACE, "AT+UBTGSN=%d,%02d,%04x%02x\r", peer->conn_handle, ble_rudder->cccd_handle, cel, drob);
}

void nina_wait_charac_handlers(ble_charac_t *charac){
	uint16_t val_handle;
	uint16_t cccd_handle;
	//parse
	charac->cccd_handle = cccd_handle;
	charac->value_handle = val_handle;
}

#ifdef SD_MODULE_TRAINER

uint8_t nina_init_services(void){
	chprintf(NINA_IFACE, "AT+UBTLE=3\r");
	if (nina_wait_response("+UBTLE\r") != NINA_SUCCESS) {
		return -1;
	}
	chThdSleepMilliseconds(200);
	chprintf(NINA_IFACE, "AT&W\r");
	if (nina_wait_response("AT&W\r") != NINA_SUCCESS) {
		return -1;
	}
	chThdSleepMilliseconds(200);
	chprintf(NINA_IFACE, "AT+CPWROFF\r");
	if (nina_wait_response("+CPWROFF\r") != NINA_SUCCESS) {
		return -1;
	}
	chThdSleepMilliseconds(1000);
	chprintf(NINA_IFACE, "AT+UBTCFG=2,4\r");
	if (nina_wait_response("+UBTCFG\r") != NINA_SUCCESS) {
		return -1;
	}
	chThdSleepMilliseconds(200);
	chprintf(NINA_IFACE, "AT+UBTLN=FastSkipper-COACH\r");
	if (nina_wait_response("+UBTLN\r") != NINA_SUCCESS) {
		return -1;
	}
	chThdSleepMilliseconds(200);
	chprintf(NINA_IFACE, "AT&W\r");
	if (nina_wait_response("AT&W\r") != NINA_SUCCESS) {
		return -1;
	}
	chThdSleepMilliseconds(200);
	chprintf(NINA_IFACE, "AT+CPWROFF\r");
	if (nina_wait_response("+CPWROFF\r") != NINA_SUCCESS) {
		return -1;
	}
	chThdSleepMilliseconds(1000);
	// Create service for GATT server (send information to tablet) 16-bit
	//UUID = 3A00
	// Response send handle of the created service (int value).
	// +UBTGSER:SER_HAND
	chprintf(NINA_IFACE, "AT+UBTGSER=3A00\r");
	if (nina_wait_response("+UBTGSER\r") != NINA_SUCCESS) {
		return -1;
	}
	chThdSleepMilliseconds(50);
	/*
# Create characteristic for service (values, witch coach complex send to tablet)
# Parametrs command:
# uuid (new, can be the same of service uuid),
# properties (notification support),
# security_read (no encryption required),
# security_write (no encryption required),
# initial value 1000,
# read_auth (Read Authorized. Any client can read data without host intervention)
# max_length (aximum length of the characteristic in bytes. The maximum value is 512 bytes)
*/
	//# THDG - uuid 3A01
	//# Response +UBTGCHA:THDG_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(thdg, 0x3A01, 10, 1, 1, 0x0F00FF, 0, 3);
	/*
	chprintf(NINA_IFACE, "AT+UBTGCHA=3A01,10,1,1,0F00FF,0,3\r");
	if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
		return -1;
	}uint8_t
	*/
	chThdSleepMilliseconds(50);
	//# RDR - uuid 3A02
	//# Response +UBTGCHA:RDR_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(rdr, 0x3A02, 10, 1, 1, 0x0F00FF, 0, 3);
	/*
	chprintf(NINA_IFACE, "AT+UBTGCHA=3A02,10,1,1,0F00FF,0,3\r");
	if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
		return -1;
	}
	*/
	chThdSleepMilliseconds(50);
	//# TWD - uuid 3A03
	//# Response +UBTGCHA:TWD_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(thdg, 0x3A03, 10, 1, 1, 0x0F00FF, 0, 3);
	/*
	chprintf(NINA_IFACE, "AT+UBTGCHA=3A03,10,1,1,0F00FF,0,3\r");
	if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
		return -1;
	}
	*/
	chThdSleepMilliseconds(50);
	//# TWS - uuid 3A04
	//# Response +UBTGCHA:TWS_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(tws, 0x3A04, 10, 1, 1, 0x0F00FF, 0, 3);
	/*
	chprintf(NINA_IFACE, "AT+UBTGCHA=3A04,10,1,1,0F00FF,0,3\r");
	if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
		return -1;
	}
	*/
	chThdSleepMilliseconds(50);
	//# TWA - uuid 3A05
	//# Response +UBTGCHA:TWD_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(twa, 0x3A05, 10, 1, 1, 0x0F00FF, 0, 3);
	/*
	chprintf(NINA_IFACE, "AT+UBTGCHA=3A05,10,1,1,0F00FF,0,3\r");
	if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
		return -1;
	}
	*/
	chThdSleepMilliseconds(50);
	//# BS - uuid 3A06
	//# Response +UBTGCHA:TWD_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(bs, 0x3A06, 10, 1, 1, 0x0F00FF, 0, 3);
	/*
	chprintf(NINA_IFACE, "AT+UBTGCHA=3A06,10,1,1,0F00FF,0,3\r");
	if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
		return -1;
	}
	*/
	chThdSleepMilliseconds(50);
	//# TWA_TG - uuid 3A07
	//# Response +UBTGCHA:TWD_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(thdg, 0x3A07, 10, 1, 1, 0x0F00FF, 0, 3);
	/*
	chprintf(NINA_IFACE, "AT+UBTGCHA=3A07,10,1,1,0F00FF,0,3\r");
	if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
		return -1;
	}
	*/
	chThdSleepMilliseconds(50);
	//# BS_TG - uuid 3A08
	//# Response +UBTGCHA:TWD_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(thdg, 0x3A08, 10, 1, 1, 0x0F00FF, 0, 3);
	/*
	chprintf(NINA_IFACE, "AT+UBTGCHA=3A08,10,1,1,0F00FF,0,3\r");
	if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
		return -1;
	}
	*/
	chThdSleepMilliseconds(50);
	//# HDG - uuid 3A09
	//# Response +UBTGCHA:TWD_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(thdg, 0x3A09, 10, 1, 1, 0x0F00FF, 0, 3);
	/*
	chprintf(NINA_IFACE, "AT+UBTGCHA=3A09,10,1,1,0F00FF,0,3\r");
	if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
		return -1;
	}
	*/
	chThdSleepMilliseconds(50);
	//# HEEL - uuid 3A0A
		//# Response +UBTGCHA:TWD_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(thdg, 0x3A0A, 10, 1, 1, 0x0F00FF, 0, 3);
	/*
		chprintf(NINA_IFACE, "AT+UBTGCHA=3A0A,10,1,1,0F00FF,0,3\r");
		if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
			return -1;
		}
		*/
		chThdSleepMilliseconds(50);
		//# Save settings and reboot
/*		chprintf(NINA_IFACE, "AT&W\r");
		if (nina_wait_response("AT&W\r") != NINA_SUCCESS) {
			return -1;
		}
		chThdSleepMilliseconds(200);
		chprintf(NINA_IFACE, "AT+CPWROFF\r");
		if (nina_wait_response("+CPWROFF\r") != NINA_SUCCESS) {
			return -1;
		}
		*/
}
#endif
void nina_send_all(ble_peer_t *peer){
#ifdef SD_MODULE_TRAINER
	if (peer->is_connected == 1){
		chSemWait(&usart1_semaph);

	chprintf(NINA_IFACE, "AT+UBTGSN=%d,%d,%06x\r", peer->conn_handle, bs->cccd_handle, bs->value);
	chSemSignal(&usart1_semaph);
	chThdSleepMilliseconds(5);
	chSemWait(&usart1_semaph);
	chprintf(NINA_IFACE, "AT+UBTGSN=%d,%d,%06x\r", peer->conn_handle, rdr->cccd_handle, rdr->value);
	chSemSignal(&usart1_semaph);
	chThdSleepMilliseconds(5);
	chSemWait(&usart1_semaph);
	chprintf(NINA_IFACE, "AT+UBTGSN=%d,%d,%06x\r", peer->conn_handle, rdr->cccd_handle, hdg->value);
	chSemSignal(&usart1_semaph);
	chThdSleepMilliseconds(5);
	chSemWait(&usart1_semaph);
	chprintf(NINA_IFACE, "AT+UBTGSN=%d,%d,%06x\r", peer->conn_handle, rdr->cccd_handle, heel->value);
	chSemSignal(&usart1_semaph);
}
#endif
}
void nina_send_two(void){
	chprintf(NINA_IFACE, "AT+UBTGSN=3,32,FF00FF\r");
}

#ifdef SD_SENSOR_BOX_RUDDER
uint8_t nina_init_services(void){
	chprintf(NINA_IFACE, "AT+UBTLE=2\r");
	if (nina_wait_response("+UBTLE\r") != NINA_SUCCESS) {
		return -1;
	}
	chThdSleepMilliseconds(200);
	chprintf(NINA_IFACE, "AT&W\r");
	if (nina_wait_response("AT&W\r") != NINA_SUCCESS) {
		return -1;
	}
	chThdSleepMilliseconds(200);
	chprintf(NINA_IFACE, "AT+CPWROFF\r");
	if (nina_wait_response("+CPWROFF\r") != NINA_SUCCESS) {
		return -1;
	}

	chThdSleepMilliseconds(1000);
	chprintf(NINA_IFACE, "AT+UBTLN=FastSkipper-RUDDER\r");
	if (nina_wait_response("+UBTLN\r") != NINA_SUCCESS) {
		return -1;
	}
	chThdSleepMilliseconds(200);
	chprintf(NINA_IFACE, "AT&W\r");
	if (nina_wait_response("AT&W\r") != NINA_SUCCESS) {
		return -1;
	}
	chThdSleepMilliseconds(200);
	chprintf(NINA_IFACE, "AT+CPWROFF\r");
	if (nina_wait_response("+CPWROFF\r") != NINA_SUCCESS) {
		return -1;
	}
	chThdSleepMilliseconds(1000);
	// Create service for GATT server (send information to tablet) 16-bit
	//UUID = 4A00
	// Response send handle of the created service (int value).
	// +UBTGSER:SER_HAND
	chprintf(NINA_IFACE, "AT+UBTGSER=5A00\r");
	if (nina_wait_response("+UBTGSER\r") != NINA_SUCCESS) {
		return -1;
	}
	chThdSleepMilliseconds(200);
	/*
# Create characteristic for service (values, witch coach complex send to tablet)
# Parametrs command:
# uuid (new, can be the same of service uuid),
# properties (notification support),
# security_read (no encryption required),
# security_write (no encryption required),
# initial value 1000,
# read_auth (Read Authorized. Any client can read data without host intervention)
# max_length (aximum length of the characteristic in bytes. The maximum value is 512 bytes)
*/
	//# RDR - uuid 5A01
	//# Response +UBTGCHA:THDG_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(ble_rudder, 0x5A01, 10, 1, 1, 0x0F00FF, 0, 3);
	/*chprintf(NINA_IFACE, "AT+UBTGCHA=5A01,10,1,1,0F00FF,0,3\r");
	if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
		return -1;
	}*/
	chThdSleepMilliseconds(200);

		//# Save settings and reboot
	/*	chprintf(NINA_IFACE, "AT&W\r");
		if (nina_wait_response("AT&W\r") != NINA_SUCCESS) {
			return -1;
		}
		chThdSleepMilliseconds(200);
		chprintf(NINA_IFACE, "AT+CPWROFF\r");
		if (nina_wait_response("+CPWROFF\r") != NINA_SUCCESS) {
			return -1;
		}
		chThdSleepMilliseconds(200);*/
}
#endif

#ifdef SD_SENSOR_BOX_LAG
uint8_t nina_init_services(void){
	chprintf(NINA_IFACE, "AT+UBTLE=2\r");
	if (nina_wait_response("+UBTLE\r") != NINA_SUCCESS) {
		return -1;
	}
	chThdSleepMilliseconds(200);
	chprintf(NINA_IFACE, "AT&W\r");
	if (nina_wait_response("AT&W\r") != NINA_SUCCESS) {
		return -1;
	}
	chThdSleepMilliseconds(200);
	chprintf(NINA_IFACE, "AT+CPWROFF\r");
	if (nina_wait_response("+CPWROFF\r") != NINA_SUCCESS) {
		return -1;
	}

	chThdSleepMilliseconds(1000);
	chprintf(NINA_IFACE, "AT+UBTLN=FastSkipper-LOG\r");
	if (nina_wait_response("+UBTLN\r") != NINA_SUCCESS) {
		return -1;
	}
	chThdSleepMilliseconds(200);
	chprintf(NINA_IFACE, "AT&W\r");
	if (nina_wait_response("AT&W\r") != NINA_SUCCESS) {
		return -1;
	}
	chThdSleepMilliseconds(200);
	chprintf(NINA_IFACE, "AT+CPWROFF\r");
	if (nina_wait_response("+CPWROFF\r") != NINA_SUCCESS) {
		return -1;
	}
	chThdSleepMilliseconds(1000);
	// Create service for GATT server (send information to tablet) 16-bit
	//UUID = 4A00
	// Response send handle of the created service (int value).
	// +UBTGSER:SER_HAND
	chprintf(NINA_IFACE, "AT+UBTGSER=4A00\r");
	if (nina_wait_response("+UBTGSER\r") != NINA_SUCCESS) {
		return -1;
	}
	chThdSleepMilliseconds(200);
	/*
# Create characteristic for service (values, witch coach complex send to tablet)
# Parametrs command:
# uuid (new, can be the same of service uuid),
# properties (notification support),
# security_read (no encryption required),
# security_write (no encryption required),
# initial value 1000,
# read_auth (Read Authorized. Any client can read data without host intervention)
# max_length (aximum length of the characteristic in bytes. The maximum value is 512 bytes)
*/
	//# LOG - uuid 4A01
	//# Response +UBTGCHA:THDG_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(ble_lag, 0x4A01, 10, 1, 1, 0x0F00FF, 0, 3);
/*	chprintf(NINA_IFACE, "AT+UBTGCHA=4A01,10,1,1,0F00FF,0,3\r");
	if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
		return -1;
	}*/
	//chThdSleepMilliseconds(200);

		//# Save settings and reboot
/*		chprintf(NINA_IFACE, "AT&W\r");
		if (nina_wait_response("AT&W\r") != NINA_SUCCESS) {
			return -1;
		}
		chThdSleepMilliseconds(200);
		chprintf(NINA_IFACE, "AT+CPWROFF\r");
		if (nina_wait_response("+CPWROFF\r") != NINA_SUCCESS) {
			return -1;
		}*/
		//chThdSleepMilliseconds(200);
}
#endif

#ifdef SD_SENSOR_BOX_WIND
void nina_init_services(void){

}
#endif
