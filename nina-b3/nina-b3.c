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
#ifdef USE_POWER_MANAGEMENT
#include "pwr_mgmt_l4.h"
#endif
#include "fsm.h"

static virtual_timer_t ble_observe_tim;
static virtual_timer_t ble_data_tx_tim;
static ble_remote_dev_t ble_remote_dev_list[NUM_OF_REMOTE_DEV];

//Events for actions requests
static event_source_t ble_observe_request_event;
static event_source_t ble_data_tx_request_event;
static event_source_t ble_remote_dev_cfg_request_event;

#ifdef USE_POWER_MANAGEMENT
extern event_source_t power_state_change_event;
#endif

SM_DEFINE(BLE_SM_1, ble_remote_dev_list)

// State enumeration order must match the order of state
// method entries in the state map
enum ble_states
{
    ST_IDLE,
    ST_OBSERVE,
    ST_DATA_RX,
	ST_DATA_TX,
	ST_PAIRING,
	ST_REMOTE_CFG,
	ST_WAIT_FOR_RESPONSE
};

// State machine state functions
STATE_DECLARE(Idle, NoEventData)
STATE_DECLARE(Observe, ble_remote_dev_t)
STATE_DECLARE(Data_rx, NoEventData)
STATE_DECLARE(Data_tx, NoEventData)
STATE_DECLARE(Pairing, ble_remote_dev_t)
STATE_DECLARE(Remote_cfg, NoEventData)
STATE_DECLARE(Wait_for_response, NoEventData)

// State map to define state function order
BEGIN_STATE_MAP(ble_remote_dev_t)
    STATE_MAP_ENTRY(ST_Idle)
    STATE_MAP_ENTRY(ST_Observe)
    STATE_MAP_ENTRY(ST_Data_rx)
    STATE_MAP_ENTRY(ST_Data_tx)
	STATE_MAP_ENTRY(ST_Pairing)
	STATE_MAP_ENTRY(ST_Remote_cfg)
	STATE_MAP_ENTRY(ST_Wait_for_response)
END_STATE_MAP(ble_remote_dev_t)

// If we parsed new data from sensors
EVENT_DEFINE(BLE_Data_rx, NoEventData)
{
    // Given the SetSpeed event, transition to a new state based upon
    // the current state of the state machine
    BEGIN_TRANSITION_MAP                        // - Current State -
        TRANSITION_MAP_ENTRY(ST_DATA_RX)          // ST_Idle
    	TRANSITION_MAP_ENTRY(EVENT_IGNORED)          // ST_Observe
		TRANSITION_MAP_ENTRY(EVENT_IGNORED)          // ST_Data_rx
		TRANSITION_MAP_ENTRY(EVENT_IGNORED)          // ST_Data_tx
		TRANSITION_MAP_ENTRY(EVENT_IGNORED)          // ST_Pairing
		TRANSITION_MAP_ENTRY(EVENT_IGNORED)          // ST_Remote_cfg
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)     // ST_Wait_for_response
    END_TRANSITION_MAP(ble_remote_dev_t, pEventData)
}

// If timer said that we should transmit data
EVENT_DEFINE(BLE_Data_tx, NoEventData)
{
    // Given the SetSpeed event, transition to a new state based upon
    // the current state of the state machine
    BEGIN_TRANSITION_MAP                        // - Current State -
        TRANSITION_MAP_ENTRY(ST_DATA_TX)	         // ST_Idle
    	TRANSITION_MAP_ENTRY(EVENT_IGNORED)          // ST_Observe
		TRANSITION_MAP_ENTRY(EVENT_IGNORED)          // ST_Data_rx
		TRANSITION_MAP_ENTRY(EVENT_IGNORED)          // ST_Data_tx
		TRANSITION_MAP_ENTRY(EVENT_IGNORED)          // ST_Pairing
		TRANSITION_MAP_ENTRY(EVENT_IGNORED)          // ST_Remote_cfg
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)     // ST_Wait_for_response
    END_TRANSITION_MAP(ble_remote_dev_t, pEventData)
}

// If timer said that we should observe devices
EVENT_DEFINE(BLE_Observe, ble_remote_dev_t)
{
    // Given the SetSpeed event, transition to a new state based upon
    // the current state of the state machine
    BEGIN_TRANSITION_MAP                        // - Current State -
        TRANSITION_MAP_ENTRY(ST_OBSERVE)	         // ST_Idle
    	TRANSITION_MAP_ENTRY(EVENT_IGNORED)          // ST_Observe
		TRANSITION_MAP_ENTRY(EVENT_IGNORED)          // ST_Data_rx
		TRANSITION_MAP_ENTRY(EVENT_IGNORED)          // ST_Data_tx
		TRANSITION_MAP_ENTRY(EVENT_IGNORED)          // ST_Pairing
		TRANSITION_MAP_ENTRY(EVENT_IGNORED)          // ST_Remote_cfg
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)     // ST_Wait_for_response
    END_TRANSITION_MAP(ble_remote_dev_t, pEventData)
}

// If timer said that we should transmit data
EVENT_DEFINE(BLE_Remote_cfg, NoEventData)
{
    // Given the SetSpeed event, transition to a new state based upon
    // the current state of the state machine
    BEGIN_TRANSITION_MAP                        // - Current State -
        TRANSITION_MAP_ENTRY(ST_REMOTE_CFG)	         // ST_Idle
    	TRANSITION_MAP_ENTRY(EVENT_IGNORED)          // ST_Observe
		TRANSITION_MAP_ENTRY(EVENT_IGNORED)          // ST_Data_rx
		TRANSITION_MAP_ENTRY(EVENT_IGNORED)          // ST_Data_tx
		TRANSITION_MAP_ENTRY(EVENT_IGNORED)          // ST_Pairing
		TRANSITION_MAP_ENTRY(EVENT_IGNORED)          // ST_Remote_cfg
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)     // ST_Wait_for_response
    END_TRANSITION_MAP(ble_remote_dev_t, pEventData)
}

// If timer said that we should transmit data
EVENT_DEFINE(BLE_Go_idle, NoEventData)
{
    // Given the SetSpeed event, transition to a new state based upon
    // the current state of the state machine
    BEGIN_TRANSITION_MAP                        // - Current State -
        TRANSITION_MAP_ENTRY(ST_IDLE)	         // ST_Idle
    	TRANSITION_MAP_ENTRY(ST_IDLE)          // ST_Observe
		TRANSITION_MAP_ENTRY(ST_IDLE)          // ST_Data_rx
		TRANSITION_MAP_ENTRY(ST_IDLE)          // ST_Data_tx
		TRANSITION_MAP_ENTRY(ST_IDLE)          // ST_Pairing
		TRANSITION_MAP_ENTRY(ST_IDLE)          // ST_Remote_cfg
        TRANSITION_MAP_ENTRY(ST_IDLE)     // ST_Wait_for_response
    END_TRANSITION_MAP(ble_remote_dev_t, pEventData)
}

#ifdef USE_UBLOX_GPS_MODULE
#include "neo-m8.h"
extern ubx_nav_pvt_t *pvt_box;
extern ubx_nav_odo_t *odo_box;
#endif
#ifdef USE_BNO055_MODULE
#include "bno055_i2c.h"
#include "bno055.h"
extern bno055_t *bno055;
#endif
struct ch_semaphore usart_nina;
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

#include "adc.h"
extern dots_t *r_rudder_dots;
extern coefs_t *r_rudder_coefs;

#endif

#ifdef USE_MATH_MODULE
#include "sailDataMath.h"
extern float lastFilterValues[10][FILTER_BUFFER_SIZE];
extern float windAngleTarget;
extern float lastSensorValues[SIZE_BUFFER_VALUES];
extern float hullSpeedTarget;
extern float velocityMadeGoodTarget;
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
static const SerialConfig nina_config = { 115200, 0, USART_CR2_STOP1_BITS, 0 };

static THD_WORKING_AREA(ble_parsing_thread_wa, 4096);
static THD_FUNCTION(ble_parsing_thread, arg);
static THD_WORKING_AREA(ble_thread_wa, 4096);
static THD_FUNCTION(ble_thread, arg);

/**
 * Start BLE threads
 */
void start_ble_module(void){
	chThdCreateStatic(ble_parsing_thread_wa, sizeof(ble_parsing_thread_wa), NORMALPRIO + 1,
			ble_parsing_thread, NULL);
	chThdCreateStatic(ble_thread_wa, sizeof(ble_thread_wa), NORMALPRIO + 1,
				ble_thread, NULL);
}

/**
 * @brief BLE observe virtual timer callback
 * @param arg
 */
static void ble_observe_tim_cb(void *arg){
	chSysLockFromISR();
	chEvtBroadcastI(&ble_observe_request_event);
	chVTSetI(&ble_observe_tim, TIME_S2I(10), ble_observe_tim_cb, peer);
	chSysUnlockFromISR();
}

/**
 * @brief BLE data sending virtual timer callback
 * @param peer_1 Peer to which data will be send
 */
static void ble_data_tx_tim_cb(ble_peer_t *peer_1){
	chSysLockFromISR();
	if (peer_1->is_connected == 1){
	chEvtBroadcastI(&ble_data_tx_request_event);
	}
	chVTSetI(&ble_data_tx_tim, TIME_S2I(1), ble_data_tx_tim_cb, peer);
	chSysUnlockFromISR();
}

/**
 * @brief Thread with parsing data from nina and AT commands responses
 */
static THD_FUNCTION(ble_parsing_thread, arg) {
	(void) arg;
	uint8_t token;
	int8_t megastring[256];
	uint16_t i = 0;
	uint8_t str_flag = 0;
	chRegSetThreadName("BLE Parsing Thread");
	sdStart(&NINA_IF, &nina_config);
	while (true) {
		token = sdGet(&NINA_IF);
		megastring[i] = token;
		i++;

	//	chSemWait(&usart1_semaph);
	//	chprintf((BaseSequentialStream*) &SD1, "%c", token);
	//	chSemSignal(&usart1_semaph);

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
	}
}

/**
 * @brief Thread with nina and state machine initialisation
 */
static THD_FUNCTION(ble_thread, arg) {
	(void) arg;
	uint8_t token;
	uint8_t i = 0;
	event_listener_t el1, el2, el3, el4;

	chRegSetThreadName("BLE Conrol Thread");
	nina_fill_memory();
	chSemObjectInit(&usart_nina, 1);
	chThdSleepMilliseconds(500);
	nina_init_services();
	chThdSleepMilliseconds(500);
#ifdef SD_MODULE_TRAINER
	nina_init_devices(&ble_remote_dev_list);
#endif
	/* Events initialization and registration.*/
	chEvtObjectInit(&ble_observe_request_event);
	chEvtObjectInit(&ble_data_tx_request_event);
	chEvtObjectInit(&ble_remote_dev_cfg_request_event);

	chEvtRegisterMask(&ble_observe_request_event, &el1, EVENT_MASK(BLE_OBSERVE_EV));
	chEvtRegisterMask(&ble_data_tx_request_event, &el2, EVENT_MASK(BLE_DATA_TX_EV));
	chEvtRegisterMask(&ble_remote_dev_cfg_request_event, &el3, EVENT_MASK(BLE_REMOTE_CFG_EV));
#ifdef USE_POWER_MANAGEMENT
	chEvtRegisterMask(&power_state_change_event, &el4, EVENT_MASK(BLE_POWER_SWITCHED_ON_EV));
#endif
	//ALLOC_Init();

/*#ifdef SD_SENSOR_BOX_LAG
	output->type = OUTPUT_LAG_BLE;
#endif
#ifdef SD_SENSOR_BOX_RUDDER
	output->type = OUTPUT_RUDDER_BLE;
#endif
*/
#ifdef SD_MODULE_TRAINER
/*
	chprintf((BaseSequentialStream*) &SD1, "Connecting to lag module\r\n");
	nina_connect("D4CA6EB91DD3", 0); //LAG
	chThdSleepMilliseconds(2000);
	chprintf((BaseSequentialStream*) &SD1, "Connecting to rudder module\r\n");
	nina_connect("D4CA6EBAFDA0", 0); //Rudder
	chThdSleepMilliseconds(1000);
	if (remote_lag->is_connected == 1) {
		nina_get_remote_characs(remote_lag->conn_handle, 0x4A01);
	}
	if (remote_rudder->is_connected == 1) {
		nina_get_remote_characs(remote_rudder->conn_handle, 0x5A01);
	}
	*/
	chThdSleepMilliseconds(2500);
#endif
	//systime_t prev = chVTGetSystemTime(); // Current system time.
#ifdef SD_SENSOR_BOX_RUDDER
	output->ble = OUTPUT_RUDDER_BLE;
#endif
#ifdef SD_SENSOR_BOX_LAG
	output->ble = OUTPUT_LAG_BLE;
#endif
	while (true) {
#ifdef SD_MODULE_TRAINER

		/* LED timer initialization.*/
		  chVTObjectInit(&ble_observe_tim);
		  chVTObjectInit(&ble_data_tx_tim);
		  /* Starting blinker.*/
		  chVTSet(&ble_observe_tim, TIME_S2I(10), ble_observe_tim_cb, NULL);
		  chVTSet(&ble_data_tx_tim, TIME_S2I(1), ble_data_tx_tim_cb, peer);
while(true){
		SM_Event(BLE_SM_1, BLE_Go_idle, NULL);
}

	//	if (output->type == OUTPUT_TEST){
						copy_to_ble();
						nina_send_all(peer);
	//	}

						if (i++ == 10) {
		remote_lag->is_avalible = 0;
		remote_rudder->is_avalible = 0;

		if (remote_lag->is_connected == 0 || remote_rudder->is_connected == 0) {

			chprintf(NINA_IFACE, "AT+UBTD=2,1,2000\r");
			chThdSleepMilliseconds(2500);
			}
			if (remote_lag->is_connected == 0 && remote_lag->is_avalible == 1) {

				nina_connect("D4CA6EB91DD3", 0); //LAG
				chThdSleepMilliseconds(1500);

			}
			if (remote_rudder->is_connected == 0
					&& remote_rudder->is_avalible == 1) {

				nina_connect("D4CA6EBAFDA0", 0); //Rudder
				chThdSleepMilliseconds(1500);
			}
			i = 0;
						}

#endif
						SM_Event(BLE_SM_1, BLE_Go_idle, NULL);
		//chThdSleepMilliseconds(1000);
	}
}

#ifdef USE_BLE_MODULE

/**
 * @brief Convert all sensors data to BLE format (3-bytes data)
 */
void copy_to_ble(void){
#ifdef SD_MODULE_TRAINER
#ifdef USE_MATH_MODULE

	thdg->value = convert_to_ble_type(lastSensorValues[HDT]);
		twd->value = convert_to_ble_type(lastSensorValues[TWD]);
		tws->value = convert_to_ble_type(lastSensorValues[TWS]);
		twa->value = convert_to_ble_type(lastSensorValues[TWA]);
		twa_tg->value = convert_to_ble_type(lastSensorValues[TWA_TGT]);
		bs_tg->value = convert_to_ble_type(lastSensorValues[VMG_TGT]);
		//bs->value = convert_to_ble_type((float)(pvt_box->gSpeed * 0.0036 / 1.852));
		heel->value = convert_to_ble_type(bno055->d_euler_hpr.h);
		hdg->value = convert_to_ble_type(fmod(lastSensorValues[HDM] + 3600.0, 360.0));
		//hdg->value = convert_to_ble_type(lastFilterValues[0][FILTER_BUFFER_SIZE - 1]);
		heel->value = convert_to_ble_type(lastSensorValues[HEEL]);
#endif
#endif
}
#endif

// State machine sits here
STATE_DEFINE(Idle, NoEventData) {

		eventmask_t evt = chEvtWaitAny(ALL_EVENTS);

		if (evt & EVENT_MASK(BLE_POWER_SWITCHED_ON_EV)) {
			//if dev[i]->conn ==0
		//	palToggleLine(LINE_ORANGE_LED);
			nina_init_module();

		}

		if (evt & EVENT_MASK(BLE_OBSERVE_EV)) {
			//if dev[i]->conn ==0
			SM_Event(BLE_SM_1, BLE_Observe, NULL);
		}
		if (evt & EVENT_MASK(BLE_DATA_TX_EV)) {
			SM_Event(BLE_SM_1, BLE_Data_tx, NULL);
		}
		if (evt & EVENT_MASK(BLE_REMOTE_CFG_EV)) {
			SM_Event(BLE_SM_1, BLE_Remote_cfg, NULL);
		}
	//printf("%s ST_Idle\n", self->name);
}

STATE_DEFINE(Observe, ble_remote_dev_t) {
	ble_remote_dev_t* devlist = SM_GetInstance(ble_remote_dev_t);
	//SM_InternalEvent(ST_IDLE, NULL);
	chprintf(NINA_IFACE, "AT+UBTD=2,1,2000\r");

	chThdSleepMilliseconds(2500);

	uint8_t i = 0;
	for (i = 0; i < NUM_OF_REMOTE_DEV; i++) {
		if ((devlist[i].avalible == 1) && (devlist[i].connected == 0)){
			SM_InternalEvent(ST_PAIRING, NULL);
			break;
		}

	}
}

STATE_DEFINE(Pairing, ble_remote_dev_t) {
	ble_remote_dev_t* devlist = SM_GetInstance(ble_remote_dev_t);

	uint8_t i = 0;
	for (i = 0; i < NUM_OF_REMOTE_DEV; i++) {
		if ((devlist[i].avalible == 1) && (devlist[i].connected == 0)) {
			nina_connect(devlist[i].addr, 0);
			chThdSleepMilliseconds(1500);
			//SM_InternalEvent(ST_PAIRING, NULL);
		}
	}
}

STATE_DEFINE(Data_tx, NoEventData){
	copy_to_ble();
	nina_send_all(peer);
}

STATE_DEFINE(Remote_cfg, NoEventData){

}

STATE_DEFINE(Wait_for_response, NoEventData){

}

STATE_DEFINE(Data_rx, NoEventData){

}

/**
 * @brief Parsing strings from Nina's USART iface
 * @param strp Null-ended string pointer
 * @return
 */
uint8_t nina_parse_command(int8_t *strp) {
	uint8_t scan_res;
	int8_t *scan_res_p;
	uint8_t scanned_vals[32];
	uint32_t val, scanned_32;
	int8_t addr[16] = { 0 };
	int8_t tmpstr[64] = { 0 };
	int8_t tmpstr2[64] = { 0 };
	//chprintf(SHELL_IFACE, "%s", strp);
	scan_res = sscanf(strp, "+UBTGSN:%d,%d,%x\r", &scanned_vals[0],
			&scanned_vals[1], &val);
	if (scan_res == 1) {

		return 1;
	}

	scan_res = sscanf(strp, "+UBTGCHA:%d,%d\r", &scanned_vals[0],
			&scanned_vals[1]);
	if (scan_res == 2) {
		charac_temporary->cccd_handle = scanned_vals[0];
		charac_temporary->value_handle = scanned_vals[1];
		charac_temporary->parsed = 1;
	/*	chprintf((BaseSequentialStream*) &SD1,
				"Scanned charac descript %d %d\r\n", scanned_vals[0],
				scanned_vals[1]);*/
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
		nina_parse_notification(scanned_vals[0], scanned_vals[1], scanned_32);
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

	//scan_res = sscanf(strp, "\r\n+UBTD:%12sp,", addr);
#ifdef SD_MODULE_TRAINER
	if (strstr(strp, "+UBTD:") != NULL) {
		nina_compare_founded_dev(strp, &ble_remote_dev_list);
		return;
		scan_res_p = strstr(strp, "D4CA6EB91DD3");
		if (scan_res_p != NULL) {

			chprintf((BaseSequentialStream*) &SD1,
					"Scanned available lag %x\r\n", scan_res_p);
			remote_lag->is_avalible = 1;
			return 1;
		}
		scan_res_p = strstr(strp, "D4CA6EBAFDA0");
		if (scan_res_p != NULL) {
			chprintf((BaseSequentialStream*) &SD1,
					"Scanned available rudder %x\r\n", scan_res_p);
			remote_rudder->is_avalible = 1;
			return 1;
		}
	}

#endif
			scan_res = sscanf(strp, "+UUBTACLC:%d,%d,%12sp\r\n", &scanned_vals[0],
			&scanned_vals[1], addr);
	if (scan_res == 3) {
		chprintf((BaseSequentialStream*) &SD1,
				"Scanned connected to dev %d %d %s\r\n", scanned_vals[0],
				scanned_vals[1], addr);
		nina_register_remote_dev(&ble_remote_dev_list, scanned_vals[0], scanned_vals[1], addr);
		return 1;
	}

	scan_res = sscanf(strp, "+UUBTACLC:%d,%d,%12sr\r\n", &scanned_vals[0],
			&scanned_vals[1], addr);
	//chprintf((BaseSequentialStream*) &SD1, "Scanned %d\r\n", scan_res);
	if (scan_res == 3) {
		chprintf((BaseSequentialStream*) &SD1,
				"Scanned connected dev %d %d %s\r\n", scanned_vals[0],
				scanned_vals[1], addr);
		nina_register_peer(scanned_vals[0], scanned_vals[1], addr);
		return 1;
	}

	scan_res = sscanf(strp, "+UUBTACLD:%d\r", &scanned_vals[0]);
	if (scan_res == 1) {
		nina_unregister_peer(&ble_remote_dev_list, scanned_vals[0]);
		return 1;
	}

	scan_res = sscanf(strp, "+UUBTB:%xr,%d\r", &addr, &scanned_vals[0]);
	if (scan_res == 2) {

		return 1;
	}
	if (strstr(strp, "OK") != NULL) {

		return 1;
	}
	if (strstr(strp, "ERROR") != NULL) {
		return 1;
	}
	return -1;
}

/**
 * @brief Parsing notification
 * @param conn_handle
 * @param val_handle
 * @param value
 */
void nina_parse_notification(uint8_t conn_handle, uint8_t val_handle, uint32_t value){
#ifdef SD_MODULE_TRAINER
	if ((remote_lag->is_connected == 1) && (remote_lag->conn_handle == conn_handle)){
#ifdef RAW_BLE_SENSOR_DATA
		r_lag->meters = (float)((value >> 8) & 0x0000FFFF);
		//r_lag->meters =
		bs->value = value;
#else
		r_lag->meters = (float)(value >> 8) + ((float)(value & 0xFF) / 100.0));
		bs->value = value;
#endif
		//chprintf((BaseSequentialStream*) &SD1, "R lag %f\r\n", r_lag->meters);
	}else if ((remote_rudder->is_connected == 1) && (remote_rudder->conn_handle == conn_handle)){
#ifdef RAW_BLE_SENSOR_DATA
		r_rudder->native = (float)((value >> 8) & 0x0000FFFF);
		r_rudder->degrees = get_polynom_degrees(r_rudder->native, r_rudder_coefs);

		//rdr->value = value;
		//in value we have native data - need to send degrees
		rdr->value = convert_to_ble_type(r_rudder->degrees);
#else
		r_rudder->degrees = (float)((int16_t)((value >> 8) & 0x0000FFFF) + ((float)(value & 0x000000FF) / 100.0));
		rdr->value = value;
#endif
		//	chSemWait(&usart1_semaph);
	//	chprintf((BaseSequentialStream*) &SD1, "R rdr %f\r\n", r_rudder->degrees);
	//	chSemSignal(&usart1_semaph);
	}
#endif
}

/**
 * @brief Connecting request to remote device
 * @param addr Device address string pointer
 * @param type Connection type (0 - default)
 */
void nina_connect(uint8_t *addr, uint8_t type){
	chprintf(NINA_IFACE, "AT+UBTACLC=%s,%d\r", addr, type);
}

/**
 * @brief Register new peer connection
 * @param conn_handle Peer connection handle
 * @param type Connection type
 * @param addr Peer's address
 */
void nina_register_peer(uint8_t conn_handle, uint8_t type, int8_t *addr){
	peer->conn_handle = conn_handle;
	peer->type = type;
	memcpy(peer->addr, addr, 12);
	peer->is_connected = 1;
	chprintf((BaseSequentialStream*) &SD1, "Connected peer %d %d %s\r\n", peer->conn_handle, peer->type, peer->addr);
#ifdef SD_MODULE_TRAINER
	output->ble = OUTPUT_BLE;
#endif
#ifdef SD_SENSOR_BOX_RUDDER
	output->ble = OUTPUT_RUDDER_BLE;
#endif
#ifdef SD_SENSOR_BOX_LAG
	output->ble = OUTPUT_LAG_BLE;
#endif

}

/**
 * @brief Process peer disconnection event
 * @param devlist Device list struct pointer
 * @param conn_handle Connection handler
 */
void nina_unregister_peer(ble_remote_dev_t* devlist, uint8_t conn_handle) {
#ifdef SD_MODULE_TRAINER

	uint8_t i = 0;
	//int8_t scan_res_p;
	for (i = 0; i < NUM_OF_REMOTE_DEV; i++) {
		//scan_res_p = strcmp(addr, devlist[i].addr);
		if (devlist[i].charac.conn_handle == conn_handle) {
			chprintf((BaseSequentialStream*) &SD1,
								"Disconnected from %s with conn_handle %d, conn_type %d (addr: %s)\r\n",
								devlist[i].ascii_name, devlist[i].charac.conn_handle,
								devlist[i].conn_type, devlist[i].addr);
			devlist[i].connected = 0;
			devlist[i].charac.conn_handle = 99;
			devlist[i].conn_type = 0;
			//nina_get_remote_characs(devlist[i].charac.conn_handle, 0x4A01);	//TODO: uuid not used, should be removed
			return;
		}
	}

	chprintf((BaseSequentialStream*) &SD1, "Disconnected peer 1 %d %d %s\r\n",
			peer->conn_handle, peer->type, peer->addr);
	peer->is_connected = 0;
	peer->conn_handle = 0;
	peer->type = 0;
	memset(peer->addr, 0, 12);
	return;

#else
	chprintf((BaseSequentialStream*) &SD1, "Disconnected peer 1 %d %d %s\r\n",
				peer->conn_handle, peer->type, peer->addr);
		peer->is_connected = 0;
		peer->conn_handle = 0;
		peer->type = 0;
		memset(peer->addr, 0, 12);
		return;
#endif
}

/**
 * @brief Register new connection
 * @param devlist Device list struct pointer
 * @param conn_handle Connection handler
 * @param type Connection type
 * @param addr Device address
 */
void nina_register_remote_dev(ble_remote_dev_t* devlist, uint8_t conn_handle, uint8_t type, int8_t *addr){
#ifdef SD_MODULE_TRAINER

	uint8_t i = 0;
	int8_t scan_res_p;
	for (i = 0; i < NUM_OF_REMOTE_DEV; i++) {
		scan_res_p = strcmp(addr, devlist[i].addr);
		if (scan_res_p == 0){
				devlist[i].connected = 1;
				devlist[i].charac.conn_handle = conn_handle;
				devlist[i].conn_type = type;
				chprintf((BaseSequentialStream*) &SD1, "Connected to %s with conn_handle %d, conn_type %d (addr: %s)\r\n", devlist[i].ascii_name, devlist[i].charac.conn_handle, devlist[i].conn_type, devlist[i].addr);
				nina_get_remote_characs(devlist[i].charac.conn_handle, 0x4A01);	//TODO: uuid not used, should be removed
				return;
		}
	}
	nina_register_peer(conn_handle, type, addr);
			output->ble = OUTPUT_BLE;
		//	toggle_test_output();
			return;

#else
	nina_register_peer(conn_handle, type, addr);
#endif
}

/**
 * @brief Request remote device characteristics
 * @param handle Connection handler
 * @param uuid -
 */
void nina_get_remote_characs(uint16_t handle, uint16_t uuid){
	chprintf(NINA_IFACE, "AT+UBTGDP=%d\r", handle);
	chThdSleepMilliseconds(300);
	chprintf(NINA_IFACE, "AT+UBTGDCS=%d,%d,%d\r", handle, 30, 65535);
	chThdSleepMilliseconds(300);
	chprintf(NINA_IFACE, "AT+UBTGDCD=%d,%d,%d\r", handle, 32, 65535);
	chThdSleepMilliseconds(300);
	chprintf(NINA_IFACE, "AT+UBTGWC=%d,%d,%d\r", handle, 33, 1);
}

/**
 * @brief Allocate memory for BLE structs
 */
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

/**
 *
 */
void nina_init_module(void){
	nina_init_services();
}

/**
 *
 * @param at_command
 * @return
 */
uint8_t nina_wait_response(int8_t *at_command){

return NINA_SUCCESS;
}

/**
 * @brief Create characteristic for device in server mode
 * @param charac Characteristic hex number
 * @param uuid UUID
 * @param properties Charac properties
 * @param sec_read Secure read
 * @param sec_write Secure write
 * @param def_val Default value
 * @param read_auth Reading autorization
 * @param max_len Max lenth
 * @return
 */
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
			//chprintf((BaseSequentialStream*) &SD1, "Handlers %d %d\r\n", charac_temporary->cccd_handle, charac_temporary->value_handle);
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

/**
 * @brief Send notification to client device
 * @param ble_rudder
 * @param val
 */
void nina_notify(ble_charac_t *ble_rudder, int32_t val){
	chprintf(NINA_IFACE, "AT+UBTGSN=%d,%02d,%06x\r", peer->conn_handle, ble_rudder->cccd_handle, val);
}

/**
 *
 * @param charac
 */
void nina_wait_charac_handlers(ble_charac_t *charac){
	uint16_t val_handle;
	uint16_t cccd_handle;
	//parse
	charac->cccd_handle = cccd_handle;
	charac->value_handle = val_handle;
}

#ifdef SD_MODULE_TRAINER
/**
 * @brief Looks for comparsion with device address in string
 * @param strp BLE search result string
 * @param devlist Device list pointer
 * @return
 */
int8_t nina_compare_founded_dev(uint8_t *strp, ble_remote_dev_t *devlist){
	uint8_t i = 0;
	int8_t *scan_res_p;
	for(i = 0; i < NUM_OF_REMOTE_DEV; i++){
		scan_res_p = strstr(strp, devlist[i].addr);
				if (scan_res_p != NULL) {
					devlist[i].avalible = 1;
				}else{
					devlist[i].avalible = 0;
				}
	}
}

/**
 * @brief Load remote device properties to common devlist struct
 * @param devlist Device list struct pointer
 * @return
 */
int8_t nina_init_devices(ble_remote_dev_t *devlist) {

	uint8_t i = 0;
	for (i = 0; i < NUM_OF_REMOTE_DEV; i++) {
		memset(&devlist[i], 0, sizeof(ble_remote_dev_t));
		devlist[i].charac.conn_handle = 99;
	}

	memcpy(devlist[0].addr, BLE_RDR_ADDR, sizeof(devlist[0].addr));
	memcpy(devlist[0].ascii_name, BLE_RDR_ASCII_NAME, sizeof(BLE_RDR_ASCII_NAME));

	memcpy(devlist[1].addr, BLE_LOG_ADDR, sizeof(devlist[1].addr));
	memcpy(devlist[1].ascii_name, BLE_LOG_ASCII_NAME, sizeof(BLE_LOG_ASCII_NAME));

#if NUM_OF_REMOTE_DEV == 6
	memcpy(devlist[2].addr, BLE_TENSO1_ADDR, sizeof(devlist[2].addr));
	memcpy(devlist[2].ascii_name, BLE_TENSO1_ASCII_NAME, sizeof(BLE_TENSO1_ASCII_NAME));

	memcpy(devlist[3].addr, BLE_TENSO2_ADDR, sizeof(devlist[3].addr));
	memcpy(devlist[3].ascii_name, BLE_TENSO2_ASCII_NAME, sizeof(BLE_TENSO2_ASCII_NAME));

	memcpy(devlist[4].addr, BLE_TENSO3_ADDR, sizeof(devlist[4].addr));
	memcpy(devlist[4].ascii_name, BLE_TENSO3_ASCII_NAME, sizeof(BLE_TENSO3_ASCII_NAME));

	memcpy(devlist[5].addr, BLE_TENSO4_ADDR, sizeof(devlist[5].addr));
	memcpy(devlist[5].ascii_name, BLE_TENSO4_ASCII_NAME, sizeof(BLE_TENSO4_ASCII_NAME));
#endif
	return NUM_OF_REMOTE_DEV;
}

/**
 * @brief Init characteristics and settings of NINA module
 * Depends on device tyty define
 * @return
 */
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
	//# THDG - uuid 3A01
	//# Response +UBTGCHA:THDG_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(thdg, 0x3A01, 10, 1, 1, 0x0F00FF, 0, 3);
	/*
	chprintf(NINA_IFACE, "AT+UBTGCHA=3A01,10,1,1,0F00FF,0,3\r");
	if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
		return -1;
	}uint8_t
	*/
	chThdSleepMilliseconds(100);
	//# RDR - uuid 3A02
	//# Response +UBTGCHA:RDR_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(rdr, 0x3A02, 10, 1, 1, 0x0F00FF, 0, 3);
	/*
	chprintf(NINA_IFACE, "AT+UBTGCHA=3A02,10,1,1,0F00FF,0,3\r");
	if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
		return -1;
	}
	*/
	chThdSleepMilliseconds(100);
	//# TWD - uuid 3A03
	//# Response +UBTGCHA:TWD_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(twd, 0x3A03, 10, 1, 1, 0x0F00FF, 0, 3);
	/*
	chprintf(NINA_IFACE, "AT+UBTGCHA=3A03,10,1,1,0F00FF,0,3\r");
	if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
		return -1;
	}
	*/
	chThdSleepMilliseconds(100);
	//# TWS - uuid 3A04
	//# Response +UBTGCHA:TWS_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(tws, 0x3A04, 10, 1, 1, 0x0F00FF, 0, 3);
	/*
	chprintf(NINA_IFACE, "AT+UBTGCHA=3A04,10,1,1,0F00FF,0,3\r");
	if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
		return -1;
	}
	*/
	chThdSleepMilliseconds(100);
	//# TWA - uuid 3A05
	//# Response +UBTGCHA:TWD_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(twa, 0x3A05, 10, 1, 1, 0x0F00FF, 0, 3);
	/*
	chprintf(NINA_IFACE, "AT+UBTGCHA=3A05,10,1,1,0F00FF,0,3\r");
	if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
		return -1;
	}
	*/
	chThdSleepMilliseconds(100);
	//# BS - uuid 3A06
	//# Response +UBTGCHA:TWD_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(bs, 0x3A06, 10, 1, 1, 0x0F00FF, 0, 3);
	/*
	chprintf(NINA_IFACE, "AT+UBTGCHA=3A06,10,1,1,0F00FF,0,3\r");
	if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
		return -1;
	}
	*/
	chThdSleepMilliseconds(100);
	//# TWA_TG - uuid 3A07
	//# Response +UBTGCHA:TWD_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(twa_tg, 0x3A07, 10, 1, 1, 0x0F00FF, 0, 3);
	/*
	chprintf(NINA_IFACE, "AT+UBTGCHA=3A07,10,1,1,0F00FF,0,3\r");
	if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
		return -1;
	}
	*/
	chThdSleepMilliseconds(100);
	//# BS_TG - uuid 3A08
	//# Response +UBTGCHA:TWD_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(bs_tg, 0x3A08, 10, 1, 1, 0x0F00FF, 0, 3);
	/*
	chprintf(NINA_IFACE, "AT+UBTGCHA=3A08,10,1,1,0F00FF,0,3\r");
	if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
		return -1;
	}
	*/
	chThdSleepMilliseconds(100);
	//# HDG - uuid 3A09
	//# Response +UBTGCHA:TWD_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(hdg, 0x3A09, 10, 1, 1, 0x0F00FF, 0, 3);
	/*
	chprintf(NINA_IFACE, "AT+UBTGCHA=3A09,10,1,1,0F00FF,0,3\r");
	if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
		return -1;
	}
	*/
	chThdSleepMilliseconds(100);
	//# HEEL - uuid 3A0A
		//# Response +UBTGCHA:TWD_HAND,CCCD_HAND (zero if not use)
	nina_add_charac(heel, 0x3A0A, 10, 1, 1, 0x0F00FF, 0, 3);
	/*
		chprintf(NINA_IFACE, "AT+UBTGCHA=3A0A,10,1,1,0F00FF,0,3\r");
		if (nina_wait_response("+UBTGCHA\r") != NINA_SUCCESS) {
			return -1;
		}
		*/
		chThdSleepMilliseconds(100);
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

/**
 * @brief Sends all data to peer
 * @param peer
 */
void nina_send_all(ble_peer_t *peer){
#ifdef SD_MODULE_TRAINER
	//if (peer->is_connected == 1) {

		chSemWait(&usart_nina);
		chprintf(NINA_IFACE, "AT+UBTGSN=%d,%d,%06x\r", peer->conn_handle,
				thdg->cccd_handle, thdg->value);
		chSemSignal(&usart_nina);

		chThdSleepMilliseconds(30);

		chSemWait(&usart_nina);
		chprintf(NINA_IFACE, "AT+UBTGSN=%d,%d,%06x\r", peer->conn_handle,
				rdr->cccd_handle, rdr->value);
		chSemSignal(&usart_nina);

		chThdSleepMilliseconds(30);

		chSemWait(&usart_nina);
		chprintf(NINA_IFACE, "AT+UBTGSN=%d,%d,%06x\r", peer->conn_handle,
				twd->cccd_handle, twd->value);
		chSemSignal(&usart_nina);

		chThdSleepMilliseconds(30);

		chSemWait(&usart_nina);
		chprintf(NINA_IFACE, "AT+UBTGSN=%d,%d,%06x\r", peer->conn_handle,
				tws->cccd_handle, tws->value);
		chSemSignal(&usart_nina);

		chThdSleepMilliseconds(30);

		chSemWait(&usart_nina);
		chprintf(NINA_IFACE, "AT+UBTGSN=%d,%d,%06x\r", peer->conn_handle,
				twa->cccd_handle, twa->value);
		chSemSignal(&usart_nina);

		chThdSleepMilliseconds(30);

		chSemWait(&usart_nina);
		chprintf(NINA_IFACE, "AT+UBTGSN=%d,%d,%06x\r", peer->conn_handle,
				bs->cccd_handle, bs->value);
		chSemSignal(&usart_nina);

		chThdSleepMilliseconds(30);

		chSemWait(&usart_nina);
		chprintf(NINA_IFACE, "AT+UBTGSN=%d,%d,%06x\r", peer->conn_handle,
				twa_tg->cccd_handle, twa_tg->value);
		chSemSignal(&usart_nina);

		chThdSleepMilliseconds(30);

		chSemWait(&usart_nina);
		chprintf(NINA_IFACE, "AT+UBTGSN=%d,%d,%06x\r", peer->conn_handle,
				bs_tg->cccd_handle, bs_tg->value);
		chSemSignal(&usart_nina);

		chThdSleepMilliseconds(30);

		chSemWait(&usart_nina);
		chprintf(NINA_IFACE, "AT+UBTGSN=%d,%d,%06x\r", peer->conn_handle,
				hdg->cccd_handle, hdg->value);
		chSemSignal(&usart_nina);

		chThdSleepMilliseconds(30);

		chSemWait(&usart_nina);
		chprintf(NINA_IFACE, "AT+UBTGSN=%d,%d,%06x\r", peer->conn_handle,
				heel->cccd_handle, heel->value);
		chSemSignal(&usart_nina);

	//}
#endif
}

#ifdef SD_SENSOR_BOX_RUDDER
/**
 *
 */
uint8_t nina_init_services(void){
	//chprintf(SHELL_IFACE, "AT+UBTLE=2\r");
	chprintf(NINA_IFACE, "AT+UBTLE=2\r");
	if (nina_wait_response("+UBTLE\r") != NINA_SUCCESS) {
		return -1;
	}
/*	chprintf(NINA_IFACE, "AT+UMRS=115200,2,8,1,1,1\r");
		if (nina_wait_response("+UMRS\r") != NINA_SUCCESS) {
			return -1;
		}*/
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

//	chThdSleepMilliseconds(200);
//	chprintf(NINA_IFACE, "AT+UBTCFG=4,8\r");

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
/**
 *
 */
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

#ifdef SD_SENSOR_BOX_TENSO
/**
 *
 */
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
	chprintf(NINA_IFACE, "AT+UBTLN=FastSkipper-TENSO\r");
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
	//# TENSO - uuid 3A01
	//# Response +UBTGCHA:THDG_HAND,CCCD_HAND (zero if not use)

	//TODO:
	//nina_add_charac(ble_lag, 0x3A01, 10, 1, 1, 0x0F00FF, 0, 3);

	/*	chprintf(NINA_IFACE, "AT+UBTGCHA=3A01,10,1,1,0F00FF,0,3\r");
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
