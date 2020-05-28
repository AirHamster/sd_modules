/*
 * nina-b3.h
 *
 *  Created on: Apr 26, 2019
 *      Author: a-h
 */

#ifndef SD_MODULES_NINA_B3_NINA_B3_H_
#define SD_MODULES_NINA_B3_NINA_B3_H_
#include "stdint.h"
#include "ch.h"
#include "hal.h"
#include "config.h"
#define NINA_SUCCESS 1

#define BLE_OBSERVE_EV	0
#define BLE_DATA_TX_EV	1
#define BLE_REMOTE_CFG_EV	2


#define BLE_DEV_TYPE_LOG	1
#define BLE_DEV_TYPE_RDR	2
#define BLE_DEV_TYPE_TENSO	3

#define BLE_POWER_SWITCHED_ON_EV	4
#define BLE_POWER_SWITCHED_OFF_EV	5

#define NUM_OF_REMOTE_DEV	7

#define BLE_RDR_ADDR		"CCF957874132"
#define BLE_LOG_ADDR		"D4CA6EB91DD3"
#define BLE_HEART_ADDR		"EBBEF86274F5"

#define BLE_TENSO1_ADDR		"CCF957874018"
#define BLE_TENSO2_ADDR		"D4CA6EB91D02"
#define BLE_TENSO3_ADDR		"D4CA6EB91D03"
#define BLE_TENSO4_ADDR		"D4CA6EB91D04"


#define BLE_RDR_ASCII_NAME		"REMOTE_RUDDER"
#define BLE_LOG_ASCII_NAME		"REMOTE_LOG"
#define BLE_HEART_ASCII_NAME	"REMOTE_HEART"

#define BLE_TENSO1_ASCII_NAME		"REMOTE_TENSO_1"
#define BLE_TENSO2_ASCII_NAME		"REMOTE_TENSO_2"
#define BLE_TENSO3_ASCII_NAME		"REMOTE_TENSO_3"
#define BLE_TENSO4_ASCII_NAME		"REMOTE_TENSO_4"

// Motor object structure
typedef struct
{
    int16_t currentSpeed;
} Motor;

// Event data structure
typedef struct
{
    int16_t speed;
} MotorData;

typedef struct{

}ble_t;

/**
 * Peer properties
 */
typedef struct{
	int8_t addr[13];
	uint8_t is_connected;
	uint8_t conn_handle;
	uint8_t type;
}ble_peer_t;

/**
 *  Internal characteristic properties
 */
typedef struct{
	uint16_t value_handle;
	uint16_t cccd_handle;
	uint16_t uuid;
	int32_t value;
	uint8_t properties;
	uint8_t security_read;
	uint8_t security_write;
	uint8_t read_auth;
	uint8_t max_lenth;
}ble_charac_t;

/**
 * Remote device characteristic properties
 */
typedef struct{
	uint16_t conn_handle;
	uint16_t attr_handle;
	uint16_t properties;
	uint16_t value_handle;
	uint16_t uuid;
}ble_remote_charac_t;

typedef struct{
	ble_remote_charac_t charac;
	uint8_t is_connected;
	uint8_t is_avalible;
	uint8_t type;
	uint16_t conn_handle;
}ble_remote_t;

/**
 * Remote device connection properties
 */
typedef struct{
	ble_remote_charac_t charac;
	uint8_t connected;
	uint8_t avalible;
	uint8_t conn_type;
	int8_t addr[13];	//zero string terminator in last byte
	int8_t ascii_name[32];
}ble_remote_dev_t;

/**
 * Remote peer connection properties
 */
typedef struct{
	ble_remote_charac_t charac;
	uint8_t connected;
	uint8_t avalible;
	uint8_t dev_type;
	uint8_t conn_type;
	int8_t addr[13];	//zero string terminator in last byte
	int8_t ascii_name[32];
}ble_remote_peer_t;

typedef struct {
	uint16_t value_handle;
	uint16_t cccd_handle;
	uint8_t parsed;
} ble_temp_charac_t;

#define NUM_OF_CHARACTS 10

#ifdef SD_SENSOR_BOX_RUDDER
#define NUM_OF_CHARACTS 1
#endif

#ifdef SD_SENSOR_BOX_LAG
#define NUM_OF_CHARACTS 1
#endif

#ifdef SD_SENSOR_BOX_WIND
#define NUM_OF_CHARACTS 1
#endif

/**
 * Start BLE threads
 */
void start_ble_module(void);
void nina_send_at(void);
/**
 * Init BLE module, depends on device type (trainer, sportsman, sensor box)
 */
void nina_init_module(void);

/**
 * Allocate memory for BLE structs
 */
void nina_fill_memory(void);
/**
 * Send all sensors data to peer device
 * @param peer Peer struct pointer
 */
void nina_send_all(ble_peer_t *peer);
void nina_send_two(void);

/**
 * Catch incoming peer connection
 * @param conn_handle
 * @param type
 * @param addr
 */
void nina_register_peer(uint8_t conn_handle, uint8_t type, int8_t *addr);
//void nina_unregister_peer(uint8_t conn_handle);

/**
 * Unregister disconnected peer
 * @param devlist Remote device list struct pointer
 * @param conn_handle Connection handler
 */
void nina_unregister_peer(ble_remote_dev_t* devlist, uint8_t conn_handle);

/**
 * Send notification to peer
 * @param ble_rudder
 * @param val
 */
void nina_notify(ble_charac_t *ble_rudder, int32_t val);

/**
 * Connect to remote device
 * @param addr Device address
 * @param type Connection type
 */
void nina_connect(uint8_t *addr, uint8_t type);

/**
 * Register connection to remote device
 * @param devlist Remote device list struct pointer
 * @param conn_handle Connection handler
 * @param type Connection type
 * @param addr Remote device address
 */
void nina_register_remote_dev(ble_remote_dev_t* devlist, uint8_t conn_handle, uint8_t type, int8_t *addr);
//void nina_register_remote_dev(uint8_t conn_handle, uint8_t type, int8_t *addr);
/**
 * Unregister connection to remote device
 * @param conn_handle
 */
void nina_unregister_remote_dev(uint8_t conn_handle);

/**
 * Request characteristics from remote dev
 * @param handle
 * @param uuid
 */
void nina_get_remote_characs(uint16_t handle, uint16_t uuid);

/**
 * Parse incoming notification
 * @param conn_handle Connection handler
 * @param val_handle Value handler
 * @param value Value
 */
void nina_parse_notification(uint8_t conn_handle, uint8_t val_handle, uint32_t value);

/**
 * Parse command from shell
 * @param strp String pointer
 * @return
 */
uint8_t nina_parse_command(int8_t *strp);

/**
 * Initialize BLE services, depends on device type
 * @return
 */
uint8_t nina_init_services(void);

/**
 * Copy sensors data to BLE struct, depends on device type
 */
void copy_to_ble(void);


uint8_t nina_wait_response(int8_t *at_command);

/**
 * Fill BLE devise list struct
 * @param devlist Device list struct pointer
 * @return
 */
int8_t nina_init_devices(ble_remote_dev_t *devlist);

/**
 * Compare discovered device address with device list
 * @param strp ASCII address representation
 * @param devlist Device list struct pointer
 * @return
 */
int8_t nina_compare_founded_dev(uint8_t *strp, ble_remote_dev_t *devlist);
#endif /* SD_MODULES_NINA_B3_NINA_B3_H_ */

void nina_get_remote_characs_heart_rate(uint16_t handle, uint16_t uuid);
