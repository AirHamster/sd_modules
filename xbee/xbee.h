/*
 * xbee.h
 *
 *  Created on: Mar 23, 2019
 *      Author: a-h
 */

#ifndef SD_MODULES_XBEE_XBEE_H_
#define SD_MODULES_XBEE_XBEE_H_

#include <stdint.h>
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

#define XBEE_ATTN_MASK			1


#define XBEE_GET_OWN_ADDR		1
#define XBEE_GET_RSSI			2
#define XBEE_GET_PACKET_PAYLOAD	3
#define XBEE_GET_STAT			4
#define XBEE_GET_PING			5
#define XBEE_GET_CHANNELS		6

#define XBEE_AT_FRAME				0x08
#define XBEE_AT_QUEUE_FRAME			0x09
#define XBEE_TRANSMIT_REQ_FRAME		0x10
#define XBEE_EXPLICIT_ADDR_FRAME	0x11
#define XBEE_REMOTE_AT_FRAME		0x17
#define XBEE_AT_RESPONSE_FRAME		0x88
#define XBEE_MODEM_STAT_FRAME		0x8A
#define XBEE_TRANSMIT_STAT_FRAME	0x8B
#define XBEE_ROUTE_INF_FRAME		0x8D
#define XBEE_AGGREGATE_ADDR_FRAME	0x8E
#define XBEE_RECEIVE_PACKET_FRAME	0x90
#define XBEE_EXPLICIT_RX_FRAME		0x91
#define XBEE_DATA_SAMPLE_FRAME		0x92
#define XBEE_NODE_ID_FRAME			0x95
#define XBEE_REMOTE_RESPONSE_FRAME	0x97

#define XBEE_HEADER_LEN				3

#define OUTPUT_USART	1
#define OUTPUT_XBEE		2

#define SIZE_OF_XBEE_ADDR	8
#define RF_PACK_LEN		128

#define RF_GPS_PACKET	1

#define RF_BOUY_PACKET	2
#define RF_SPORTSMAN_PACKET	3

#define NUM_OF_SPORTSMAN_DEVICES	10
#define NUM_OF_BOUY_DEVICES			4

#define DEV_TYPE_SPORTSMAN		1
#define DEV_TYPE_BOUY			2
#define DEV_TYPE_TRAINER		3

// Diagnostic commands

#define AT_BC				"BC"	// Bytes Transmitted
#define AT_DB				"DB"	// Last Packet RSSI
#define AT_ER				"ER"	// Received Error Count
#define AT_GD				"GD"	// Good Packets Received
#define AT_EA				"EA"	// MAC ACK Timeouts
#define AT_TR				"TR"	// Transmission Errors
#define AT_UA				"UA"	// MAC Unicast Transmission Count
#define AT_H				"%H"	// MAC Unicast One Hop Time
#define AT_8				"%8"

// Network commands

#define AT_CE				"CE"	// Node Messaging Options
#define AT_BH				"BH"	// Broadcast Hops
#define AT_NH				"NH"	// Network Hops
#define AT_NN				"NN"	// Network Delay Slots
#define AT_MR				"MR"	// Mesh Unicast Retries

// Addressing commands

#define AT_SH				"SH"	// Serial Number High
#define AT_SL				"SL"	// Serial Number Low
#define AT_DH				"DH"	// Destination Address High
#define AT_DL				"TL"	// Destination Address Low
#define AT_TO				"TO"	// Transmit Options
#define AT_NI				"NI"	// Node Identifier
#define AT_NT				"NT"	// Node Discover Timeout
#define AT_NO				"NO"	// Node Discovery Options
#define AT_CI				"CI"	// Cluster ID
#define AT_DE				"DE"	// Destination Endpoint
#define AT_SE				"SE"	// Source Endpoint

// Addressing discovery/configuration commands

#define AT_AG				"AG"	// Aggregator Support
#define AT_DN				"DN"	// Discover Node
#define AT_ND				"ND"	// Network Discover
#define AT_FN				"FN"	// Find Neighbors

// Diagnostic - addressing commands

#define AT_N				"N?"	// Network Discovery Timeout

// Security commands

#define AT_EE				"EE"	// Security Enable
#define AT_KY				"KY"	// AES Encryption Key

// Serial interfacing commands

#define AT_BD				"BD"	// Interface Data Rate
#define AT_NB				"NB"	// Parity
#define AT_SB				"SB"	// Stop Bits
#define AT_RO				"RO"	// Packetization Timeout
#define AT_FT				"FT"	// Flow Control Threshold
#define AT_AP				"AP"	// API Mode
#define AT_AO				"AO"	// API Options

// Sleep commands

#define AT_SM				"SM"	// Sleep Mode
#define AT_SO				"SO"	// Sleep Options
#define AT_SN				"SN"	// Number of Sleep Periods
#define AT_SP				"SP"	// Sleep Period
#define AT_ST				"ST"	// Wake Time
#define AT_WH				"WH"	// Wake Host

// Diagnostic - sleep status/timing commands

#define AT_SS				"SS"	// Sleep Status
#define AT_OS				"OS"	// Operating Sleep Time
#define AT_OW				"OW"	// Operating Wake Time
#define AT_MS				"MS"	// Missed Sync Messages
#define AT_SQ				"SQ"	// Missed Sleep Sync Count

// Command mode options

#define AT_CC				"CC"	// Command Sequence Character
#define AT_CT				"CT"	// Command Mode Timeout
#define AT_CN				"CN"	// Exit Command Mode
#define AT_GT				"GT"	// Guard Times

// Firmware commands

#define AT_VL				"VL"	// Version Long
#define AT_VR				"VR"	// Firmware Version
#define AT_HV				"HV"	// Hardware Version
#define AT_HS				"HS"	// Hardware Series
#define AT_DD				"DD"	// Device Type Identifier
#define AT_NP				"NP"	// Maximum Packet Payload Bytes
#define AT_CK				"CK"	// Configuration CRC

typedef struct{
	uint32_t own_addr_h;
	uint32_t own_addr_l;
	uint32_t dest_addr_h;
	uint32_t dest_addr_l;
	uint32_t channels;
	uint16_t packet_payload;
	uint16_t rssi;
	uint16_t bytes_transmitted;
	uint16_t rec_err_count;
	uint16_t good_packs_res;
	uint16_t trans_errs;
	uint16_t unicast_trans_count;
	uint8_t suspend_state;
	uint8_t poll_suspend_state;
	uint8_t loopback_mode;
	uint8_t tx_ready;
}xbee_struct_t;

typedef struct{
	int32_t lat;
	int32_t lon;
	int32_t headMot;
	int32_t headVeh;
	int32_t yaw;
	float pitch;
	float roll;
	float speed;
	uint16_t dist;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t sat;
	uint8_t bat;
}tx_box_t;

typedef struct {
	int32_t lat;
	int32_t lon;
	int32_t headMot;
	int32_t headVeh;
	int32_t yaw;
	float pitch;
	float roll;
	float speed;
	float rdr;
	uint16_t log;
	uint16_t tenso_1;
	uint16_t tenso_2;
	uint16_t tenso_3;
	uint16_t tenso_4;
	uint16_t dist;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t sat;
	uint8_t bat;
} xbee_sportsman_data_t;

typedef struct {
	int32_t lat;
	int32_t lon;
	int32_t headMot;
	int32_t headVeh;
	int32_t yaw;
	float pitch;
	float roll;
	float speed;
	float rdr;
	float wind_speed;
	uint16_t log;
	uint16_t tenso_1;
	uint16_t tenso_2;
	uint16_t tenso_3;
	uint16_t tenso_4;
	uint16_t dist;
	uint16_t wind_direction;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t sat;
	uint8_t bat;
} xbee_trainer_data_t;

typedef struct {
	int32_t lat;
	int32_t lon;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t sat;
	uint8_t bat;
} xbee_bouy_data_t;

typedef struct {
	uint8_t addr[8];	//64 bit addressing
	uint8_t type;		//sportsmen or bouy
	uint8_t is_connected;
	uint8_t heartbit;	//decrease every second, if == 0 - dev not availible, update to 10 if new data has come
	int8_t rssi;
	int8_t number;
	void *rf_data;

} xbee_remote_dev_t;



void xbee_read(SPIDriver *SPID, uint8_t rxlen, uint8_t *at_msg, uint8_t *rxbuff);
void xbee_write(BaseSequentialStream* chp, int argc, char* argv[]);
void xbee_send(SPIDriver *SPID, uint8_t *txbuf, uint8_t len);
void xbee_read_no_cs(SPIDriver *SPID, uint8_t len, uint8_t *rxbuff);
void xbee_receive(SPIDriver *SPID, uint8_t len, uint8_t *rxbuf);
void xbee_attn(BaseSequentialStream* chp, int argc, char* argv[]);
void xbee_attn_event(void);

uint8_t xbee_create_at_read_message(uint8_t *at, uint8_t *buffer);
//uint8_t xbee_create_data_write_message(uint8_t *buffer, uint8_t *data, uint8_t num);
uint8_t xbee_create_data_write_message(uint8_t *buffer, void *data, uint8_t packet_type);
uint8_t xbee_calc_CRC(uint8_t *buffer, uint8_t num);
uint8_t xbee_check_attn(void);

void xbee_get_rssi(void);
void xbee_get_stat(void);
void xbee_get_addr(void);
void xbee_get_ping(void);
void xbee_read_own_addr(xbee_struct_t *str);
void xbee_set_loopback(char* argv[]);
void xbee_get_lb_status(void);
void xbee_thread_execute(uint8_t command);

void xbee_process_at_frame(uint8_t* buffer);
void xbee_process_at_queue_frame(uint8_t* buffer);
void xbee_process_tx_req_frame(uint8_t* buffer);
void xbee_process_explicit_frame(uint8_t* buffer);
void xbee_process_remote_at_frame(uint8_t* buffer);
void xbee_process_at_response(uint8_t* buffer);
void xbee_process_modem_stat_frame(uint8_t* buffer);
void xbee_process_tx_stat(uint8_t* buffer);
void xbee_process_route_inf_frame(uint8_t* buffer);
void xbee_process_aggregade_addr_frame(uint8_t* buffer);
void xbee_process_receive_packet_frame(uint8_t* buffer);
void xbee_process_explicit_rx_frame(uint8_t* buffer);
void xbee_process_data_sample_frame(uint8_t* buffer);
void xbee_process_node_id_frame(uint8_t* buffer);
void xbee_process_remote_response_frame(uint8_t* buffer);

void xbee_send_rf_message(void *packet, uint8_t packet_type);
//void xbee_send_rf_message(xbee_struct_t *xbee_strc, uint8_t *buffer, uint8_t len);
void xbee_send_rf_message_back(xbee_struct_t *xbee_strc, uint8_t *buffer, uint8_t len);
void xbee_parse_rf_packet(uint8_t *rxbuff);
void xbee_parse_gps_packet_back(uint8_t *rxbuff);
void xbee_parse_gps_packet(uint8_t *rxbuff);
uint16_t xbee_read_last_rssi(xbee_struct_t *xbee_str);
uint32_t xbee_read_channels(xbee_struct_t *xbee_str);
uint16_t xbee_read_baudrate(xbee_struct_t *xbee_str);
uint16_t xbee_get_attn_pin_cfg(xbee_struct_t *xbee_str);
uint16_t xbee_get_packet_payload(xbee_struct_t *xbee_str);
uint16_t xbee_get_bytes_transmitted(xbee_struct_t *xbee_str);
uint16_t xbee_get_good_packets_res(xbee_struct_t *xbee_str);
uint16_t xbee_get_received_err_count(xbee_struct_t *xbee_str);
uint16_t xbee_get_transceived_err_count(xbee_struct_t *xbee_str);
uint16_t xbee_get_unicast_trans_count(xbee_struct_t *xbee_str);
void xbee_send_ping_message(xbee_struct_t *xbee_strc);
void start_xbee_module(void);
void xbee_polling(void);
void xbee_set_10kbs_rate(void);
void xbee_set_80kbs_rate(void);
#endif /* SD_MODULES_XBEE_XBEE_H_ */
