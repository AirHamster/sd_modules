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
//#include <mem.h>
#include <string.h>

void xbee_read(SPIDriver *SPID, uint8_t rxlen, uint8_t *at_msg, uint8_t *rxbuff);
void xbee_write(BaseSequentialStream* chp, int argc, char* argv[]);
void xbee_send(SPIDriver *SPID, uint8_t len, uint8_t *txbuf);
void xbee_receive(SPIDriver *SPID, uint8_t len, uint8_t *rxbuf);
void xbee_attn(BaseSequentialStream* chp, int argc, char* argv[]);
uint8_t xbee_create_at_read_message(uint8_t *at, uint8_t *buffer);
uint8_t xbee_create_at_write_message(char *at, uint8_t *buffer, uint8_t *data, uint8_t num);
uint8_t xbee_calc_CRC(uint8_t *buffer, uint8_t num);
uint8_t xbee_check_attn(void);

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


#endif /* SD_MODULES_XBEE_XBEE_H_ */