/*
 * neo-m8.c
 *
 *  Created on: Mar 19, 2019
 *      Author: a-h
 */

#include "neo-m8.h"

void neo_write_byte(SPIDriver *SPID, uint8_t reg_addr, uint8_t value) {
	uint8_t txbuf[2];
	txbuf[0] = reg_addr;
	txbuf[1] = value;
	spiAcquireBus(SPID);              /* Acquire ownership of the bus.    */
	palSetLine(LINE_GREEN_LED); /* LED ON.                          */
	palClearLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
	spiSend(SPID, 2, txbuf); /* send request       */
	spiReleaseBus(SPID); /* Ownership release.               */
	palSetLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
	palClearLine(LINE_GREEN_LED); /* LED OFF.*/
}

void neo_write(SPIDriver *SPID, uint8_t *txbuff, uint8_t len) {
	spiAcquireBus(SPID);              /* Acquire ownership of the bus.    */
	//palSetLine(LINE_GREEN_LED); /* LED ON.                          */
	palClearLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
	spiSend(SPID, len, txbuff); /* send request       */
	spiReleaseBus(SPID); /* Ownership release.               */
	palSetLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
//	palClearLine(LINE_GREEN_LED); /* LED OFF.*/
}

uint8_t neo_read_byte(SPIDriver *SPID, uint8_t reg_addr) {
	uint8_t value;
	reg_addr |= 0x80;	//0x80 indicates read operation
	//palSetLine(LINE_GREEN_LED); /* LED ON.                          */
	spiAcquireBus(SPID);              /* Acquire ownership of the bus.    */
	palClearLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
	spiSend(SPID, 1, &reg_addr); /* send request       */
	spiReceive(SPID, 1, &value);
	spiReleaseBus(SPID); /* Ownership release.               */
	palSetLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
	//palClearLine(LINE_GREEN_LED); /* LED OFF.*/
	return value;
}

void neo_read_bytes(SPIDriver *SPID, uint16_t num, uint8_t *rxbuf) {
	uint8_t *txbuf[num];
	memset(txbuf, 0xFF, num);
	//palSetLine(LINE_GREEN_LED); /* LED ON.                          */
	spiAcquireBus(SPID);              /* Acquire ownership of the bus.    */
	palClearLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
	spiExchange(SPID, num, txbuf, rxbuf); /* Atomic transfer operations.      */
	//neo_process_packet();
	spiReleaseBus(SPID); /* Ownership release.               */
	palSetLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
	//palClearLine(LINE_GREEN_LED); /* LED OFF.*/
}

/* Brief: function that finds sequence of 50 0xFF bytes
 *
 *	returns offset from first array byte to 0xFF sequence
 *	returns -1 if no 0xFF sequences found
 *	returns -2 if found not full part of 0xFF sequence
 */
int8_t neo_find_ff(uint8_t *buff, uint8_t num){
	uint8_t i;
	uint8_t ff_series = 0;
	for (i = 1; i < num; i++){
		if ((buff[i-1] == buff[i]) && (buff[i] == 0xFF)){
				ff_series++;
				if (ff_series == 49){
					return i - 49;	// Return offset
				}
		}else{
			ff_series = 0;
		}
	}
	if (i != 0)
	{
		return -2;
	}else{
		return -1;
	}
}

void neo_switch_to_ubx(void){
	uint8_t packet[UBX_CFG_PRT_LEN + 8];
	uint8_t len = UBX_CFG_PRT_LEN + 8;
	uint16_t crc;
	memset(packet, 0, UBX_CFG_PRT_LEN + 8);
	neo_apply_header(packet, UBX_HEADER);
	neo_apply_class(packet, UBX_CFG_CLASS);
	neo_apply_id(packet, UBX_CFG_PRT_ID);
	neo_apply_length(packet, UBX_CFG_PRT_LEN);
	packet[UBX_PAYLOAD + 0] = 4;
	packet[UBX_PAYLOAD + 12] = (1 << 0) | (RTCM3_EN << 5);
	packet[UBX_PAYLOAD + 14] = (1 << 0) | (RTCM3_EN << 5);
	crc = neo_calc_crc(packet, len);
	packet[len-2] = crc >> 8;
	packet[len-1] = crc & 0xFF;
	int i;
	chprintf((BaseSequentialStream*)&SD1, "NMEA OFF: \n\r");
	for (i = 0; i< len; i++){
		chprintf((BaseSequentialStream*)&SD1, "%x ", packet[i]);
	}
	chprintf((BaseSequentialStream*)&SD1, "\n\r");
	neo_write(&SPID2, packet, len);
}

void neo_apply_header(uint8_t *buffer, uint16_t header){
	buffer[0] = header >> 8;
	buffer[1] = header & 0xFF;
}
void neo_apply_class(uint8_t *buffer, uint8_t class){
	buffer[2] = class;
}
void neo_apply_id(uint8_t *buffer, uint8_t id){
	buffer[3] = id;
}
// Little-endian
void neo_apply_length(uint8_t *buffer, uint8_t len){
	buffer[4] = len;
	buffer[5] = 0;
}

uint16_t neo_calc_crc(uint8_t *buffer, uint8_t len){
	uint8_t crc_a = 0;
	uint8_t crc_b = 0;
	uint8_t i;
	for(i = 2; i < len-2; i++)
	{
		crc_a = crc_a + buffer[i];
		crc_b = crc_b + crc_a;
	}
	return ((crc_a << 8) | crc_b);
	//buffer[len - 1] = crc_b;
	//buffer[len - 2] = crc_a;

}
void neo_set_pvt_1hz(){
	uint8_t packet[UBX_CFG_MSG_LEN_SINGLE + 8];
		uint8_t len = UBX_CFG_MSG_LEN_SINGLE + 8;
		uint16_t crc;
		memset(packet, 0, UBX_CFG_MSG_LEN_SINGLE + 8);
		neo_apply_header(packet, UBX_HEADER);
		neo_apply_class(packet, UBX_CFG_CLASS);
		neo_apply_id(packet, UBX_CFG_MSG_ID);
		neo_apply_length(packet, UBX_CFG_MSG_LEN_SINGLE);
		packet[UBX_PAYLOAD + 0] = UBX_NAV_CLASS;
		packet[UBX_PAYLOAD + 1] = UBX_NAV_PVT_ID;
		packet[UBX_PAYLOAD + 2] = 1;
		crc = neo_calc_crc(packet, len);
		packet[len-2] = crc >> 8;
		packet[len-1] = crc & 0xFF;
		int i;
		chprintf((BaseSequentialStream*)&SD1, "PVT_ON: \n\r");
		for (i = 0; i< len; i++){
			chprintf((BaseSequentialStream*)&SD1, "%x ", packet[i]);
		}
		chprintf((BaseSequentialStream*)&SD1, "\n\r");
		neo_write(&SPID2, packet, len);
}

void neo_poll_prt(void){
	uint8_t packet[0 + 8];
		uint8_t len = 0 + 8;
		uint16_t crc;
		memset(packet, 0, 0 + 8);
		neo_apply_header(packet, UBX_HEADER);
		neo_apply_class(packet, UBX_CFG_CLASS);
		neo_apply_id(packet, UBX_CFG_PRT_ID);
		neo_apply_length(packet, 0);
		//packet[UBX_PAYLOAD + 0] = UBX_NAV_CLASS;
		//packet[UBX_PAYLOAD + 1] = UBX_NAV_PVT_ID;
		//packet[UBX_PAYLOAD + 2] = 1;
		crc = neo_calc_crc(packet, len);
		packet[len-2] = crc >> 8;
		packet[len-1] = crc & 0xFF;
		int i;
		chprintf((BaseSequentialStream*)&SD1, "PVT_ON: \n\r");
		for (i = 0; i< len; i++){
			chprintf((BaseSequentialStream*)&SD1, "%x ", packet[i]);
		}
		chprintf((BaseSequentialStream*)&SD1, "\n\r");
		neo_write(&SPID2, packet, len);
}
