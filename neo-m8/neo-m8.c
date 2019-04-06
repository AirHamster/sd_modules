/*
 * neo-m8.c
 *
 *  Created on: Mar 19, 2019
 *      Author: a-h
 */

#include "neo-m8.h"

extern nav_pvt_t *pvt_box;
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

void neo_polling(SPIDriver *SPID, uint8_t class, uint8_t id) {
	uint8_t packet[0 + 8];
	uint8_t len = 0 + 8;
	uint16_t crc;
	memset(packet, 0, 0 + 8);
	neo_apply_header(packet, UBX_HEADER);
	neo_apply_class(packet, class);
	neo_apply_id(packet, id);
	neo_apply_length(packet, 0);
	//packet[UBX_PAYLOAD + 0] = UBX_NAV_CLASS;
	//packet[UBX_PAYLOAD + 1] = UBX_NAV_PVT_ID;
	//packet[UBX_PAYLOAD + 2] = 1;
	crc = neo_calc_crc(packet, len);
	packet[len-2] = crc >> 8;
	packet[len-1] = crc & 0xFF;
	int i;
	chprintf((BaseSequentialStream*)&SD1, "POLLING: \n\r");
			for (i = 0; i< len; i++){
				chprintf((BaseSequentialStream*)&SD1, "%x ", packet[i]);
			}
			chprintf((BaseSequentialStream*)&SD1, "\n\r");
	//chprintf((BaseSequentialStream*)&SD1, "PVT_ON: \n\r");
	//for (i = 0; i< len; i++){
	//	chprintf((BaseSequentialStream*)&SD1, "%x ", packet[i]);
	//}
	//chprintf((BaseSequentialStream*)&SD1, "\n\r");
	neo_write(SPID, packet, len);
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
void neo_poll_nav_pvt(void){
	const uint16_t pack_len = (UBX_NAV_PVT_LEN + UBX_PAYLOAD + CRC_LEN);
	uint8_t pvt_message[pack_len*2];
	//uint8_t txbuff[pack_len];
	uint8_t i, j;
	uint16_t crc;
	//memset(txbuff, 0xFF, pack_len);
	//neo_polling(&SPID2, UBX_NAV_CLASS, UBX_NAV_PVT_ID);
	chThdSleepMilliseconds(100);
	neo_read_bytes(&SPID2, pack_len*2, pvt_message);
	//chprintf((BaseSequentialStream*)&SD1, "SPI: ");
		//    			    for (j = i; j < 120; j++){
		  //  			    	chprintf((BaseSequentialStream*)&SD1, "%x ", pvt_message[j]);
		    //			    }
		    	//		    chprintf((BaseSequentialStream*)&SD1, "\n\r");
	//spiExchange(&SPID2, pack_len, txbuff, pvt_message);
	    for (i = 0; i < pack_len*2-1; i++)
	    {
	    	if ((pvt_message[i] == 0xB5) && (pvt_message[i+1] == 0x62)
	    			&& (pvt_message[i+2] == 0x01) && (pvt_message[i+3] == 0x07)){
	    		neo_cp_to_struct(&pvt_message[i], pvt_box);
	    		crc = ((pvt_message[i + pack_len-2] << 8) | (pvt_message[i +pack_len-1]));
	    	//	chprintf((BaseSequentialStream*)&SD1, "i is %d \n\r", i);
	    		if (crc == neo_calc_crc(&pvt_message[i], pack_len)){
	    			chprintf((BaseSequentialStream*)&SD1, "CRC is the same \n\r");
	    			break;
	    		}else{
	    			chprintf((BaseSequentialStream*)&SD1, "CRC fault: %x vs %x \n\r", crc, neo_calc_crc(&pvt_message[i], pack_len));
	    			break;
	    		}
	    	}
	    }
}
void neo_cp_to_struct(uint8_t *msg, nav_pvt_t *pvt){
	memcpy(pvt, &msg[6], 92);
}

int32_t neo_get_lat(nav_pvt_t *pvt){
	return (reverse32(pvt->lat));
}

int32_t neo_get_lon(nav_pvt_t *pvt){
	return (reverse32(pvt->lon));
}

uint8_t neo_get_numsv(nav_pvt_t *pvt){
	return pvt->numSV;
}

uint8_t neo_get_valid(nav_pvt_t *pvt){
	return pvt->valid;
}

int32_t neo_get_ground_speed(nav_pvt_t *pvt){
	return (reverse32(pvt->gSpeed));
}

int16_t reverse16(int16_t x)
{
    x = (x & 0xFF) << 8 | (x & 0xFF00) >>  8;
    return x;
}

int32_t reverse32(int32_t num)
{
//	int32_t b;
	//b = x >> 24;
	//b |= ((x >> 8) & 0x0000FF00);
	//b |= ((x << 8) & 0x00FF0000);
	//b |= ((x >> 24) & 0xFF000000);
    //x = (x & 0x00FF00FF) <<  8 | (x & 0xFF00FF00) >>  8;
    //x = (x & 0x0000FFFF) << 16 | (x & 0xFFFF0000) >> 16;
	//int32_t num = 9;
	int32_t b0,b1,b2,b3,b4,b5,b6,b7;
	int32_t res = 0;

	b0 = (num & 0xf) << 28;
	b1 = (num & 0xf0) << 24;
	b2 = (num & 0xf00) << 20;
	b3 = (num & 0xf000) << 16;
	b4 = (num & 0xf0000) << 12;
	b5 = (num & 0xf00000) << 8;
	b6 = (num & 0xf000000) << 4;
	b7 = (num & 0xf0000000) << 4;

	res = b0 + b1 + b2 + b3 + b4 + b5 + b6 + b7;
    return res;
}
