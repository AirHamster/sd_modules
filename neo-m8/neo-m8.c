/*
 * neo-m8.c
 *
 *  Created on: Mar 19, 2019
 *      Author: a-h
 */

#include "neo-m8.h"

extern nav_pvt_t *pvt_box;

void neo_write(SPIDriver *SPID, uint8_t *txbuff, uint8_t len) {
	spiAcquireBus(SPID);              /* Acquire ownership of the bus.    */
	palClearLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
	spiSend(SPID, len, txbuff); /* send request       */
	spiReleaseBus(SPID); /* Ownership release.               */
	palSetLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
}

void neo_read_bytes(SPIDriver *SPID, uint16_t num, uint8_t *rxbuf) {
	uint8_t *txbuf[num];
	memset(txbuf, 0xFF, num);
	spiAcquireBus(SPID);              /* Acquire ownership of the bus.    */
	palClearLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
	spiExchange(SPID, num, txbuf, rxbuf); /* Atomic transfer operations.      */
	spiReleaseBus(SPID); /* Ownership release.               */
	palSetLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
}

void neo_read_bytes_no_cs(SPIDriver *SPID, uint16_t num, uint8_t *rxbuf) {
	uint8_t *txbuf[num];
	memset(txbuf, 0xFF, num);
	spiAcquireBus(SPID);              /* Acquire ownership of the bus.    */
	palClearLine(LINE_NEO_CS);
	spiExchange(SPID, num, txbuf, rxbuf); /* Atomic transfer operations.      */
	spiReleaseBus(SPID); /* Ownership release.               */
	//palSetLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);

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
	packet[UBX_HEADER_LEN + 0] = 4;
	packet[UBX_HEADER_LEN + 12] = (1 << 0) | (RTCM3_EN << 5);
	packet[UBX_HEADER_LEN + 14] = (1 << 0) | (RTCM3_EN << 5);
	crc = neo_calc_crc(packet, len);
	packet[len-2] = crc >> 8;
	packet[len-1] = crc & 0xFF;
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

uint16_t neo_calc_crc(uint8_t *buffer, uint16_t len){
	uint8_t crc_a = 0;
	uint8_t crc_b = 0;
	uint8_t i;
	for(i = 2; i < len-2; i++)
	{
		crc_a = crc_a + buffer[i];
		crc_b = crc_b + crc_a;
	}
	return ((crc_a << 8) | crc_b);
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
		packet[UBX_HEADER_LEN + 0] = UBX_NAV_CLASS;
		packet[UBX_HEADER_LEN + 1] = UBX_NAV_PVT_ID;
		packet[UBX_HEADER_LEN + 2] = 1;
		crc = neo_calc_crc(packet, len);
		packet[len-2] = crc >> 8;
		packet[len-1] = crc & 0xFF;
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

void neo_process_nav(uint8_t *message){
	switch (message[3]){
	case UBX_NAV_PVT_ID:
		neo_process_pvt(message);
		break;
	}
}

void neo_process_pvt(uint8_t *message){
	const uint16_t pack_len = (UBX_NAV_PVT_LEN + UBX_HEADER_LEN + CRC_LEN);
		uint8_t pvt_message[pack_len];
		uint16_t crc;
		neo_read_bytes(&SPID2, pack_len - UBX_HEADER_LEN, &pvt_message[UBX_HEADER_LEN]);
		memcpy(pvt_message, message, UBX_HEADER_LEN);
				 /*chprintf((BaseSequentialStream*)&SD1, "SPI2: ");
					    			    for (j = 0; j < 100; j++){
						    			    	chprintf((BaseSequentialStream*)&SD1, "%x ", pvt_message[j]);
						    			    }
						    			    chprintf((BaseSequentialStream*)&SD1, "\n\r");
*/
					crc = ((pvt_message[pack_len-2] << 8) | (pvt_message[pack_len-1]));
		    		if (crc == neo_calc_crc(pvt_message, pack_len)){
		    			neo_cp_to_struct(pvt_message, pvt_box, UBX_NAV_PVT_LEN);
		    			//chprintf((BaseSequentialStream*)&SD1, "CRC is the same \n\r");
		    		}else{
		    			chprintf((BaseSequentialStream*)&SD1, "CRC fault: %x vs %x \n\r", crc, neo_calc_crc(pvt_message, pack_len));
   		}
}

void neo_process_ack(uint8_t *message){
	(void)message;
}

void neo_process_cfg(uint8_t *message){
	(void)message;
}

void neo_process_inf(uint8_t *message){
	(void)message;
}

void neo_process_rxm(uint8_t *message){
	(void)message;
}

void neo_process_sec(uint8_t *message){
	(void)message;
}

void neo_process_tim(uint8_t *message){
	(void)message;
}

void neo_process_upd(uint8_t *message){
	(void)message;
}

void neo_process_esf(uint8_t *message){
	(void)message;
}

void neo_process_aid(uint8_t *message){
	(void)message;
}

void neo_process_mon(uint8_t *message){
	(void)message;
}

void neo_process_hnr(uint8_t *message){
	(void)message;
}

void neo_poll(void){
	const uint16_t pack_len = UBX_HEADER_LEN;
	uint8_t message[pack_len];
	uint8_t i;
	chThdSleepMilliseconds(100);
	for (i = 0; i < 30; i++){
		neo_read_bytes_no_cs(&SPID2, 1, &message[0]);
		if ((message[0] == 0xB5)){
			neo_read_bytes_no_cs(&SPID2, 1, &message[1]);
			if (message[1] == 0x62){
				neo_read_bytes_no_cs(&SPID2, 4, &message[2]);
				switch(message[2]){
				case UBX_NAV_CLASS:
					neo_process_nav(message);
					break;
				case UBX_ACK_CLASS:
					neo_process_ack(message);
					break;
				case UBX_CFG_CLASS:
					neo_process_cfg(message);
					break;
				case UBX_INF_CLASS:
					neo_process_inf(message);
					break;
				case UBX_RXM_CLASS:
					neo_process_rxm(message);
					break;
				case UBX_SEC_CLASS:
					neo_process_sec(message);
					break;
				case UBX_TIM_CLASS:
					neo_process_tim(message);
					break;
				case UBX_UPD_CLASS:
					neo_process_upd(message);
					break;
				case UBX_AID_CLASS:
					neo_process_aid(message);
					break;
				case UBX_ESF_CLASS:
					neo_process_esf(message);
					break;
				case UBX_HNR_CLASS:
					neo_process_hnr(message);
					break;
				case UBX_MON_CLASS:
					neo_process_mon(message);
					break;
				}
			}
		}
	}
}
void neo_cp_to_struct(uint8_t *msg, nav_pvt_t *pvt, uint8_t len){
	memcpy(pvt, &msg[6], len);
}


