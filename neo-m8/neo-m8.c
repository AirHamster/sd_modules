/*
 * neo-m8.c
 *
 *  Created on: Mar 19, 2019
 *      Author: a-h
 */

#include "neo-m8.h"

extern struct ch_semaphore usart1_semaph;
extern const SPIConfig neo_spi_cfg;
extern struct ch_semaphore spi2_semaph;
ubx_nav_pvt_t pvt;
ubx_nav_pvt_t *pvt_box = &pvt;

ubx_cfg_nav5_t nav5;
ubx_cfg_nav5_t *nav5_box = &nav5;

ubx_cfg_pm2 pm2;
ubx_cfg_pm2 *pm2_box = &pm2;

neo_struct_t neo_struct;
neo_struct_t *neo = &neo_struct;

ubx_cfg_odo_t odo_struct;
ubx_cfg_odo_t *odo_box = &odo_struct;

ubx_cfg_rate_t rate_struct;
ubx_cfg_rate_t *rate_box = &rate_struct;

void neo_write(SPIDriver *SPID, uint8_t *txbuff, uint8_t len) {

	chSemWait(&spi2_semaph);
	spiAcquireBus(SPID);              /* Acquire ownership of the bus.    */
	//spiStart(&SPID2, &neo_spi_cfg);
	palClearLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
	spiSend(SPID, len, txbuff); /* send request       */
	palSetLine(LINE_NEO_CS);
	spiReleaseBus(SPID); /* Ownership release.               */
	chSemSignal(&spi2_semaph);
	chThdSleepMilliseconds(1);
}

void neo_read_bytes(SPIDriver *SPID, uint16_t num, uint8_t *rxbuf) {
	uint8_t *txbuf[num];
	memset(txbuf, 0xFF, num);
	chSemWait(&spi2_semaph);
	spiAcquireBus(SPID);              /* Acquire ownership of the bus.    */
	//spiStart(&SPID2, &neo_spi_cfg);
	palClearLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
	spiExchange(SPID, num, txbuf, rxbuf); /* Atomic transfer operations.      */
	palSetLine(LINE_NEO_CS);
	spiReleaseBus(SPID); /* Ownership release.               */
	chSemSignal(&spi2_semaph);
	chThdSleepMilliseconds(1);
}

void neo_read_bytes_no_cs(SPIDriver *SPID, uint16_t num, uint8_t *rxbuf) {
	uint8_t *txbuf[num];
	memset(txbuf, 0xFF, num);
	spiAcquireBus(SPID);              /* Acquire ownership of the bus.    */
	if (palReadLine(LINE_NEO_CS)){
		chThdSleepMilliseconds(1);
	}
	//spiStart(&SPID2, &neo_spi_cfg);
	palClearLine(LINE_NEO_CS);
	spiExchange(SPID, num, txbuf, rxbuf); /* Atomic transfer operations.      */
	spiReleaseBus(SPID); /* Ownership release.               */
	//palSetLine(LINE_NEO_CS);
	//chThdSleepMilliseconds(1);

}

void neo_read_bytes_release_cs(SPIDriver *SPID, uint16_t num, uint8_t *rxbuf) {
	uint8_t *txbuf[num];
	memset(txbuf, 0xFF, num);
	spiAcquireBus(SPID);              /* Acquire ownership of the bus.    */
	//spiStart(&SPID2, &neo_spi_cfg);
	palClearLine(LINE_NEO_CS);
	spiExchange(SPID, num, txbuf, rxbuf); /* Atomic transfer operations.      */
	palSetLine(LINE_NEO_CS);
	spiReleaseBus(SPID); /* Ownership release.               */
	chThdSleepMilliseconds(1);

}

void neo_create_poll_request(uint8_t class, uint8_t id){
	uint8_t len = UBX_HEADER_LEN + CRC_LEN;
	uint8_t packet[len];
	uint16_t crc;
	memset(packet, 0, len);
	neo_apply_header(packet, UBX_HEADER);
	neo_apply_class(packet, class);
	neo_apply_id(packet, id);
	neo_apply_length(packet, 0);		// Len is 0 because this is poll request
	crc = neo_calc_crc(packet, len);
	packet[len-2] = crc >> 8;
	packet[len-1] = crc & 0xFF;
	neo_write(&SPID2, packet, len);
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

void neo_write_struct(uint8_t *strc, uint8_t class, uint8_t id, uint8_t payload_len){
	uint8_t len = UBX_HEADER_LEN + CRC_LEN + payload_len;
	uint8_t packet[len];
	uint16_t crc;
	memset(packet, 0, len);
	neo_apply_header(packet, UBX_HEADER);
	neo_apply_class(packet, class);
	neo_apply_id(packet, id);
	neo_apply_length(packet, payload_len);
	memcpy(&packet[UBX_HEADER_LEN], strc, payload_len);
	crc = neo_calc_crc(packet, len);
	packet[len-2] = crc >> 8;
	packet[len-1] = crc & 0xFF;
	neo_write(&SPID2, packet, UBX_HEADER_LEN + CRC_LEN + len);
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

	/*int i;
	chprintf((BaseSequentialStream*)&SD1, "PVT_ON: \n\r");
		for (i = 0; i< len; i++){
			chprintf((BaseSequentialStream*)&SD1, "%x ", packet[i]);
		}
		chprintf((BaseSequentialStream*)&SD1, "\n\r");*/
	neo_write(&SPID2, packet, len);
}

void neo_process_nav(uint8_t *message){
	switch (message[3]){
	case UBX_NAV_PVT_ID:
		neo_process_pvt(message);
		break;
	case UBX_NAV_ODO_ID:
		neo_process_odo(message);
		break;
	case UBX_CFG_NAV5_ID:
		neo_process_nav5(message);
	}
}

void neo_process_odo(uint8_t *message){
	const uint16_t pack_len = (UBX_NAV_ODO_LEN + UBX_HEADER_LEN + CRC_LEN);
	uint8_t odo_message[pack_len];
	uint16_t crc;
	neo_read_bytes_release_cs(&SPID2, pack_len - UBX_HEADER_LEN, &odo_message[UBX_HEADER_LEN]);
	chSemSignal(&spi2_semaph);
	memcpy(odo_message, message, UBX_HEADER_LEN);
	/*chprintf((BaseSequentialStream*)&SD1, "SPI2: ");
				    			    for (j = 0; j < 100; j++){
					    			    	chprintf((BaseSequentialStream*)&SD1, "%x ", pvt_message[j]);
					    			    }
					    			    chprintf((BaseSequentialStream*)&SD1, "\n\r");
	 */		crc = ((odo_message[pack_len-2] << 8) | (odo_message[pack_len-1]));
	 if (crc == neo_calc_crc(odo_message, pack_len)){
		 neo_cp_to_struct(odo_message, (uint8_t*)odo_box, UBX_NAV_ODO_LEN);
		 //memcpy(odo_box, odo_message, UBX_NAV_ODO_LEN);
		 //chprintf((BaseSequentialStream*)&SD1, "CRC is the same \n\r");
	 }else{
		 chSemWait(&usart1_semaph);
		 chprintf((BaseSequentialStream*)&SD1, "CRC fault: %x vs %x \n\r", crc, neo_calc_crc(odo_message, pack_len));
		 chSemSignal(&usart1_semaph);
	 }
}

void neo_process_nav5(uint8_t *message){
	const uint16_t pack_len = (UBX_CFG_NAV5_LEN + UBX_HEADER_LEN + CRC_LEN);
	uint8_t nav5_message[pack_len];
	uint16_t crc;
	neo_read_bytes_release_cs(&SPID2, pack_len - UBX_HEADER_LEN, &nav5_message[UBX_HEADER_LEN]);
	chSemSignal(&spi2_semaph);
	memcpy(nav5_message, message, UBX_HEADER_LEN);
	/*chprintf((BaseSequentialStream*)&SD1, "SPI2: ");
						    			    for (j = 0; j < 100; j++){
							    			    	chprintf((BaseSequentialStream*)&SD1, "%x ", pvt_message[j]);
							    			    }
							    			    chprintf((BaseSequentialStream*)&SD1, "\n\r");
	 */
	crc = ((nav5_message[pack_len-2] << 8) | (nav5_message[pack_len-1]));
	if (crc == neo_calc_crc(nav5_message, pack_len)){
		neo_cp_to_struct(nav5_message, (uint8_t*)nav5_box, UBX_CFG_NAV5_LEN);
		chSemWait(&usart1_semaph);
			chprintf((BaseSequentialStream*)&SD1, "CFG_NAV5: stat_hold_dist %d, stat_hold_thresh %d, dynModel %d \n\r",
											nav5_box->staticHoldMaxDist, nav5_box->staticHoldThresh, nav5_box->dynModel);
			chSemSignal(&usart1_semaph);
		//memcpy(nav5_box, nav5_message, UBX_CFG_NAV5_LEN);
		//chprintf((BaseSequentialStream*)&SD1, "CRC is the same \n\r");
	}else{
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*)&SD1, "CRC fault: %x vs %x \n\r", crc, neo_calc_crc(nav5_message, pack_len));
		chSemSignal(&usart1_semaph);
	}
}

void neo_process_rate(uint8_t *message){
	const uint16_t pack_len = (UBX_CFG_RATE_LEN + UBX_HEADER_LEN + CRC_LEN);
	uint8_t rate_message[pack_len];
	uint16_t crc;
	neo_read_bytes_release_cs(&SPID2, pack_len - UBX_HEADER_LEN, &rate_message[UBX_HEADER_LEN]);
	chSemSignal(&spi2_semaph);
	memcpy(rate_message, message, UBX_HEADER_LEN);
	/*chprintf((BaseSequentialStream*)&SD1, "SPI2: ");
	   			    for (j = 0; j < 100; j++){
	    			    	chprintf((BaseSequentialStream*)&SD1, "%x ", pvt_message[j]);
	    			    }
	 			    chprintf((BaseSequentialStream*)&SD1, "\n\r");
	 */
	crc = ((rate_message[pack_len-2] << 8) | (rate_message[pack_len-1]));
	if (crc == neo_calc_crc(rate_message, pack_len)){
		neo_cp_to_struct(rate_message, (uint8_t*)rate_box, UBX_CFG_RATE_LEN);
		//memcpy(rate_box, rate_message, UBX_CFG_RATE_LEN);
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*)&SD1, "CFG_RATE: meas = %d nav = %d, time = %d \n\r", rate_box->measRate, rate_box->navRate, rate_box->timeRef);
		chSemSignal(&usart1_semaph);
	}else{
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*)&SD1, "CRC fault: %x vs %x \n\r", crc, neo_calc_crc(rate_message, pack_len));
		chSemSignal(&usart1_semaph);
	}
}
void neo_process_pvt(uint8_t *message){
	const uint16_t pack_len = (UBX_NAV_PVT_LEN + UBX_HEADER_LEN + CRC_LEN);
	uint8_t pvt_message[pack_len];
	uint16_t crc;
	neo_read_bytes_release_cs(&SPID2, pack_len - UBX_HEADER_LEN, &pvt_message[UBX_HEADER_LEN]);
	chSemSignal(&spi2_semaph);
	memcpy(pvt_message, message, UBX_HEADER_LEN);
	/*chSemWait(&usart1_semaph);
	int8_t j;
	chprintf((BaseSequentialStream*)&SD1, "SPI2: ");
					    			    for (j = 0; j < pack_len; j++){
						    			    	chprintf((BaseSequentialStream*)&SD1, "%x ", pvt_message[j]);
						    			    }
						    			    chprintf((BaseSequentialStream*)&SD1, "\n\r");
	chSemSignal(&usart1_semaph); */
	crc = ((pvt_message[pack_len-2] << 8) | (pvt_message[pack_len-1]));
	if (crc == neo_calc_crc(pvt_message, pack_len)){
		neo_cp_to_struct(pvt_message, (uint8_t*)pvt_box, UBX_NAV_PVT_LEN);
		//chprintf((BaseSequentialStream*)&SD1, "CRC is the same \n\r");
	}else{
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*)&SD1, "CRC fault: %x vs %x \n\r", crc, neo_calc_crc(pvt_message, pack_len));
		chSemSignal(&usart1_semaph);
	}
}

void neo_process_ack(uint8_t *message){
	uint8_t id = message[3];
	uint8_t ack_payload[10];
	uint16_t crc;
	neo_read_bytes_release_cs(&SPID2, 6, &ack_payload[6]);
	chSemSignal(&spi2_semaph);
	memcpy(&ack_payload[0], &message[0], 6);
	crc = neo_calc_crc(ack_payload, 10);

	/*uint8_t i;
	chprintf((BaseSequentialStream*)&SD1, "ACK: ");
	for (i = 0; i < 10; i++){
		chprintf((BaseSequentialStream*)&SD1, "%x ", ack_payload[i]);
	}
	chprintf((BaseSequentialStream*)&SD1, "\n\r");
*/
	if (id == 0x01){
		if (crc == (ack_payload[8] << 8 | ack_payload[9])){
			chSemWait(&usart1_semaph);
			chprintf((BaseSequentialStream*)&SD1, "NEO ACK message, class: %x ID: %x\n\r", ack_payload[6], ack_payload[7]);
			chSemSignal(&usart1_semaph);
			return;
		}else{
			chSemWait(&usart1_semaph);
			chprintf((BaseSequentialStream*)&SD1, "CRC failed, %x vs %x\n\r", crc, (ack_payload[8] << 8 | ack_payload[9]));
			chSemSignal(&usart1_semaph);
			return;
		}
	}else if (id == 0x00){
		if (crc == (ack_payload[8] << 8 | ack_payload[9])){
			chSemWait(&usart1_semaph);
			chprintf((BaseSequentialStream*)&SD1, "NEO NACK message, class: %x ID: %x\n\r", ack_payload[6], ack_payload[7]);
			chSemSignal(&usart1_semaph);
			return;
		}else{
			chSemWait(&usart1_semaph);
			chprintf((BaseSequentialStream*)&SD1, "CRC failed\n\r");
			chSemSignal(&usart1_semaph);
			return;
		}
	}else{
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*)&SD1, "Unknown ACK ID\n\r");
		chSemSignal(&usart1_semaph);
		return;
	}

}

void neo_process_cfg(uint8_t *message){
	switch (message[3]){
	case UBX_CFG_NAV5_ID:
		neo_process_nav5(message);
		break;
	case UBX_CFG_RATE_ID:
		neo_process_rate(message);
		break;
	}
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
	uint8_t i = 0;
	chSemWait(&spi2_semaph);
	while (i < 50){
		neo_read_bytes_no_cs(&SPID2, 1, &message[0]);
		if ((message[0] == 0xB5)){
			neo_read_bytes_no_cs(&SPID2, 1, &message[1]);
			if (message[1] == 0x62){
				i = 50;
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
				default:
					break;
				}
				return;
			}
		}else if (message[0] == 0xFF){
			i++;
		}
	}
	chSemSignal(&spi2_semaph);
}

void neo_cp_to_struct(uint8_t *msg, uint8_t *strc, uint8_t len){
	memcpy(strc, &msg[6], len);
}


