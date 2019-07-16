/*
 * neo-m8.h
 *
 *  Created on: Mar 19, 2019
 *      Author: a-h
 */

#ifndef SD_MODULES_NEO_M8_NEO_M8_H_
#define SD_MODULES_NEO_M8_NEO_M8_H_

#include <stdlib.h>
#include <string.h>
#include "stdint.h"
#include "math.h"
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"
#include "minmea.h"

#define RTCM3_EN			0

#define UBX_HEADER				0xB562
#define UBX_CFG_CLASS			0x06
#define UBX_ACK_CLASS			0x05
#define UBX_INF_CLASS			0x04
#define UBX_RXM_CLASS			0x02
#define UBX_SEC_CLASS			0x27
#define UBX_TIM_CLASS			0x0D
#define UBX_UPD_CLASS			0x09
#define UBX_AID_CLASS			0x0B
#define UBX_ESF_CLASS			0x10
#define UBX_HNR_CLASS			0x28
#define UBX_MON_CLASS			0x0A
#define UBX_NAV_CLASS			0x01

#define UBX_CFG_PRT_ID			0x00
#define UBX_CFG_MSG_ID			0x01
#define UBX_CFG_RATE_ID			0x08
#define UBX_CFG_ODO_ID			0x1E
#define UBX_CFG_SBAS_ID			0x16
#define UBX_NAV_PVT_ID			0x07
#define UBX_CFG_NAV5_ID			0x24
#define UBX_NAV_ODO_ID			0x09

#define UBX_HEADER_LEN			6
#define CRC_LEN					2
#define UBX_CFG_MSG_LEN_SINGLE	3
#define UBX_CFG_PRT_LEN			20
#define UBX_NAV_PVT_LEN			92
#define UBX_CFG_NAV5_LEN		36
#define UBX_NAV_ODO_LEN			20
#define UBX_CFG_ODO_LEN			20
#define UBX_CFG_RATE_LEN		6
#define UBX_CFG_SBAS_LEN		8

typedef uint8_t		U1;
typedef uint8_t		RU1_3;
typedef int8_t		I1;
typedef uint8_t		X1;
typedef uint16_t	U2;
typedef int16_t 	I2;
typedef uint16_t	X2;
typedef uint32_t	U4;
typedef int32_t		I4;
typedef uint32_t	X4;
typedef float		R4;
typedef double		R8;
typedef char		CH;

typedef struct{
	uint8_t suspend_state;
	uint8_t cfg_rate_req;
	uint8_t cfg_nav5_req;
	uint8_t cfg_odo_req;
	uint8_t cfg_pm2_req;
	uint8_t cfg_dgnss_req;
}neo_struct_t;

typedef struct{
	U4 iTOW;
	U2 year;
	U1 month;
	U1 day;
	U1 hour;
	U1 min;
	U1 sec;
	X1 valid;
	U4 tAcc;
	I4 nano;
	U1 fixType;
	X1 flags;
	X1 flags2;
	U1 numSV;
	I4 lon;
	I4 lat;
	I4 height;
	I4 hMSK;
	U4 hAcc;
	U4 vAcc;
	I4 velN;
	I4 velE;
	I4 velD;
	I4 gSpeed;
	I4 headMot;
	U4 sAcc;
	U4 headAcc;
	U2 pDOP;
	U1 res1;
	U1 res2;
	U1 res3;
	U1 res4;
	U1 res5;
	U1 res6;
	I4 headVeh;
	I2 magDec;
	U2 magAcc;

}ubx_nav_pvt_t;

typedef struct{
	X2 mask;
	U1 dynModel;
	U1 fixMode;
	I4 fixedAlt;
	U4 fixedAltVar;
	I1 minElev;
	U1 drLimit;
	U2 pDop;
	U2 tDop;
	U2 pAcc;
	U2 tAcc;
	U1 staticHoldThresh;
	U1 dgnssTimeout;
	U1 cnoThreshNumSVs;
	U1 cnoThresh;
	U1 res1;
	U1 res2;
	U2 staticHoldMaxDist;
	U1 utcStandard;
	U1 res3;
	U1 res4;
	U1 res5;
	U1 res6;
	U1 res7;
}ubx_cfg_nav5_t;

typedef struct{
	U1 version;
	U1 res1;
	U1 res2;
	U1 res3;
	U1 flags;
	X1 odoCfg;
	U1 res4;
	U1 res5;
	U1 res6;
	U1 res7;
	U1 res8;
	U1 res9;
	U1 cogMaxSpeed;
	U1 cogMaxPosAcc;
	U1 res10;
	U1 res11;
	U1 velLpGain;
	U1 cogLpGain;
	U1 res12;
	U1 res13;
}ubx_cfg_odo_t;

typedef struct{
	U1 dgnssMode;
	U1 res1;
	U1 res2;
	U1 res3;
}ubx_cfg_dgnss_t;

typedef struct{
	U2 measRate;
	U2 navRate;
	U2 timeRef;
}ubx_cfg_rate_t;

typedef struct{
	U1 version;
	U1 res1;
	U1 maxStartupStateDur;
	U1 res2;
	X4 flags;
	U4 updatePeriod;
	U4 searchPeriod;
	U4 gridOffset;
	U2 onTime;
	U2 minAcqTime;
	U1 res[20];
}ubx_cfg_pm2;

typedef struct{
	U1 version;
	U1 res1;
	U1 res2;
	U1 res3;
	U4 iTOW;
	U4 distance;
	U4 totalDistance;
	U4 distanceStd;
}ubx_nav_odo_t;

typedef struct{
	X1 mode;
	X1 usage;
	U1 maxSBAS;
	X1 scanmode2;
	X4 scanmode1;
}ubx_cfg_sbas_t;

void neo_write_byte(SPIDriver *SPID, uint8_t reg_addr, uint8_t value);
void neo_write_no_cs(SPIDriver *SPID, uint8_t *txbuff, uint8_t len);
uint8_t neo_read_byte(SPIDriver *SPID, uint8_t reg_addr);
void neo_read_bytes(SPIDriver *SPID, uint16_t num, uint8_t *rxbuf);
void neo_read_bytes_release_cs(SPIDriver *SPID, uint16_t num, uint8_t *rxbuf);
void neo_read_bytes_no_cs(SPIDriver *SPID, uint16_t num, uint8_t *rxbuf);
void neo_switch_to_ubx(void);
void neo_set_pvt_1hz(void);
void neo_poll_prt(void);
void neo_poll_nav_pvt(void);
void neo_polling(SPIDriver *SPID, uint8_t class, uint8_t id);
void neo_cp_to_struct(uint8_t *msg, uint8_t *strc, uint8_t len);
void neo_write_struct(uint8_t *strc, uint8_t class, uint8_t id, uint8_t payload_len);
void neo_create_poll_request(uint8_t class, uint8_t id);


void neo_apply_header(uint8_t *buffer, uint16_t header);
void neo_apply_class(uint8_t *buffer, uint8_t class);
void neo_apply_id(uint8_t *buffer, uint8_t id);
void neo_apply_length(uint8_t *buffer, uint8_t len);
uint16_t neo_calc_crc(uint8_t *buffer, uint16_t len);

void neo_process_cfg(uint8_t *message);
void neo_process_cfg_odo(uint8_t *message);
void neo_process_pvt(uint8_t *message);
void neo_process_nav(uint8_t *message);
void neo_process_nav5(uint8_t *message);
void neo_process_odo(uint8_t *message);
void neo_process_rate(uint8_t *message);
void neo_poll(void);

int32_t neo_get_lat(ubx_nav_pvt_t *pvt);
int32_t neo_get_lon(ubx_nav_pvt_t *pvt);
uint8_t neo_get_numsv(ubx_nav_pvt_t *pvt);
uint8_t neo_get_valid(ubx_nav_pvt_t *pvt);
int32_t neo_get_ground_speed(ubx_nav_pvt_t *pvt);




#endif /* SD_MODULES_NEO_M8_NEO_M8_H_ */
