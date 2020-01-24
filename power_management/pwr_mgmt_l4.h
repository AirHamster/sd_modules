/*
 * pwr_mgmt.h
 *
 *  Created on: Nov 27, 2019
 *      Author: a-h
 */

#ifndef SD_MODULES_POWER_MANAGEMENT_PWR_MGMT_L4_H_
#define SD_MODULES_POWER_MANAGEMENT_PWR_MGMT_L4_H_

typedef enum{
	DCDC_OFF = 0,
	DCDC_ON,
	DCDC_3_3,
	DCDC_5,
	DCDC_12
}dcdc_enum;

void start_power_management_module(void);

#endif /* SD_MODULES_POWER_MANAGEMENT_PWR_MGMT_L4_H_ */
