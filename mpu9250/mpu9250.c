/*
 * mpu9250.c
 *
 *  Created on: Mar 14, 2019
 *      Author: a-h
 */

#include "MPU9250.h"

float PI = CONST_PI;
float GyroMeasError = CONST_GME; // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = CONST_beta;  // compute beta
float GyroMeasDrift = CONST_GMD; // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = CONST_zeta; // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

uint8_t Ascale = AFS_8G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
uint8_t Gscale = GFS_1000DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06; // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

void mpu_write_byte(SPIDriver *SPID, uint8_t reg_addr, uint8_t value) {
	uint8_t txbuf[2];
	txbuf[0] = reg_addr;
	txbuf[1] = value;
	// spiAcquireBus(&SPID2);              /* Acquire ownership of the bus.    */
	palSetLine(LINE_GREEN_LED); /* LED ON.                          */
	chThdSleepMilliseconds(100);
	palClearLine(LINE_MPU_CS);
	spiSend(SPID, 2, txbuf); /* send request       */
	//spiSelect(&SPID2);                  /* Slave Select assertion.          */
	//  spiExchange(SPID, 1,
	// 		txbuf, rxbuf);          /* Atomic transfer operations.      */
	//  spiUnselect(&SPID2);                /* Slave Select de-assertion.       */
	//spiReceive(&SPID2, 1, &read_buff);
	spiReleaseBus(SPID); /* Ownership release.               */
	palSetLine(LINE_MPU_CS);
	palClearLine(LINE_GREEN_LED); /* LED OFF.*/
}

uint8_t mpu_read_byte(SPIDriver *SPID, uint8_t reg_addr) {
	uint8_t value;
	reg_addr |= 0x80;
	palSetLine(LINE_GREEN_LED); /* LED ON.                          */
	chThdSleepMilliseconds(100);
	palClearLine(LINE_MPU_CS);
	spiSend(SPID, 1, &reg_addr); /* send request       */
	//spiSelect(&SPID2);                  /* Slave Select assertion.          */
	//spiExchange(SPID, 1,
	//		txbuf, rxbuf);          /* Atomic transfer operations.      */
	//spiUnselect(SPID);                /* Slave Select de-assertion.       */
	spiReceive(SPID, 1, &value);
	spiReleaseBus(SPID); /* Ownership release.               */
	palSetLine(LINE_MPU_CS);
	palClearLine(LINE_GREEN_LED); /* LED OFF.*/
	return value;
}

void mpu_read_bytes(SPIDriver *SPID, uint8_t num, uint8_t reg_addr,
		uint8_t *rxbuf) {
	uint8_t txbuf[num];
	//uint8_t i;
	reg_addr |= 0x80;
	palSetLine(LINE_GREEN_LED); /* LED ON.                          */
	chThdSleepMilliseconds(100);
	palClearLine(LINE_MPU_CS);
	spiSend(SPID, 1, &reg_addr); /* send request       */
	//spiSelect(&SPID2);                  /* Slave Select assertion.          */
	spiExchange(SPID, num, txbuf, rxbuf); /* Atomic transfer operations.      */
	//spiUnselect(SPID);                /* Slave Select de-assertion.       */
	//spiReceive(SPID, 1, value);
	spiReleaseBus(SPID); /* Ownership release.               */
	palSetLine(LINE_MPU_CS);
	palClearLine(LINE_GREEN_LED); /* LED OFF.*/
}

uint16_t mpu9250_init(void) {
	uint8_t tmp = 15;


	tmp = mpu_read_byte(&SPID2, WHO_AM_I_MPU9250);
	chprintf((BaseSequentialStream*)&SD1, "WHO_AM_I: %x \r\n", tmp);
	// wake up device
	// Clear sleep mode bit (6), enable all sensors
	mpu_write_byte(&SPID2, PWR_MGMT_1, 0x00);
	chThdSleepMilliseconds(100); // Wait for all registers to reset

	// Get stable time source
	// Auto select clock source to be PLL gyroscope reference if ready else
	mpu_write_byte(&SPID2, PWR_MGMT_1, 0x01);
	chThdSleepMilliseconds(200);

	// Set accelerometer full-scale range configuration
	// Get current ACCEL_CONFIG register value
	tmp = mpu_read_byte(&SPID2, ACCEL_CONFIG);
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	tmp = tmp & ~0x18;  // Clear AFS bits [4:3]
	tmp = tmp | Ascale << 3; // Set full scale range for the accelerometer
	// Write new ACCEL_CONFIG register value
	mpu_write_byte(&SPID2, ACCEL_CONFIG, tmp);

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by
	// choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
	// 1.13 kHz
	// Get current ACCEL_CONFIG2 register value
	tmp = mpu_read_byte(&SPID2, ACCEL_CONFIG2);
	tmp = tmp & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	tmp = tmp | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	// Write new ACCEL_CONFIG2 register value
	mpu_write_byte(&SPID2, ACCEL_CONFIG2, tmp);
	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because
	// of the SMPLRT_DIV setting

	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
	// respectively;
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion
	// update rates cannot be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!),
	// 8 kHz, or 1 kHz
	mpu_write_byte(&SPID2, CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	// Use a 200 Hz rate; a rate consistent with the filter update rate
	// determined inset in CONFIG above.
	mpu_write_byte(&SPID2, SMPLRT_DIV, 0x04);

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
	// left-shifted into positions 4:3

	// get current GYRO_CONFIG register value
	tmp = mpu_read_byte(&SPID2, GYRO_CONFIG);
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	tmp = tmp & ~0x02; // Clear Fchoice bits [1:0]
	tmp = tmp & ~0x18; // Clear AFS bits [4:3]
	tmp = tmp | Gscale << 3; // Set full scale range for the gyro
	// Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of
	// GYRO_CONFIG
	// c =| 0x00;
	// Write new GYRO_CONFIG value to register
	mpu_write_byte(&SPID2, GYRO_CONFIG, tmp);

	// First extract the factory calibration for each magnetometer axis
	//uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	// TODO: Test this!! Likely doesn't work
	//mpu_write_byte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	chThdSleepMilliseconds(10);
	//mpu_write_byte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	chThdSleepMilliseconds(10);
	// Read the x-, y-, and z-axis calibration values
	//mpu_read_bytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);

	// Return x-axis sensitivity adjustment values, etc.
	//destination[0] = (float) (rawData[0] - 128) / 256. + 1.;
	//destination[1] = (float) (rawData[1] - 128) / 256. + 1.;
	//destination[2] = (float) (rawData[2] - 128) / 256. + 1.;
	//mpu_write_byte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	chThdSleepMilliseconds(10);

	// Configure the magnetometer for continuous read and highest resolution.
	// Set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL
	// register, and enable continuous mode data acquisition Mmode (bits [3:0]),
	// 0010 for 8 Hz and 0110 for 100 Hz sample rates.

	// Set magnetometer data resolution and sample ODR
	//mpu_write_byte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode);
	chThdSleepMilliseconds(10);

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
	// until interrupt cleared, clear on read of INT_STATUS, and enable
	// I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
	// controlled by the Arduino as master.
	mpu_write_byte(&SPID2, INT_PIN_CFG, 0x22);
	// Enable data ready (bit 0) interrupt
	mpu_write_byte(&SPID2, INT_ENABLE, 0x01);
	chThdSleepMilliseconds(100);
}

void mpu_read_accel_data(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  // Read the six raw data registers into data array
  mpu_read_bytes(&SPID2, 6, ACCEL_XOUT_H, &rawData[0]);

  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void mpu_read_gyro_data(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  // Read the six raw data registers sequentially into data array
  mpu_read_bytes(&SPID2, 6, GYRO_XOUT_H, &rawData[0]);

  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}
