/*
 * mpu9250.c
 *
 *  Created on: Mar 14, 2019
 *      Author: a-h
 */

#include "MPU9250.h"
#include "quaternionFilters.h"
mpu_struct_t mpu_struct;
mpu_struct_t *mpu = &mpu_struct;
extern const SPIConfig mpu_spi_cfg;
extern struct ch_semaphore usart1_semaph;
extern struct ch_semaphore spi2_semaph;
float PI = CONST_PI;
float GyroMeasError = CONST_GME; // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = CONST_beta;  // compute beta
float GyroMeasDrift = CONST_GMD; // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = CONST_zeta; // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value



int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
float temperature;
float SelfTest[6];
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
extern volatile float q0;
extern volatile float q1;
extern volatile float q2;
extern volatile float q3;

uint8_t Ascale = AFS_8G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
uint8_t Gscale = GFS_1000DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06; // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR
extern float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

void mpu_write_byte(SPIDriver *SPID, uint8_t reg_addr, uint8_t value) {
	uint8_t txbuf[2];
	txbuf[0] = reg_addr;
	txbuf[1] = value;
	chSemWait(&spi2_semaph);
	spiAcquireBus(SPID);              /* Acquire ownership of the bus.    */
//	spiStart(&SPID2, &mpu_spi_cfg);
	palClearLine(LINE_MPU_CS);
	chThdSleepMilliseconds(1);
	spiSend(SPID, 2, txbuf); /* send request       */
	palSetLine(LINE_MPU_CS);
	spiReleaseBus(SPID); /* Ownership release.               */
	chSemSignal(&spi2_semaph);
	chThdSleepMilliseconds(1);
}

uint8_t mpu_read_byte(SPIDriver *SPID, uint8_t reg_addr) {
	uint8_t value;
	reg_addr |= 0x80;	//0x80 indicates read operation
	chSemWait(&spi2_semaph);
	spiAcquireBus(SPID);              /* Acquire ownership of the bus.    */
//	spiStart(&SPID2, &mpu_spi_cfg);
	palClearLine(LINE_MPU_CS);
	chThdSleepMilliseconds(1);
	spiSend(SPID, 1, &reg_addr); /* send request       */
	spiReceive(SPID, 1, &value);
	palSetLine(LINE_MPU_CS);
	spiReleaseBus(SPID); /* Ownership release.               */
	chSemSignal(&spi2_semaph);
	chThdSleepMilliseconds(1);
	return value;
}

void mpu_read_bytes(SPIDriver *SPID, uint8_t num, uint8_t reg_addr,
		uint8_t *rxbuf) {
	uint8_t txbuf[num];
	reg_addr |= 0x80;
	chSemWait(&spi2_semaph);
	spiAcquireBus(SPID);              /* Acquire ownership of the bus.    */
//	spiStart(&SPID2, &mpu_spi_cfg);
	palClearLine(LINE_MPU_CS);
	//chThdSleepMilliseconds(1);
	spiSend(SPID, 1, &reg_addr); /* send request       */
	spiExchange(SPID, num, txbuf, rxbuf); /* Atomic transfer operations.      */
	palSetLine(LINE_MPU_CS);
	spiReleaseBus(SPID); /* Ownership release.               */
	chSemSignal(&spi2_semaph);

	//chThdSleepMilliseconds(1);
}

void mpu_get_gyro_data(void){
	float deltat = 0.0025f;

	//mpu_read_accel_data(&mpu->accelCount[0]);
	//mpu_read_gyro_data(&mpu->gyroCount[0]);
	mpu_read_mag_data(&mpu->magCount[0]);
	/*chSemWait(&usart1_semaph);
							chprintf((BaseSequentialStream*)&SD1, "magCount: %d\r\n",
									mpu->magCount[1]);
							chSemSignal(&usart1_semaph); */
	/*chSemWait(&usart1_semaph);
					chprintf((BaseSequentialStream*)&SD1, "A1: %d, A2: %d, A3: %d  G1: %d, G2: %d, G3: %d  M1: %d, M2: %d, M3: %d\r\n",
							mpu->accelCount[0], mpu->accelCount[1], mpu->accelCount[2],
							mpu->gyroCount[0], mpu->gyroCount[1], mpu->gyroCount[2],
							mpu->magCount[0], mpu->magCount[1], mpu->magCount[2]);
					chSemSignal(&usart1_semaph);
*/
	//chSysLock();
	// Now we'll calculate the accleration value into actual g's
	mpu->ax = (float)mpu->accelCount[0]*mpu->aRes - mpu->accelBias[0];  // get actual g value, this depends on scale being set
	mpu->ay = (float)mpu->accelCount[1]*mpu->aRes - mpu->accelBias[1];
	mpu->az = (float)mpu->accelCount[2]*mpu->aRes - mpu->accelBias[2];

	// Calculate the gyro value into actual degrees per second
	mpu->gx = (float)mpu->gyroCount[0]*mpu->gRes - mpu->gyroBias[0];  // get actual gyro value, this depends on scale being set
	mpu->gy = (float)mpu->gyroCount[1]*mpu->gRes - mpu->gyroBias[1];
	mpu->gz = (float)mpu->gyroCount[2]*mpu->gRes - mpu->gyroBias[2];

	mpu->mx = (float)mpu->magCount[0]*mpu->mRes*mpu->magCalibration[0] - mpu->magbias[0];  // get actual magnetometer value, this depends on scale being set
	mpu->my = (float)mpu->magCount[1]*mpu->mRes*mpu->magCalibration[1] - mpu->magbias[1];
	mpu->mz = (float)mpu->magCount[2]*mpu->mRes*mpu->magCalibration[2] - mpu->magbias[2];

	/*chSemWait(&usart1_semaph);
	chprintf((BaseSequentialStream*)&SD1, "magCount: %d, mRes: %f, magCalibration: %f, my: %f\r\n",
						mpu->magCount[1], mpu->mRes, mpu->magCalibration[1], mpu->my);
	chSemSignal(&usart1_semaph); */
	/*chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*)&SD1, "AX: %f, AY: %f, AZ: %f  GX: %f, GY: %f, GZ: %f  MX: %f, MY: %f, MZ: %f\r\n",
		mpu->ax, mpu->ay, mpu->az,
		mpu->gx, mpu->gy, mpu->gz,
		//mpu->magCount[0], mpu->magCount[1], mpu->magCount[2]);
		mpu->mx, mpu->my, mpu->mz);
	chSemSignal(&usart1_semaph);*/
	palToggleLine(LINE_ORANGE_LED);

	MahonyQuaternionUpdate(mpu->ax, mpu->ay, mpu->az, mpu->gx*PI/180.0f, mpu->gy*PI/180.0f, mpu->gz*PI/180.0f, mpu->my, mpu->mx, mpu->mz, deltat);
	mpu->yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
	mpu->pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
	mpu->roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

	/*
	MahonyAHRSupdate(mpu->ax, mpu->ay, mpu->az, mpu->gx*PI/180.0f, mpu->gy*PI/180.0f, mpu->gz*PI/180.0f, mpu->my, mpu->mx, mpu->mz);
	mpu->yaw   = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
	mpu->pitch = -asin(2.0f * (q1 * q3 - q0 * q2));
	mpu->roll  = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
	*/
	mpu->pitch *= 180.0f / PI;

	mpu->yaw   *= 180.0f / PI;
	mpu->yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04

	mpu->roll  *= 180.0f / PI;

	//chSysUnlock();

}

uint16_t mpu9250_init(void) {

	uint8_t tmp = 15;


	tmp = mpu_read_byte(&SPID2, WHO_AM_I_MPU9250);
	if (tmp == 0x71){
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*)&SD1, "MPU9250 on-line\r\n");
		chSemSignal(&usart1_semaph);
	}else{
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*)&SD1, "MPU9250 not found, %x\r\n", tmp);
		chSemSignal(&usart1_semaph);
	}

	calibrateMPU9250(mpu->gyroBias, mpu->accelBias);


	// Initialize MPU9250 device
	// wake up device
	mpu_write_byte(&SPID2, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
	chThdSleepMilliseconds(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

	// get stable time source
	mpu_write_byte(&SPID2, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

	// Configure Gyro and Accelerometer
	// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
	// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	// Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
	mpu_write_byte(&SPID2, CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	mpu_write_byte(&SPID2, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c = mpu_read_byte(&SPID2, GYRO_CONFIG); // get current GYRO_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x02; // Clear Fchoice bits [1:0]
	c = c & ~0x18; // Clear AFS bits [4:3]
	c = c | Gscale << 3; // Set full scale range for the gyro
	// c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	mpu_write_byte(&SPID2, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

	// Set accelerometer full-scale range configuration
	c = mpu_read_byte(&SPID2, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x18;  // Clear AFS bits [4:3]
	c = c | Ascale << 3; // Set full scale range for the accelerometer
	mpu_write_byte(&SPID2, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = mpu_read_byte(&SPID2, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
	c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	mpu_write_byte(&SPID2, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	mpu_write_byte(&SPID2, INT_PIN_CFG, 0x22);
	mpu_write_byte(&SPID2, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt

	//Enable I2C master - needed to access to magn
	mpu_write_byte(&SPID2, USER_CTRL, I2C_MST_EN);
	chThdSleepMilliseconds(10);
	mpu_write_byte(&SPID2, I2C_MST_CTRL, I2C_MST_CLK);

	mpu_write_byte(&SPID2, I2C_SLV0_CTRL, 0);	//step 3 - reset i2c slave reg
	mpu_write_byte(&SPID2, I2C_SLV0_ADDR, 0x80 | 0x0C);	//step 7 - rnw to 1 and addr of magn

	//Slave 4 used only for writting registers in magnetometr
	mpu_write_byte(&SPID2, I2C_SLV4_CTRL, 0);	//step 3 - reset i2c slave reg
	mpu_write_byte(&SPID2, I2C_SLV4_ADDR, 0x0C);	//step 7 - rnw to 0 and addr of magn

	mpu->magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
	mpu->magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
	mpu->magbias[2] = +125.;  // User environmental x-axis correction in milliGauss

	getAres(); // Get accelerometer sensitivity
	getGres(); // Get gyro sensitivity
	getMres(); // Get magnetometer sensitivity
	chThdSleepMilliseconds(100);
	//palToggleLine(LINE_RED_LED);
	return 0;
}

uint8_t get_mag_whoami(void)
{
	uint8_t rawData;
	rawData = read_AK8963_register(0x00);
	return rawData;
}

void initAK8963(float *destination){
	uint8_t tmp = get_mag_whoami();
	if (tmp == 0x48){
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*)&SD1, "Magnetometr on-line\r\n");
		chSemSignal(&usart1_semaph);
	}else{
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*)&SD1, "Magnetometr not found\r\n");
		chSemSignal(&usart1_semaph);
	}
	palToggleLine(LINE_GREEN_LED);
	// First extract the factory calibration for each magnetometer axis
	uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	write_AK8963_register(AK8963_CNTL, 0x00); // Power down magnetometer
	chThdSleepMilliseconds(100);
	write_AK8963_register(AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	chThdSleepMilliseconds(100);
	// Read the x-, y-, and z-axis calibration values
	read_AK8963_registers(AK8963_ASAX, 3, &rawData[0]);
	chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*)&SD1, "ASA: %d, %d, %d\r\n", rawData[0], rawData[1], rawData[2]);
		chSemSignal(&usart1_semaph);
	// Return x-axis sensitivity adjustment values, etc.
	destination[0] = (float) (rawData[0] - 128) / 256. + 1.;
	destination[1] = (float) (rawData[1] - 128) / 256. + 1.;
	destination[2] = (float) (rawData[2] - 128) / 256. + 1.;
	write_AK8963_register(AK8963_CNTL, 0x00); // Power down magnetometer
	chThdSleepMilliseconds(10);

	// Configure the magnetometer for continuous read and highest resolution.
	// Set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL
	// register, and enable continuous mode data acquisition Mmode (bits [3:0]),
	// 0010 for 8 Hz and 0110 for 100 Hz sample rates.

	// Set magnetometer data resolution and sample ODR
	write_AK8963_register(AK8963_CNTL, Mscale << 4 | Mmode);
	chThdSleepMilliseconds(10);

}

uint8_t read_AK8963_register(uint8_t regaddr){
	uint8_t data;
	mpu_write_byte(&SPID2, I2C_SLV0_REG, regaddr);	//step 11 - reg addr
	mpu_write_byte(&SPID2, I2C_SLV0_CTRL, I2C_SLV_EN | 1);	//step 15 - slave en to 1 and num of bytes (1)
	chThdSleepMilliseconds(1);
	mpu_write_byte(&SPID2, I2C_SLV0_CTRL, 0);	//step 20 - reset i2c slave
	data = mpu_read_byte(&SPID2, EXT_SENS_DATA_00);	//read byte
	return data;
}
void read_AK8963_registers(uint8_t regaddr, uint8_t num, uint8_t *buff){
	mpu_write_byte(&SPID2, I2C_SLV0_REG, regaddr);	//step 11 - reg addr
	mpu_write_byte(&SPID2, I2C_SLV0_CTRL, I2C_SLV_EN | num);	//step 15 - slave en to 1 and num of bytes (1)
	chThdSleepMilliseconds(1);
	mpu_write_byte(&SPID2, I2C_SLV0_CTRL, 0);	//step 20 - reset i2c slave
	mpu_read_bytes(&SPID2, num, EXT_SENS_DATA_00, buff);	//read byte
}

void write_AK8963_register(uint8_t regaddr, uint8_t data){
	mpu_write_byte(&SPID2, I2C_SLV4_REG, regaddr);	//step 11 - reg addr
	mpu_write_byte(&SPID2, I2C_SLV4_DO, data);	//step 11 - data to be written
	mpu_write_byte(&SPID2, I2C_SLV4_CTRL, I2C_SLV_EN);	//step 15 - slave en to 1 and num of bytes (1)
	chThdSleepMilliseconds(1);
	mpu_write_byte(&SPID2, I2C_SLV4_CTRL, 0);	//step 20 - reset i2c slave
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

void mpu_read_mag_data(int16_t * destination)
{
	uint8_t rawData[7];  // x/y/z gyro register data stored here
	// Read the six raw data registers sequentially into data array
	//seven bytes reading because last reg should be read to unlatch data regs
	read_AK8963_registers(AK8963_XOUT_L, 7, &rawData[0]);

	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void getMres() {
	switch (Mscale)
	{
	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
	case MFS_14BITS:
		mpu->mRes = 10.0*4912.0/8190.0; // Proper scale to return milliGauss
		break;
	case MFS_16BITS:
		mpu->mRes = 10.0*4912.0/32760.0; // Proper scale to return milliGauss
		break;
	}
}


void getGres() {
	switch (Gscale)
	{
	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
	// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case GFS_250DPS:
		mpu->gRes = 250.0/32768.0;
		break;
	case GFS_500DPS:
		mpu->gRes = 500.0/32768.0;
		break;
	case GFS_1000DPS:
		mpu->gRes = 1000.0/32768.0;
		break;
	case GFS_2000DPS:
		mpu->gRes = 2000.0/32768.0;
		break;
	}
}


void getAres() {
	switch (Ascale)
	{
	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
	// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case AFS_2G:
		mpu->aRes = 2.0/32768.0;
		break;
	case AFS_4G:
		mpu->aRes = 4.0/32768.0;
		break;
	case AFS_8G:
		mpu->aRes = 8.0/32768.0;
		break;
	case AFS_16G:
		mpu->aRes = 16.0/32768.0;
		break;
	}
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(float * dest1, float * dest2)
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	// reset device, reset all registers, clear gyro and accelerometer bias registers
	mpu_write_byte(&SPID2, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	chThdSleepMilliseconds(100);

	// get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	mpu_write_byte(&SPID2, PWR_MGMT_1, 0x01);
	mpu_write_byte(&SPID2, PWR_MGMT_2, 0x00);
	chThdSleepMilliseconds(200);

	// Configure device for bias calculation
	mpu_write_byte(&SPID2, INT_ENABLE, 0x00);   // Disable all interrupts
	mpu_write_byte(&SPID2, FIFO_EN, 0x00);      // Disable FIFO
	mpu_write_byte(&SPID2, PWR_MGMT_1, 0x00);   // Turn on internal clock source
	mpu_write_byte(&SPID2, I2C_MST_CTRL, 0x00); // Disable I2C master
	mpu_write_byte(&SPID2, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	mpu_write_byte(&SPID2, USER_CTRL, 0x0C);    // Reset FIFO and DMP
	chThdSleepMilliseconds(15);

	// Configure MPU9250 gyro and accelerometer for bias calculation
	mpu_write_byte(&SPID2, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	mpu_write_byte(&SPID2, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	mpu_write_byte(&SPID2, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	mpu_write_byte(&SPID2, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	mpu_write_byte(&SPID2, USER_CTRL, 0x40);   // Enable FIFO
	mpu_write_byte(&SPID2, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
	chThdSleepMilliseconds(4);; // accumulate 40 samples in 80 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	mpu_write_byte(&SPID2, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	mpu_read_bytes(&SPID2, 2, FIFO_COUNTH, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		mpu_read_bytes(&SPID2, 12, FIFO_R_W, &data[0]); // read data for averaging
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];

	}
	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) accelsensitivity;}

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;

	/// Push gyro biases to hardware registers
	/*  mpu_write_byte(&SPID2, XG_OFFSET_H, data[0]);
  mpu_write_byte(&SPID2, XG_OFFSET_L, data[1]);
  mpu_write_byte(&SPID2, YG_OFFSET_H, data[2]);
  mpu_write_byte(&SPID2, YG_OFFSET_L, data[3]);
  mpu_write_byte(&SPID2, ZG_OFFSET_H, data[4]);
  mpu_write_byte(&SPID2, ZG_OFFSET_L, data[5]);
	 */
	dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
	dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	mpu_read_bytes(&SPID2, 2, XA_OFFSET_H, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	mpu_read_bytes(&SPID2, 2, YA_OFFSET_H, &data[0]);
	accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	mpu_read_bytes(&SPID2, 2, ZA_OFFSET_H, &data[0]);
	accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

	for(ii = 0; ii < 3; ii++) {
		if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	// Apparently this is not working for the acceleration biases in the MPU-9250
	// Are we handling the temperature correction bit properly?
	// Push accelerometer biases to hardware registers
	/*  mpu_write_byte(&SPID2, XA_OFFSET_H, data[0]);
  mpu_write_byte(&SPID2, XA_OFFSET_L, data[1]);
  mpu_write_byte(&SPID2, YA_OFFSET_H, data[2]);
  mpu_write_byte(&SPID2, YA_OFFSET_L, data[3]);
  mpu_write_byte(&SPID2, ZA_OFFSET_H, data[4]);
  mpu_write_byte(&SPID2, ZA_OFFSET_L, data[5]);
	 */
	// Output scaled accelerometer biases for manual subtraction in the main program
	dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
	dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
	dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}
