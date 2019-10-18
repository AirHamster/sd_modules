/*
 * sailDataMath.h
 *
 *  Created on: 13.
 *      Author: Weld
 */

#ifndef SAILDATAMATH_H_
#define SAILDATAMATH_H_


#include <math.h>
#include <stdint.h>
#include "filters.h"

#define FILTER_BUFFER_SIZE 150
#define SIZE_POLYNOMS_ROW 16
#define SIZE_POLYNOMS_COLS 6
static const float pi = 3.1415926535798932384626433;
static const float epsilon = pow(10., -6);

static const float polynoms[SIZE_POLYNOMS_ROW][SIZE_POLYNOMS_COLS] = {
	{4.0,	-0.00137366,	0.092899,	0.000570625,	-0.000013094,	0.0000000413081},
	{5.0,	-0.00377787,	0.142829,	-0.000260888,	-0.00000680707,	0.0000000240675},
	{6.0,	-0.00369895, 	0.17924,	-0.00060287,	-0.00000563639,	0.000000022454},
	{7.0,	-0.000403524, 	0.257207,	-0.00256176,	0.0000115037,	-0.0000000251304},
	{8.0,	0.000886637,	0.29613,	-0.00349742,	0.0000196293,	-0.0000000475181},
	{9.0,	0.00514052,		0.337691,	-0.00442627,	0.0000275715,	-0.0000000695704},
	{10.0,	0.00925364,		0.343032,	-0.00437227,	0.0000265983,	-0.0000000662184},
	{12.0,	0.0110311,		0.3735,		-0.00506253,	0.0000331547,	-0.0000000855097},
	{14.0,	0.00377768,		0.399644,	-0.00578915,	0.0000408705,	-0.000000109406},
	{16.0,	-0.00142062,	0.41378,	-0.0061903,		0.0000459652,	-0.000000126885},
	{18.0,	0.000683698,	0.421678,	-0.00646876,	0.0000505985,	-0.000000144667},
	{20.0,	-0.00258891,	0.438666,	-0.0070677,		0.0000585593,	-0.000000172584},
	{22.0,	0.0115031,		0.441826,	-0.00719214,	0.0000615326,	-0.000000185071},
	{24.0,	0.00102591,		0.41709,	-0.00634482,	0.0000543442,	-0.000000166166},
	{26.0,	-0.00344542,	0.394572,	-0.00551774,	0.0000470076,	-0.000000146204},
	{28.0,	-0.0181405,		0.396778,	-0.00554412,	0.0000482581,	-0.000000151823}};

enum calculatedValuesIndex{
	/// GPS - Global Positioning System
	LAT,
	LNG,
	TS,
	CMG,
	SOG,
	DIST,
	/// INS - Inertial Navigation System
	/// Accelerometer + Gyroscope + Magnetometer
	HDM,
	HEEL,
	PITCH,
	/// WS - Wind Sensor
	AWA,
	AWS,
	/// DST - Depth, Speed and Temperature Sensor
	/// Log + Depth Sensor
	HSP,
	DEPTH,
	/// RDR - Rudder
	RDR,
	/// HBS - Heartbeat Sensor
	HB,
	/// Calculated Parameters
	HDT,
	AWD,
	TWS,
	TWD,
	TWA,
	DRIFT,
	_SET_,
	VMG,
	/// Additional wind sensor parametera
	AWA_MIN,
	AWA_MAX,
	AWS_MIN,
	AWS_MAX,
	/// Targets
	TWA_TGT,
	HSP_TGT,
	VMG_TGT,
	SIZE_BUFFER_VALUES
};

enum TypeFiltering{
	DIRECTION,
	ANGLE,
	SPEED
};

typedef struct{
	float MagneticDeclanation;
	float WindCorrection;
	float PitchCorrection;
	float HeelCorrection;
	float CompassCorrection;
	float HSPCorrection;
	float RudderCorrection;
	uint8_t WindowSize1;
	uint8_t WindowSize2;
	uint8_t WindowSize3;
} CalibrationParmDef;

// Additional functions
float direction(const float income);
float radiansFromDegrees(const float degrees);
float angleCalc(const float income);
float degreesFromRadians(const float radians);

static inline void filterDirectionValues(
		const float* income,
		float * filtered,
		const uint8_t lengthBuf,
		const uint8_t windwoSize)
{
	static float sineValues[FILTER_BUFFER_SIZE] = {0.0};
	static float cosineValues[FILTER_BUFFER_SIZE] = {0.0};
	if(lengthBuf > FILTER_BUFFER_SIZE)
	{
		return;
	}
	for(uint8_t i = 0; i < lengthBuf; i++)
	{
		filtered[i] = sin(radiansFromDegrees(income[i]));
	}
	trimmedMeanFilter(filtered, &sineValues[0], lengthBuf, windwoSize, 0.0001);
	for(uint8_t i = 0; i < lengthBuf; i++)
	{
		filtered[i] = cos(radiansFromDegrees(income[i]));
	}
	trimmedMeanFilter(filtered, &cosineValues[0], lengthBuf, windwoSize, 0.0001);
	for(uint8_t i = 0; i < lengthBuf; i++)
	{
		filtered[i] = direction(degreesFromRadians(atan2(sineValues[i], cosineValues[i])));
	}
};

static inline void filterAngleValues(
		float * income,
		float * filtered,
		const uint8_t lengthBuf,
		const uint8_t windowSize)
{
	if(lengthBuf > FILTER_BUFFER_SIZE)
	{
		return;
	}
	filterDirectionValues(income, filtered, lengthBuf, windowSize);
	for(uint8_t i = 0; i < lengthBuf; i++)
	{
		filtered[i] = angleCalc(filtered[i]);
	}
}

static inline void filterSpeedValues(
		float * income,
		float * filtered,
		const uint8_t lengthBuf,
		const uint8_t windowSize1,
		const uint8_t windowSize2)
{
	static float temporaryValues[FILTER_BUFFER_SIZE] = {0.0};
	if(lengthBuf > FILTER_BUFFER_SIZE)
	{
		return;
	}
	medianFilter(income, &temporaryValues[0], lengthBuf, windowSize1, 0.0001);
	trimmedMeanFilter(&temporaryValues[0], filtered, lengthBuf, windowSize2, 0.0001);
}

void dataFiltering(
		CalibrationParmDef *calibParam);

void calculateValues(
		CalibrationParmDef *calibParam);

void calculateTargets(void);

void correctSensorValues(
		CalibrationParmDef *calibParam);

// Main calculation functions
void correctApparentWind(
        float* apparentWindAngle,
        float* apparentWindSpeed,
        const float heel,
        const float pitch);

float trueHeading(
		const float heading,
		const float magneticDeclanation);

float apparentWindDirection(
		const float apparentWindAngle,
		const float trueHeading);

float trueWindSpeed(
		const float apparentWindDirection,
		const float apparentWindSpeed,
		const float courseMadeGood,
		const float speedOverGround);

float trueWindDirection(
		const float apparentWindAngle,
		const float apparentWindSpeed,
		const float trueHeading,
		const float courseMadeGood,
		const float speedOverGround);

float trueWindAngle(
		const float trueWindDirection,
		const float trueHeading);

/// Temporary

float hullSpeed(
		const float speedOverGround,
		const float trueHedaing,
		const float courseMadeGood);
///

float drift(
		const float trueHeading,
		const float hullSpeed,
		const float courseMadeGood,
		const float speedOverGround);

float set(
		const float trueHeading,
		const float hullSpeed,
		const float courseMadeGood,
		const float speedOverGround);

float velocityMadeGood(
		const float hullSpeed,
		const float trueWindAngle);


#endif /* SAILDATAMATH_H_ */

