/*
 * sailDataMath.c
 *
 *  Created on: .
 *      Author: Weld
 */
#include "sailDataMath.h"

#define COUNT_FILTERS 10
float lastSensorValues[SIZE_BUFFER_VALUES] = {0.0};
float bufferValues[COUNT_FILTERS][FILTER_BUFFER_SIZE] = {{0.0}};
float lastFilterValues[COUNT_FILTERS][FILTER_BUFFER_SIZE] = {{0.0}};


void correctApparentWind(
        float* apparentWindAngle,
        float* apparentWindSpeed,
        const float heel,
        const float pitch)
{
	float angle = radiansFromDegrees(*apparentWindAngle);
	float x = *apparentWindSpeed * sin(angle)
		/ cos(radiansFromDegrees(HEEL));
	float y = *apparentWindSpeed * cos(angle)
		/ cos(radiansFromDegrees(PITCH));
	*apparentWindSpeed = sqrt(x * x + y * y);
	*apparentWindAngle = angleCalc(degreesFromRadians(atan2(x, y)));
}

float trueHeading(
		const float heading,
		const float magneticDeclanation)
{
	return direction(heading - magneticDeclanation);
}

float apparentWindDirection(
		const float apparentWindAngle,
		const float trueHeading)
{
	return direction(apparentWindAngle + trueHeading);
}

float trueWindSpeed(
		const float apparentWindDirection,
		const float apparentWindSpeed,
		const float courseMadeGood,
		const float speedOverGround)
{
	return sqrt(
			apparentWindSpeed * apparentWindSpeed
			+ speedOverGround * speedOverGround
			- 2 * apparentWindSpeed * speedOverGround * cos(
					radiansFromDegrees(
						apparentWindDirection - courseMadeGood
					)
				)
			);
}

float trueWindDirection(
		const float apparentWindDirection,
		const float apparentWindSpeed,
		const float courseMadeGood,
		const float speedOverGround)
{
	const float angleValue = radiansFromDegrees(
	            apparentWindDirection - courseMadeGood);

	if(
		epsilon > apparentWindSpeed
		&& epsilon > speedOverGround
	){
		return 0.0;
	}

	return direction(
		courseMadeGood
		+ degreesFromRadians(pi
		- atan2(apparentWindSpeed * sin(angleValue),
		speedOverGround - apparentWindSpeed * cos(angleValue))));
}

float trueWindAngle(
		const float trueWindDirection,
		const float trueHeading)
{
	 return angleCalc(trueWindDirection - trueHeading);
}

float hullSpeed(
		const float speedOverGround,
		const float trueHedaing,
		const float courseMadeGood)
{
	return speedOverGround * cos(
			radiansFromDegrees(
			trueHedaing - courseMadeGood));
}

float drift(
		const float trueHeading,
		const float hullSpeed,
		const float courseMadeGood,
		const float speedOverGround)
{
	return sqrt(
			hullSpeed * hullSpeed
			+ speedOverGround * speedOverGround
			- 2 * hullSpeed * speedOverGround * cos(
			radiansFromDegrees(courseMadeGood - trueHeading)));
}

float set(
		const float trueHeading,
		const float hullSpeed,
		const float courseMadeGood,
		const float speedOverGround)
{
	const float angleValue = radiansFromDegrees(
			courseMadeGood - trueHeading);

		return direction(
			trueHeading
			+ degreesFromRadians(pi
			- atan2(speedOverGround * sin(angleValue),
			hullSpeed - speedOverGround * cos(angleValue))));
}

float velocityMadeGood(
		const float hullSpeed,
		const float trueWindAngle)
{
	return hullSpeed * cos(radiansFromDegrees(trueWindAngle));
}

float direction(const float income)
{
	return fmod(income + 3600.0, 360.0);
}

float radiansFromDegrees(const float degrees)
{
	return degrees * pi / 180.0;
}

float degreesFromRadians(const float radians)
{
	return radians * 180.0 / pi;
}

float angleCalc(const float income)
{
	return direction(income + 180.0) - 180.0;
}

void correctSensorValues(CalibrationParmDef *calibParam)
{
	lastSensorValues[AWA]-=calibParam->WindCorrection;
	lastSensorValues[PITCH]-=calibParam->PitchCorrection;
	lastSensorValues[HEEL]-=calibParam->HeelCorrection;
	lastSensorValues[HDM]-=calibParam->CompassCorrection;
}

void calculateValues(CalibrationParmDef *calibParam)
{
	correctSensorValues(calibParam);

	correctApparentWind(
			&lastSensorValues[AWA],
			&lastSensorValues[AWS],
			lastSensorValues[HEEL],
			0.0);

	lastSensorValues[HDT] = trueHeading(
			lastSensorValues[HDM],
			calibParam->MagneticDeclanation);

	lastSensorValues[AWD] = apparentWindDirection(
			lastSensorValues[AWA],
			lastSensorValues[HDT]);

	lastSensorValues[TWS] = trueWindSpeed(
			lastSensorValues[AWD],
			lastSensorValues[AWS],
			lastSensorValues[CMG],
			lastSensorValues[SOG]);

	lastSensorValues[TWD] = trueWindDirection(
			lastSensorValues[AWD],
			lastSensorValues[AWS],
			lastSensorValues[CMG],
			lastSensorValues[SOG]);

	lastSensorValues[TWA] = trueWindAngle(
			lastSensorValues[TWD],
			lastSensorValues[HDT]);

	/// Temporary Delete after connected LAG sensor
	lastSensorValues[HSP] = lastSensorValues[SOG] * cos(
		radiansFromDegrees(lastSensorValues[HDT] - lastSensorValues[CMG]));
	///

	lastSensorValues[DRIFT] = drift(
			lastSensorValues[HDT],
			lastSensorValues[HSP],
			lastSensorValues[CMG],
			lastSensorValues[SOG]);

	lastSensorValues[_SET_] = set(
				lastSensorValues[HDT],
				lastSensorValues[HSP],
				lastSensorValues[CMG],
				lastSensorValues[SOG]);

	lastSensorValues[VMG] = velocityMadeGood(
				lastSensorValues[HSP],
				lastSensorValues[TWA]);
}

void dataFiltering(
		CalibrationParmDef *calibParam)
{
	const uint8_t filteredValuesIters[COUNT_FILTERS] =
			{HDM, HDT, HEEL, RDR, TWD, TWS, TWA, HSP, TWA_TGT, HSP_TGT};
	const uint8_t typeFilteringValue[COUNT_FILTERS] =
			{DIRECTION, DIRECTION, ANGLE, ANGLE, DIRECTION, SPEED, DIRECTION,
					SPEED, ANGLE, SPEED};
	// float bufferValues[COUNT_FILTERS][FILTER_BUFFER_SIZE]
	for(uint8_t i = 0; i < COUNT_FILTERS; i++)
	{
		for(uint8_t j = 0; j < FILTER_BUFFER_SIZE - 1; j++)
		{
			bufferValues[i][j] = bufferValues[i][j+1];
		}
		// Add new value
		bufferValues[i][FILTER_BUFFER_SIZE-1] = lastSensorValues[filteredValuesIters[i]];
		// Filtering data
		switch(typeFilteringValue[i])
		{
		case DIRECTION:
			filterDirectionValues(&bufferValues[i][FILTER_BUFFER_SIZE],
					&lastFilterValues[i][FILTER_BUFFER_SIZE],
					FILTER_BUFFER_SIZE,
					calibParam->WindowSize1);
			break;
		case ANGLE:
			filterAngleValues(&bufferValues[i][FILTER_BUFFER_SIZE],
					&lastFilterValues[i][FILTER_BUFFER_SIZE],
					FILTER_BUFFER_SIZE,
					calibParam->WindowSize1);
			break;
		case SPEED:
			filterSpeedValues(&bufferValues[i][FILTER_BUFFER_SIZE],
					&lastFilterValues[i][FILTER_BUFFER_SIZE],
					FILTER_BUFFER_SIZE,
					calibParam->WindowSize2,
					calibParam->WindowSize3);
			break;
		default:
			filterSpeedValues(&bufferValues[i][FILTER_BUFFER_SIZE],
								&lastFilterValues[i][FILTER_BUFFER_SIZE],
								FILTER_BUFFER_SIZE,
								calibParam->WindowSize2,
								calibParam->WindowSize3);
			break;
		}
	}

}

void calculateTargets(
		//float windSpeed,
		//float windAngle,
		float *windAngleTarget,
		float *hullSpeedTarget,
		float *velocityMadeGoodTarget)
{
	float instantHSP = 0.0;
	float instantVMG = 0.0;
	float minimumTWA = 10.0;
	float windSpeed = lastFilterValues[5][0];
	float windAngle = lastFilterValues[6][0];

	if(fabs(windAngle)> 90.0)
	{
		minimumTWA = 100.0;
	}
	*windAngleTarget = 0.0;
	*hullSpeedTarget = 0.0;
	*velocityMadeGoodTarget = 0.0;

	if(windSpeed < polynoms[0][0]){
		windSpeed = polynoms[0][0];
	}
	if(windSpeed > polynoms[SIZE_POLYNOMS_ROW - 1][0]){
		windSpeed = polynoms[SIZE_POLYNOMS_ROW - 1][0];
	}

	for(uint8_t i = 0; i + 1 < SIZE_POLYNOMS_ROW; ++i)
	{
		if(polynoms[i][0] == windSpeed)
		{
			uint8_t polynomPower = SIZE_POLYNOMS_COLS - 2;

			for(float instantTWA = minimumTWA; instantTWA <= minimumTWA + 70.0;
				instantTWA += 2.0)
			{
				instantHSP = 0.0;
				for(uint8_t j = 0; j <= polynomPower; ++j)
				{
					instantHSP += polynoms[i][1 + j] * pow(instantTWA, j);
				}

				instantVMG = instantHSP * fabs(cos(radiansFromDegrees(instantTWA)));
				if(instantVMG > *velocityMadeGoodTarget){
					*velocityMadeGoodTarget = instantVMG;
					*hullSpeedTarget = instantHSP;
					*windAngleTarget = instantTWA;
				}
			}
			break;
		}else{
			if(windSpeed > polynoms[i][0]
				&& windSpeed < polynoms[i + 1][0])
			{
				uint8_t polynomPower = SIZE_POLYNOMS_COLS - 2;

				float part = (windSpeed - polynoms[i][0])
					/ (polynoms[i + 1][0] - polynoms[i][0]);

				for(float instantTWA = minimumTWA;
					instantTWA <= minimumTWA + 70.0;
					instantTWA += 2.0)
				{
					instantHSP = 0.0;
					for(uint8_t j = 0; j <= polynomPower; ++j)
					{
						instantHSP += ((1. - part) * polynoms[i][1 + j]
								+ part * polynoms[i + 1][1 + j]) * pow(instantTWA, j);
					}

					instantVMG = instantHSP * fabs(cos(radiansFromDegrees(instantTWA)));

					if(instantVMG > *velocityMadeGoodTarget)
					{
						*velocityMadeGoodTarget = instantVMG;
						*hullSpeedTarget = instantHSP;
						*windAngleTarget = instantTWA;
					}
				}
				break;
			}
		}
	}
}
