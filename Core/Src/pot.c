#include "main.h"
#include "util.h"
#include "pot.h"

#define POT_CALIBRATION_COUNT 20

float readPot(Pot* pot) {
	HAL_ADC_Start(pot->hadc);
	HAL_ADC_PollForConversion(pot->hadc, 20);
	return map(pot->minPos, pot->maxPos, HAL_ADC_GetValue(pot->hadc) + pot->actuatorOffset);
}

// Converts the potentiometer reading (float) to the distance of bucket drum axle from bottom of the actuators
float readPotCm(Pot *pot) {
	float potReading = readPot(pot);
	return (37.652 * potReading + 10.172);
}


void calibrateYourMom(Pot *leftPot, Pot *rightPot) {
	  int offset = 0;

	  for (int i = 0; i < POT_CALIBRATION_COUNT; i++) {
		  HAL_ADC_Start(leftPot->hadc);
		  HAL_ADC_Start(rightPot->hadc);
		  HAL_ADC_PollForConversion(leftPot->hadc, 20);
		  HAL_ADC_PollForConversion(rightPot->hadc, 20);
		  offset += (HAL_ADC_GetValue(leftPot->hadc) - HAL_ADC_GetValue(rightPot->hadc)) / POT_CALIBRATION_COUNT;
	  }

	  rightPot->actuatorOffset = offset;
}




// todo: create state that stores current value of potentiometer
Pot PotInit(ADC_HandleTypeDef *hadc) {
	Pot pot = {
		.hadc = hadc,
		.read = readPot,
		.readCm = readPotCm,
		.actuatorOffset = 0,
		.minPos = 1190,
		.maxPos = 3153,
	};

	HAL_ADC_Start(hadc);

	return pot;
}
