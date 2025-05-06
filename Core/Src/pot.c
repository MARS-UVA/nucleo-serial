#include "main.h"
#include "util.h"
#include "pot.h"

#define POT_CALIBRATION_COUNT 20

float readPot(Pot* pot) {
	HAL_ADC_Start(pot->hadc);
	HAL_ADC_PollForConversion(pot->hadc, 20);
	return map(pot->minPos, pot->maxPos, HAL_ADC_GetValue(pot->hadc) + pot->actuatorOffset);
}


// converts raw potentiometer reading to the distance of the bucket drum from the bottom (in cm)
float rawPotToCm(float rawReading) {
 	return (-74.9 * rawReading + 4.4625);
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
		.actuatorOffset = 0,
		.minPos = 1190,
		.maxPos = 3153,
	};

	HAL_ADC_Start(hadc);

	return pot;
}
