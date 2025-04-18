#include "main.h"
#include "util.h"
#include "pot.h"

#define POT_CALIBRATION_COUNT 20

float readPot(Pot* pot) {
	HAL_ADC_PollForConversion(pot->hadc, 20);
	return map(pot->minPos, pot->maxPos, HAL_ADC_GetValue(pot->hadc) + pot->actuatorOffset);
}

void calibrateYourMom(Pot *leftPot, Pot *rightPot) {
	  int offset = 0;

	  for (int i = 0; i < POT_CALIBRATION_COUNT; i++) {
		  HAL_ADC_PollForConversion(leftPot->hadc, 20);
		  HAL_ADC_PollForConversion(rightPot->hadc, 20);
		  offset += (HAL_ADC_GetValue(leftPot->hadc) - HAL_ADC_GetValue(rightPot->hadc)) / POT_CALIBRATION_COUNT;
	  }

	  rightPot->actuatorOffset = offset;
}

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
