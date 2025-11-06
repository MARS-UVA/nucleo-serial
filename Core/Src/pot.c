#include "main.h"
#include "util.h"
#include "pot.h"

#define POT_CALIBRATION_COUNT 20

// read a decimal value between 0 and 1 indicating position of potentiometer
float readPot(Pot* pot) {
	HAL_ADC_Start(pot->hadc);
	HAL_ADC_PollForConversion(pot->hadc, 20);
	return map(pot->minPos, pot->maxPos, HAL_ADC_GetValue(pot->hadc) + pot->actuatorOffset);
}


Pot PotInit(ADC_HandleTypeDef *hadc) {
	Pot pot = {
		.hadc = hadc,
		.read = readPot,
		.minPos = 1190,
		.maxPos = 3153,
	};

	HAL_ADC_Start(hadc);

	return pot;
}
