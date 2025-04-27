
#ifndef INC_POT_H_
#define INC_POT_H_

#include "main.h"
#include "util.h"

typedef struct pot {
	ADC_HandleTypeDef *hadc;
	float(*read)(struct pot*);
	float(*readCm)(struct pot*);
	uint32_t actuatorOffset;
	int minPos;
	int maxPos;
} Pot;

Pot PotInit();

void calibrateYourMom(Pot *leftPot, Pot *rightPot);

#endif
