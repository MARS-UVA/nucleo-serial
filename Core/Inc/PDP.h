/*
 * PDP.h
 *
 *  Created on: Feb 28, 2025
 *      Author: diana
 */

#ifndef INC_PDP_H_
#define INC_PDP_H_

#include <stdbool.h>
#include "util.h"
#include "main.h"

typedef struct pdp {
	CAN_HandleTypeDef *hcan;
	int identifier;
	unsigned long cache;
	short cacheWords[6];
	float (*getChannelCurrent) (struct pdp*, int channelID);
	void (*getSixParam) (struct pdp*, int arbID);
	void (*requestCurrentReadings) (struct pdp*);
} PDP;

PDP PDPInit();

#endif /* INC_PDP_H_ */
