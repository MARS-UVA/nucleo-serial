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
	uint64_t cache0;
	uint64_t cache40;
	uint64_t cache80;
	short cacheWords[6];
	float (*getChannelCurrent) (struct pdp*, int channelID);
	void (*getSixParam) (struct pdp*, uint64_t *cache);
	void (*requestCurrentReadings) (struct pdp*);
	void (*receiveCAN) (struct pdp*, CAN_RxHeaderTypeDef *msg, uint64_t *data);
	bool receivedNew0;
	bool receivedNew40;
	bool receivedNew80;
	float (*getBusVoltage) (struct pdp*);
	float busVoltage;
} PDP;

PDP PDPInit();

#endif /* INC_PDP_H_ */
