/*
 * can.h
 *
 *  Created on: Mar 9, 2025
 *      Author: Austen
 */

// For sending/receiving packets on CAN bus

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "main.h"

void sendCANMessage(CAN_HandleTypeDef *hcan, int identifier, char *message, uint8_t length);
void sendGlobalEnableFrame(CAN_HandleTypeDef *hcan);

#endif /* INC_CAN_H_ */
