/*
 * util.h
 *
 *  Created on: Jan 28, 2025
 *      Author: diana
 */

// For general utility functions like converting a float to bytes, or sending CAN messages

#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#include "main.h"

void sendCANMessage(CAN_HandleTypeDef *hcan, int identifier, char *message, uint8_t length);
void floatToByteArray(float f, char *arr);

#endif /* INC_UTIL_H_ */
