/*
 * util.h
 *
 *  Created on: Jan 28, 2025
 *      Author: diana
 */

#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#include "main.h"

void sendCANMessage(CAN_HandleTypeDef *hcan, int identifier, char *message, uint8_t length);

#endif /* INC_UTIL_H_ */
