/*
 * util.h
 *
 *  Created on: Jan 28, 2025
 *      Author: diana
 */

// For general utility functions like converting a float to bytes

#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#include "main.h"
#include <string.h>

void floatToByteArray(float f, char *arr);
void altFloatToByteArray(float f, char *arr);
int min(int a, int b);
int max(int a, int b);
float map(int min, int max, int pos);

#endif /* INC_UTIL_H_ */
