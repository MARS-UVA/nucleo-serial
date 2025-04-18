/*
 * util.c
 *
 *  Created on: Jan 28, 2025
 *      Author: diana
 */
#include "util.h"

void floatToByteArray(float f, char *arr)
{
    unsigned int asInt = *((int*) &f);

    for (int i = 0; i < 4; i++)
        arr[i] = (asInt >> 8 * i) & 0xFF;
}

void altFloatToByteArray(float f, char *arr)
{
	uint32_t asInt;
	memcpy(&asInt, &f, sizeof(float));  // Safe type-punning

	for (int i = 0; i < 4; i++)
		arr[i] = (asInt >> (8 * i)) & 0xFF;
}

void float16ToByteArray(float f, char *arr)
{
    unsigned int asInt = *((int*) &f);

    for (int i = 0; i < 4; i++)
        arr[i] = (asInt >> 8 * i) & 0xFF;
}

int min(int a, int b)
{
	return a < b ? a : b;
}

int max(int a, int b)
{
	return a > b ? a : b;
}

float map(int min, int max, int pos)
{
	return ((float) (pos - min)) / (max - min);
}
