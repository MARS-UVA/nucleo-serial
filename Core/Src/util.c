/*
 * util.c
 *
 *  Created on: Jan 28, 2025
 *      Author: diana
 */
#include "util.h"

// convert a float to an array of 4 bytes
void floatToByteArray(float f, char *arr)
{
    unsigned int asInt = *((int*) &f);

    for (int i = 0; i < 4; i++)
        arr[i] = (asInt >> 8 * i) & 0xFF;
}

// unused
void altFloatToByteArray(float f, char *arr)
{
	uint32_t asInt;
	memcpy(&asInt, &f, sizeof(float));  // Safe type-punning

	for (int i = 0; i < 4; i++)
		arr[i] = (asInt >> (8 * i)) & 0xFF;
}

// unused
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

// map a value "pos" from a range of "min" to "max" to a float between 0 and 1
float map(int min, int max, int pos)
{
	return ((float) (pos - min)) / (max - min);
}
