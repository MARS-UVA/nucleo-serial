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

void float16ToByteArray(float f, char *arr)
{
    unsigned int asInt = *((int*) &f);

    for (int i = 0; i < 4; i++)
        arr[i] = (asInt >> 8 * i) & 0xFF;
}
