/*
 * util.c
 *
 *  Created on: Jan 28, 2025
 *      Author: diana
 */
#include "util.h"

void sendCANMessage(CAN_HandleTypeDef *hcan, int identifier, char *message, uint8_t length)
{
	  uint32_t mb;
	  CAN_TxHeaderTypeDef hdr;

	  hdr.ExtId = identifier;
	  hdr.IDE = CAN_ID_EXT;
	  hdr.RTR = CAN_RTR_DATA;
	  hdr.DLC = length;
	  hdr.TransmitGlobalTime = DISABLE;

	  if (HAL_CAN_AddTxMessage(hcan, &hdr, (unsigned char *) message, &mb) != HAL_OK)
		Error_Handler();
}

 pdpGetCurrentReadings(CAN_HandleTypeDef *hcan, int pdpIdentifier, float *readings)
{
	sendCANMessage(hcan, pdpIdentifier, "\x00\x00\x00\x00\x20\x00", 6);
	HAL_Delay(1);

	CAN_RxHeaderTypeDef hdr;
	char data[8];
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data);

	float val = ((data[5] << 16) + (data[6] << 8) + data[7]) * 0.125;

	for (int i = 0; i < 3; i++)
	{
		switch (hdr.ExtId)
		{
		case 0x8041400:

			break;
		case 0x8041440:
			break;
		case 0x8041480:
			break;
		}
	}
}

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
