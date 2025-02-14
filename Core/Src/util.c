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
