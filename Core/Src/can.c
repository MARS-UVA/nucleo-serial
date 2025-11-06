/*
 * can.c
 *
 *  Created on: Mar 9, 2025
 *      Author: Austen
 */

#include "can.h"

// send a CAN packet
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

// send the Global Enable Frame required to enable the Talon motor controllers
void sendGlobalEnableFrame(CAN_HandleTypeDef *hcan)
{
	  uint32_t mb;
	  CAN_TxHeaderTypeDef hdr;

	  hdr.ExtId = 0x401bf;
	  hdr.IDE = CAN_ID_EXT;
	  hdr.RTR = CAN_RTR_DATA;
	  hdr.DLC = 2;
	  hdr.TransmitGlobalTime = DISABLE;

	  if (HAL_CAN_AddTxMessage(hcan, &hdr, (unsigned char *) "\x01\x00", &mb) != HAL_OK)
		Error_Handler();
}
