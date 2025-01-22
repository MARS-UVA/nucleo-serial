#include "TalonSRX.h"

void sendCANMessage(TalonSRX *talonSRX, int identifier, char *message, uint8_t length)
{
	  uint32_t mb;
	  CAN_TxHeaderTypeDef hdr;

	  hdr.ExtId = talonSRX->identifier | identifier;
	  hdr.IDE = CAN_ID_EXT;
	  hdr.RTR = CAN_RTR_DATA;
	  hdr.DLC = length;
	  hdr.TransmitGlobalTime = DISABLE;

	  if (HAL_CAN_AddTxMessage(talonSRX->hcan, &hdr, (unsigned char *) message, &mb) != HAL_OK)
		Error_Handler();
}

void setInverted(TalonSRX *talonSRX, bool invert)
{
	if (!invert)
		return;

  sendCANMessage(talonSRX, 0x2040080, "\x00\x00\x00\x00\x00\x00\x00\x08", 8);
}

void set(TalonSRX *talonSRX, double value)
{
	int valueInt = (int) (value * 1023);

	sendCANMessage(talonSRX, 0x2040200, (char[]){(valueInt >> 16) & 255, (valueInt >> 8) & 255, valueInt & 255, 0, 0, 0, 0x0b, 0}, 8);
}

TalonSRX TalonSRXInit(CAN_HandleTypeDef *hcan, int32_t identifier)
{
	TalonSRX talonSRX = {
			.hcan = hcan,
			.setInverted = setInverted,
			.set = set,
			.identifier = identifier
	};

    sendCANMessage(&talonSRX, 0x2040080, "\x00\x00\x00\x00\x00\x00\x00\x00", 8);

	return talonSRX;
}
