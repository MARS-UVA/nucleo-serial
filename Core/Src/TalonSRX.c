#include "TalonSRX.h"

void sendSRXCANMessage(TalonSRX *talonSRX, int identifier, char *message, uint8_t length)
{
	sendCANMessage(talonSRX->hcan, talonSRX->identifier | identifier, message, length);
}

void setInvertedSRX(TalonSRX *talonSRX, bool invert)
{
  talonSRX->inverted = invert;
}

void setSRX(TalonSRX *talonSRX, double value)
{
	int valueInt = (int) (value * 1023);

	sendSRXCANMessage(talonSRX, 0x2040200, (char[]){(valueInt >> 16) & 255, (valueInt >> 8) & 255, valueInt & 255, 0, 0, 0, 0x0b, talonSRX->inverted ? 0x40 : 0x00}, 8);
}

TalonSRX TalonSRXInit(CAN_HandleTypeDef *hcan, int32_t identifier)
{
	TalonSRX talonSRX = {
			.hcan = hcan,
			.setInverted = setInvertedSRX,
			.set = setSRX,
			.identifier = identifier,
			.inverted = false
	};

    sendSRXCANMessage(&talonSRX, 0x2040080, "\x00\x00\x00\x00\x00\x00\x00\x00", 8);

	return talonSRX;
}
