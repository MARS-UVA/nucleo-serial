#include "TalonSRX.h"

// send a CAN message to a Talon SRX
void sendSRXCANMessage(TalonSRX *talonSRX, int identifier, char *message, uint8_t length)
{
	sendCANMessage(talonSRX->hcan, talonSRX->identifier | identifier, message, length);
}

// set the direction of a Talon SRX to be inverted
void setInvertedSRX(TalonSRX *talonSRX, bool invert)
{
  talonSRX->inverted = invert;
}

// set the output magnitude of a Talon SRX
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


	return talonSRX;
}
