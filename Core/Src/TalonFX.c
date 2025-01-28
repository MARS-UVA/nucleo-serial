#include "TalonFX.h"
#include "util.h"
#include <math.h>

void sendFXCANMessage(TalonFX *talonFX, int identifier, char *message, uint8_t length)
{
	sendCANMessage(talonFX->hcan, talonFX->identifier | identifier, message, length);
}

void setFX(TalonFX *talonFX, double speed) {
	int valueInt = (int) (speed * 1023);

	sendFXCANMessage(talonFX, 0x204b540, (char[]){0, 1, 0, 0, 0, 0, (valueInt >> 8) & 255, valueInt & 255}, 8);
}

void setNeutralModeFX(TalonFX *talonFX, NeutralModeValue neutralModeValue) {
	char mode[] = {(neutralModeValue == COAST ? 0 : 4), 0, 0, 0, 0, 0, 0, 0};

	sendFXCANMessage(talonFX, 0x204b4c0, mode, 8);
}

void applySupplyCurrentLimitFX(TalonFX *talonFX, int current) {
	int value = 0x3F00;
	for(int amp = 0; amp < current; amp++) {
		int n = amp <= 1 ? 1 : (int) (1 + log2(amp));
			value += pow(2, 7 - n + 1);
	}

	if (current == 1)
		    value =  0x3f08;

	char x[] = {0x21, 0x72, 0x08, 0, 0, value & 255, (value >> 8) & 255, 0xaa};
	sendFXCANMessage(talonFX, 0x2047c19, x, 8);
}

TalonFX TalonFXInit(CAN_HandleTypeDef *hcan, int32_t identifier)
{
	TalonFX talonFX = {
			.hcan = hcan,
			.set = setFX,
			.setNeutralMode = setNeutralModeFX,
			.applySupplyCurrentLimit = applySupplyCurrentLimitFX,
			.identifier = identifier
	};

    sendFXCANMessage(&talonFX, 0x2040080, "\x00\x00\x00\x00\x00\x00\x00\x00", 8);

	return talonFX;
}
