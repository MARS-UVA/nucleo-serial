#include "TalonFX.h"
#include "util.h"
#include <math.h>

void sendFXCANMessage(TalonFX *talonFX, int identifier, char *message, uint8_t length)
{
	sendCANMessage(talonFX->hcan, talonFX->identifier | identifier, message, length);
}

void setFX(TalonFX *talonFX, double speed) {
	short valueInt = (short) (speed * 1024);
	if (valueInt < 0) {
		valueInt = 0xfff - (-1 * valueInt);
	}

	sendFXCANMessage(talonFX, 0x204b540, (char[]){0, 1, 0, 0, 0, 0, valueInt & 255, (valueInt >> 8) & 255}, 8);
}

void setNeutralModeFX(TalonFX *talonFX, NeutralModeValue neutralModeValue) {
	char mode[] = { 0x21, 0x6E, 0x08, (neutralModeValue == COAST ? 0 : 1), 0, 0, 0, 0xAA};
	for (int pair = 0; pair < (neutralModeValue == COAST ? 2 : 1); pair++) {
		sendFXCANMessage(talonFX, 0x2047c00, mode, 8);
		HAL_Delay(1);
	}
	sendFXCANMessage(talonFX, 0x2047c00, "\x10\x0c\xc5\x06\x0d\x00\x00\x00", 8);
}

void applySupplyCurrentLimitFX(TalonFX *talonFX, float current) {
	char x[] = {0x21, 0x72, 0x08, 0, 0, 0, 0, 0xaa};
	floatToByteArray(current, &x[3]);
	sendFXCANMessage(talonFX, 0x2047c00, x, 8);
	sendFXCANMessage(talonFX, 0x2047c00, "\x10\x0c\xc5\x06\x0d\x00\x00\x00", 8);
}

void applyConfigFX(TalonFX *talonFX, Slot0Configs *config)
{
	double *configs[] = {
			&(config->kP),
			&(config->kI),
			&(config->kD),
			&(config->kS),
			&(config->kV),
			&(config->kA),
			&(config->kG)
	};

	for (int i = 0; i < 7; i++)
	{
		char x[] = { 0x21, 0x53 + i, 0x08, 0, 0, 0, 0, 0xaa };
		floatToByteArray(*configs[i], &x[3]);
		sendFXCANMessage(talonFX, 0x2047c00, x, 8);
		sendFXCANMessage(talonFX, 0x2047c00, "\x10\x0c\xc5\x06\x0d\x00\x00\x00", 8);
		HAL_Delay(1);
	}

//	char y[] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//	floatToByteArray(speed, &y[0]);
//	y[0] = 0x00;
//	y[1] = 0x01;
//	floatToByteArray(feedforwardV, &y[4]);
//	y[4] = 0x00;
//	y[5] = 0x00;
//	sendFXCANMessage(talonFX, 0x2047c00, y, 8);
//	sendFXCANMessage(talonFX, 0x2047c00, "\x10\x0c\xc5\x06\x0d\x00\x00\x00", 8);
//

}

void setControlFX(TalonFX *talonFX, int velocity, double feedforward)
{
	velocity *= 16;
	int feedforwardInt = feedforward * 100;
	if (feedforward < 0)
		feedforwardInt = (~(feedforwardInt * -1)) + 1;

	char x[] = {0, 1, velocity & 0xff, (velocity >> 8) & 0xff, 0, 0, feedforwardInt & 0xff, (feedforwardInt >> 8) & 0xff};
	sendFXCANMessage(talonFX, 0x2043700, x, 8);
}

void voltageCycleClosedLoopRampPeriodFX(TalonFX *talonFX, float period)
{
	char x[] = {0x21, 0x83, 0x08, 0x00, 0x00, 0x00, 0x00, 0xaa};
	floatToByteArray(period, &x[3]);
	sendFXCANMessage(talonFX, 0x2047c00, x, 8);
	sendFXCANMessage(talonFX, 0x2047c00, "\x10\x0c\xc5\x06\x0d\x00\x00\x00", 8);
	HAL_Delay(1);
}

TalonFX TalonFXInit(CAN_HandleTypeDef *hcan, int32_t identifier)
{
	TalonFX talonFX = {
			.hcan = hcan,
			.set = setFX,
			.setNeutralMode = setNeutralModeFX,
			.applySupplyCurrentLimit = applySupplyCurrentLimitFX,
			.identifier = identifier,
			.applyConfig = applyConfigFX,
			.voltageCycleClosedLoopRampPeriod = voltageCycleClosedLoopRampPeriodFX,
			.setControl = setControlFX
	};

    sendFXCANMessage(&talonFX, 0x2040080, "\x00\x00\x00\x00\x00\x00\x00\x00", 8);

	return talonFX;
}

