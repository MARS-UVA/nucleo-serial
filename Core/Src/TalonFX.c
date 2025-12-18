#include "TalonFX.h"

// send a CAN packet to a Talon FX
void sendFXCANMessage(TalonFX *talonFX, int identifier, char *message, uint8_t length)
{
	sendCANMessage(talonFX->hcan, talonFX->identifier | identifier, message, length);
}

// set the percent output speed of a Talon FX (-1 to 1)
void setFX(TalonFX *talonFX, double speed) {
	short valueInt = (short) (speed * 1024);
	if (valueInt < 0) {
		valueInt = 0xfff - (-1 * valueInt);
	}

	sendFXCANMessage(talonFX, 0x204b540, (char[]){0, 1, 0, 0, 0, 0, valueInt & 255, (valueInt >> 8) & 255}, 8);
	HAL_Delay(1);
}

// set the Talon FX to coast or brake mode
void setNeutralModeFX(TalonFX *talonFX, NeutralModeValue neutralModeValue) {
	char mode[] = { 0x21, 0x6E, 0x08, (neutralModeValue == COAST ? 0 : 1), 0, 0, 0, 0xAA};
	for (int pair = 0; pair < (neutralModeValue == COAST ? 2 : 1); pair++) {
		sendFXCANMessage(talonFX, 0x2047c00, mode, 8);
		HAL_Delay(1);
	}
	sendFXCANMessage(talonFX, 0x2047c00, "\x10\x0c\xc5\x06\x0d\x00\x00\x00", 8);
}

// apply a maximum supply current limit to the Talon FX
void applySupplyCurrentLimitFX(TalonFX *talonFX, float current) {
	char x[] = {0x21, 0x70, 0x08, 0, 0, 0, 0, 0xaa};
	floatToByteArray(current, &x[3]);
	sendFXCANMessage(talonFX, 0x2047c00, x, 8);
	sendFXCANMessage(talonFX, 0x2047c00, "\x10\x0c\xc5\x06\x0d\x00\x00\x00", 8);
	HAL_Delay(1);
	sendFXCANMessage(talonFX, 0x2047c00, x, 8);
	sendFXCANMessage(talonFX, 0x2047c00, "\x10\x0c\xc5\x06\x0d\x00\x00\x00", 8);
	HAL_Delay(1);
}

// apply PID parameter configurations to the Talon FX
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
}

// set the speed of a Talon FX using PID feedback
void setControlFX(TalonFX *talonFX, int velocity, double feedforward)
{
	// Get velocity value (3 bytes)
	if (velocity >= 0) {
		velocity *= 16;
	}
	else {
		velocity = 0x40000 - (-16 * velocity);
	}
	// Get feedforward value
	int feedforwardInt = feedforward * 100;
	if (feedforward < 0)
		feedforwardInt = (~(feedforwardInt * -1)) + 1;


	char x[] = {0, 1, velocity & 0xff, (velocity >> 8) & 0xff, velocity >> 16 & 0xff, 0, feedforwardInt & 0xff, (feedforwardInt >> 8) & 0xff};
	sendFXCANMessage(talonFX, 0x2043700, x, 8);
	HAL_Delay(1);
}

// Set the time taken (in seconds) to for the Talon FX to ramp up from 0% to 100% of target output speed
void voltageCycleClosedLoopRampPeriodFX(TalonFX *talonFX, float period)
{
	char x[] = {0x21, 0x84, 0x08, 0x00, 0x00, 0x00, 0x00, 0xaa};
	floatToByteArray(period, &x[3]);
	sendFXCANMessage(talonFX, 0x2047c00, x, 8);
	sendFXCANMessage(talonFX, 0x2047c00, "\x10\x0c\xc5\x06\x0d\x00\x00\x00", 8);
	HAL_Delay(1);
	sendFXCANMessage(talonFX, 0x2047c00, x, 8);
	sendFXCANMessage(talonFX, 0x2047c00, "\x10\x0c\xc5\x06\x0d\x00\x00\x00", 8);
	HAL_Delay(1);
}

// set the position of a Talon FX using PID feedback
void setPositionFX(TalonFX *talonFX, double position, double feedforward)
{
	uint32_t positionBytes = 0;
	position *= 16;
	// Get position value (18 bits, stored in 3 bytes)
	if (position >= 0) {
		positionBytes = (uint32_t)position;
	}
	else { // if position is negative
		positionBytes = 0x40000 - (uint32_t)(-1 * position); // convert to 18-bit two's complement
	}
	// Get feedforward value (12 bits, stored in 2 bytes)
	uint16_t feedforwardBytes = 0;
	feedforward *= 100;
	if (feedforward >= 0) {
		feedforwardBytes = (uint16_t)feedforward;
	}
	else { // if feedforward is negative
		feedforwardBytes  = 0x1000 - (uint16_t)(-1 * feedforward); // convert to 12-bit two's complement
	}

	char x[] = {0, 1, positionBytes & 0xff, (positionBytes >> 8) & 0xff, (positionBytes >> 16) & 0xff, 0, feedforwardBytes & 0xff, (feedforwardBytes >> 8) & 0xff};
	sendFXCANMessage(talonFX, 0x204b640, x, 8);
	HAL_Delay(1);
}

// process CAN packets received from the TalonFX
void receiveCANFX(TalonFX *talonFX, CAN_RxHeaderTypeDef *msg, uint8_t *data)
{
	  // not correct TalonFX id
	  if ((msg->ExtId & talonFX->identifier) != talonFX->identifier)
		  return;

	  uint32_t mask = 0xFFFFFFC0; // last 6 bits can be different based on CAN ID of TalonFX
	  if ((msg->ExtId & mask) == 0x20447c0) // position readings
	  {
		// read bytes 2-5 of data as a little endian, then discard last 6 bits as only the first 26 bits are used for position
		int32_t positionBytes = ((data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2]) >> 6; // using int32_t instead of uint32_t to handle negative positions
		talonFX->position = (float) positionBytes / 2048;
	  }
	  else {
		  // other CAN messages can be processed here
	  }
}

// gets the rotor position of a Talon FX based on its internal encoder
float getPositionFX(TalonFX *talonFX) {
	return talonFX->position;
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
			.setControl = setControlFX,
			.setPosition = setPositionFX,
			.getPosition = getPositionFX,
			.receiveCAN = receiveCANFX,

	};

	return talonFX;
}
