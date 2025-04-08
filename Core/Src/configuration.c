#include "control.h"

extern CAN_HandleTypeDef hcan1;

extern TalonFX frontLeft;
extern TalonFX backLeft;
extern TalonFX frontRight;
extern TalonFX backRight;
extern TalonFX bucketDrum;
extern TalonSRX leftActuator;
extern TalonSRX rightActuator;

void pid_control(SerialPacketPID packet){
	//write to the slot0Configs
	uint8_t CAN_ID = packet.CAN_ID;
	    /*.kP = pid_settings[1],
			.kI = pid_settings[2],
			.kD  = pid_settings[3],
			.kS = pid_settings[4],
			.kV  = pid_settings[5],
			.kA  = pid_settings[6],
			.kG  = pid_settings[7]*/
	struct slot0Configs pidConfigs = {
			(double) packet.kP / 255.0,
			(double) packet.kI / 255.0,
			(double) packet.kD / 255.0,
			(double) packet.kS / 255.0,
			(double) packet.kV / 255.0,
			(double) packet.kA / 255.0,
			(double) packet.kG / 255.0
		};
	switch (CAN_ID){
	case FRONT_LEFT_WHEEL_ID: {
		frontLeft.applyConfig(&frontLeft, &pidConfigs);
		break;
	}
	case BACK_LEFT_WHEEL_ID: {
		backLeft.applyConfig(&backLeft, &pidConfigs);
		break;
	}
	case FRONT_RIGHT_WHEEL_ID: {
		frontRight.applyConfig(&frontRight, &pidConfigs);
		break;
	}
	case BACK_RIGHT_WHEEL_ID: {
		backRight.applyConfig(&backRight, &pidConfigs);
		break;
	}
	case BUCKET_DRUM_ID: {
		bucketDrum.applyConfig(&bucketDrum, &pidConfigs);
		break;
	}
	}
}

void voltageCycleClosedLoopRampPeriod_control(SerialPacketCV packet){
	//write to voltageCycleClosedLoopRampPeriod
	uint8_t CAN_ID = packet.CAN_ID;
	int8_t value = packet.value;

	switch (CAN_ID){
	case FRONT_LEFT_WHEEL_ID: {
		frontLeft.voltageCycleClosedLoopRampPeriod(&frontLeft, (float) value / 255.0);
		break;
	}
	case BACK_LEFT_WHEEL_ID: {
		backLeft.voltageCycleClosedLoopRampPeriod(&backLeft, (float) value / 255.0);
		break;
	}
	case FRONT_RIGHT_WHEEL_ID: {
		frontRight.voltageCycleClosedLoopRampPeriod(&frontRight, (float) value / 255.0);
		break;
	}
	case BACK_RIGHT_WHEEL_ID: {
		backRight.voltageCycleClosedLoopRampPeriod(&backRight, (float) value / 255.0);
		break;
	}
	case BUCKET_DRUM_ID: {
		bucketDrum.voltageCycleClosedLoopRampPeriod(&bucketDrum, (float) value / 255.0);
		break;
	}
	}
}

void appleSupplyCurrentLimit_control(SerialPacketCV packet){
	//write to applySupplyCurrentLimit
	uint8_t CAN_ID = packet.CAN_ID;
	int8_t value = packet.value;
//valuee is just integers basically but still float
	switch (CAN_ID){
	case FRONT_LEFT_WHEEL_ID: {
		frontLeft.applySupplyCurrentLimit(&frontLeft, (float) value);
		break;
	}
	case BACK_LEFT_WHEEL_ID: {
		backLeft.applySupplyCurrentLimit(&backLeft, (float) value);
		break;
	}
	case FRONT_RIGHT_WHEEL_ID: {
		frontRight.applySupplyCurrentLimit(&frontRight, (float) value);
		break;
	}
	case BACK_RIGHT_WHEEL_ID: {
		backRight.applySupplyCurrentLimit(&backRight, (float) value);
		break;
	}
	case BUCKET_DRUM_ID: {
		bucketDrum.applySupplyCurrentLimit(&bucketDrum, (float) value);
		break;
	}
	}
}

