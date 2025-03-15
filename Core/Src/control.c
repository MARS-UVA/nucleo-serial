#include "control.h"

extern CAN_HandleTypeDef hcan1;

TalonFX frontLeft;
TalonFX backLeft;
TalonFX frontRight;
TalonFX backRight;
TalonFX bucketDrum;
TalonSRX leftActuator;
TalonSRX rightActuator;



// Initialize Talon "objects"
void initializeTalons() {
	frontLeft = TalonFXInit(&hcan1, FRONT_LEFT_WHEEL_ID);
//	backLeft = TalonFXInit(&hcan1, BACK_LEFT_WHEEL_ID);
	frontRight = TalonFXInit(&hcan1, FRONT_RIGHT_WHEEL_ID);
//	backRight = TalonFXInit(&hcan1, BACK_RIGHT_WHEEL_ID);
	bucketDrum = TalonFXInit(&hcan1, BUCKET_DRUM_ID);
	// TODO: apply PID Configs

	leftActuator = TalonSRXInit(&hcan1, LEFT_ACTUATOR_ID);
	rightActuator = TalonSRXInit(&hcan1, RIGHT_ACTUATOR_ID);
	leftActuator.setInverted(&leftActuator, true);
	rightActuator.setInverted(&rightActuator, true);
}

// Given packet from Jetson, set outputs of motors and actuators
void directControl(SerialPacket packet)
{
	// Send global enable frame (so that Talons actively receive CAN packets)
	sendGlobalEnableFrame(&hcan1);

	// Set output speeds of left motors
	uint8_t leftSpeed = packet.top_left_wheel; // a value between 0 and 0xff
	frontLeft.setControl(&frontLeft, leftSpeed, 0); // sets velocity of TalonFX (in turns per second) to leftSpeed
//	backLeft.setControl(&backLeft, leftSpeed, 1);


	// Set output speeds of right motors
	uint8_t rightSpeed = packet.top_right_wheel;
	frontRight.setControl(&frontRight, rightSpeed, 0);
//	backRight.setControl(&backRight, rightSpeed, 1);

	// Set output speed of the bucket drum
	uint8_t bucketDrumSpeed = packet.drum;
	bucketDrum.setControl(&bucketDrum, bucketDrumSpeed, 0);

	// Set outputs of linear actuators
	uint8_t actuatorPosition = packet.actuator;
	float percentExtension = (float) actuatorPosition / 255; // convert value between 0 and 255 to decimal between 0 and 1
	if (DIRECT_ACTUATOR_CONTROL) {
		// this expects packet.actuator to be a two's complement value between 127 and -127
		// eg. a 0xff corresponds to -1/128 = 0.8% output, 0x7F corresponds to 128/128 = 100% output
		float actuatorOutput = ((int8_t) packet.actuator) / 127.0;
//		writeDebugFormat("Actuator Output: %f\r\n", actuatorOutput);
		leftActuator.set(&leftActuator, actuatorOutput);
		rightActuator.set(&rightActuator, actuatorOutput);
	}
	else {
		setActuatorLength(leftActuator, rightActuator, percentExtension);
	}

}
