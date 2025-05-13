#include "control.h"

extern CAN_HandleTypeDef hcan1;

TalonFX frontLeft;
TalonFX backLeft;
TalonFX frontRight;
TalonFX backRight;
TalonFX bucketDrum;
TalonFX bucketDrumLeft;
TalonSRX leftActuator;
TalonSRX rightActuator;



// Initialize Talon "objects"
void initializeTalons() {
	frontLeft = TalonFXInit(&hcan1, FRONT_LEFT_WHEEL_ID);
	backLeft = TalonFXInit(&hcan1, BACK_LEFT_WHEEL_ID);
	frontRight = TalonFXInit(&hcan1, FRONT_RIGHT_WHEEL_ID);
	backRight = TalonFXInit(&hcan1, BACK_RIGHT_WHEEL_ID);
	bucketDrum = TalonFXInit(&hcan1, BUCKET_DRUM_ID);
	bucketDrumLeft = TalonFXInit(&hcan1, BUCKET_DRUM_LEFT_ID);

	// TODO: apply PID Configs?
//	struct slot0Configs pidConfigs = {
//		0.25,
//		0.25,
//		0,
//		0,
//		0,
//		0,
//		0
//	};
//	frontLeft.applyConfig(&frontLeft, &pidConfigs);
//	backLeft.applyConfig(&backLeft, &pidConfigs);
//	frontRight.applyConfig(&frontRight, &pidConfigs);
//	backRight.applyConfig(&backRight, &pidConfigs);
//	bucketDrum.applyConfig(&bucketDrum, &pidConfigs);
//	bucketDrumLeft.applyConfig(&bucketDrumLeft, &pidConfigs);
	backLeft.applySupplyCurrentLimit(&backLeft, 120);


	leftActuator = TalonSRXInit(&hcan1, LEFT_ACTUATOR_ID);
	rightActuator = TalonSRXInit(&hcan1, RIGHT_ACTUATOR_ID);
}

// Given packet from Jetson, set outputs of motors and actuators
void directControl(SerialPacket packet, int enableSync)
{
	// Send global enable frame (so that Talons actively receive CAN packets)
	sendGlobalEnableFrame(&hcan1);

	// Set output speeds of left motors
	int8_t leftSpeed = packet.top_left_wheel; // a value between 0 and 0xff (-127 and 128)
	// TODO: check if we want to scale up this value so that we can have a higher max speed (eg. scale up to above -127 and 128)
	// invert because of the way the motors are mounted
	frontLeft.setControl(&frontLeft, ((int8_t)(leftSpeed - 127)) * -1, 0); // sets velocity of TalonFX (in turns per second) to leftSpeed
	backLeft.setControl(&backLeft, ((int8_t)(leftSpeed - 127)) * -1, 0);


	// Set output speeds of right motors
	int8_t rightSpeed = packet.top_right_wheel;
	frontRight.setControl(&frontRight, ((int8_t)(rightSpeed - 127)), 0);
	backRight.setControl(&backRight, ((int8_t)(rightSpeed - 127)), 0);


	// Set output speed of the bucket drum
	int8_t bucketDrumSpeed = packet.drum;
	bucketDrum.setControl(&bucketDrum, ((int8_t)(bucketDrumSpeed - 127)), 0);
	bucketDrumLeft.setControl(&bucketDrumLeft, ((int8_t)(bucketDrumSpeed - 127)) * -1, 0); // todo: talon fx 60 is possibly set to inverted.


	// Set outputs of linear actuators

	// todo: check if packet.actuator is 0xFFs
//    struct ActuatorValues actuatorSyncOutputs; // to store the percent outputs of left and right actuator after synchronization
	if (DIRECT_ACTUATOR_CONTROL) {
//		int actuatorInteger = packet.actuator - 127;
//		writeDebugFormat("Actuator Integer: %d\r\n", actuatorInteger);
		float actuatorOutput = (packet.actuator - 127) / 127.0;



//		if (enableSync == 1) {
//			actuatorSyncOutputs = syncLinearActuators(actuatorOutput);
//			writeDebugFormat("Actuator Output: %f\r\n", actuatorOutput);
//			leftActuator.set(&leftActuator, actuatorSyncOutputs.left);
//			rightActuator.set(&rightActuator, actuatorSyncOutputs.right);
//		}
//
//		else {
		leftActuator.set(&leftActuator, actuatorOutput);
		rightActuator.set(&rightActuator, actuatorOutput);
//		}

	}
	else {
		int8_t actuatorPosition = packet.actuator;
		float percentExtension = (float) actuatorPosition / 255; // convert value between 0 and 255 to decimal between 0 and 1
		setActuatorLength(leftActuator, rightActuator, percentExtension);
	}
}
