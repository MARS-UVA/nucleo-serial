#include "control.h"

extern CAN_HandleTypeDef hcan1;

TalonFX frontLeft;
TalonFX backLeft;
TalonFX frontRight;
TalonFX backRight;
TalonFX backDrum;
TalonFX frontDrum;
TalonFX frontArm;
TalonFX backArm;



// Initialize Talon "objects"
void initializeTalons() {
	frontLeft = TalonFXInit(&hcan1, FRONT_LEFT_WHEEL_ID);
	backLeft = TalonFXInit(&hcan1, BACK_LEFT_WHEEL_ID);
	frontRight = TalonFXInit(&hcan1, FRONT_RIGHT_WHEEL_ID);
	backRight = TalonFXInit(&hcan1, BACK_RIGHT_WHEEL_ID);
	backDrum = TalonFXInit(&hcan1, BACK_DRUM_ID);
	frontDrum = TalonFXInit(&hcan1, FRONT_DRUM_ID);
	frontArm = TalonFXInit(&hcan1, FRONT_ARM_ID);
	backArm = TalonFXInit(&hcan1, BACK_ARM_ID);
}

// Given packet from Jetson, set outputs of motors and actuators
void directControl(SerialPacket packet, int enableSync)
{
	// Send global enable frame (so that Talons actively receive CAN packets)
	sendGlobalEnableFrame(&hcan1);

	// Set output speeds of left motors
	int8_t leftSpeed = packet.front_left_wheel; // a value between 0 and 0xff (-127 and 128)
	// invert because of the way the motors are mounted
	frontLeft.setControl(&frontLeft, ((int8_t)(leftSpeed - 127)) * -1, 0); // sets velocity of TalonFX (in turns per second) to leftSpeed
	backLeft.setControl(&backLeft, ((int8_t)(leftSpeed - 127)) * -1, 0);


	// Set output speeds of right motors
	int8_t rightSpeed = packet.front_right_wheel;
	frontRight.setControl(&frontRight, ((int8_t)(rightSpeed - 127)), 0);
	backRight.setControl(&backRight, ((int8_t)(rightSpeed - 127)), 0);

	// Set output speeds of the bucket drum motors
	int8_t backDrumSpeed = packet.back_drum;
	backDrum.setControl(&backDrum, ((int8_t)(backDrumSpeed - 127)), 0);
	int8_t frontDrumSpeed = packet.front_drum;
	frontDrum.setControl(&frontDrum, ((int8_t)(frontDrumSpeed - 127)), 0);


	// Set output speeds of the arms
	int8_t frontArmSpeed = packet.front_arm;
	frontArm.setControl(&frontArm, ((int8_t)(frontArmSpeed - 127)), 0);
	int8_t backArmSpeed = packet.back_arm;
	backArm.setControl(&backArm, ((int8_t)(backArmSpeed - 127)), 0);
}
