#include "control.h"

extern CAN_HandleTypeDef hcan1;

TalonFX frontLeft;
TalonFX backLeft;
TalonFX frontRight;
TalonFX backRight;
TalonFX rightDrum;
TalonFX leftDrum;
TalonFX leftArm;
TalonFX rightArm;



// Initialize Talon "objects"
void initializeTalons() {
	frontLeft = TalonFXInit(&hcan1, FRONT_LEFT_WHEEL_ID);
	backLeft = TalonFXInit(&hcan1, BACK_LEFT_WHEEL_ID);
	frontRight = TalonFXInit(&hcan1, FRONT_RIGHT_WHEEL_ID);
	backRight = TalonFXInit(&hcan1, BACK_RIGHT_WHEEL_ID);
	rightDrum = TalonFXInit(&hcan1, RIGHT_DRUM_ID);
	leftDrum = TalonFXInit(&hcan1, LEFT_DRUM_ID);
	leftArm = TalonFXInit(&hcan1, LEFT_ARM_ID);
	rightArm = TalonFXInit(&hcan1, RIGHT_ARM_ID);
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
	int8_t rightDrumSpeed = packet.right_drum;
	rightDrum.setControl(&rightDrum, ((int8_t)(rightDrumSpeed - 127)), 0);
	int8_t leftDrumSpeed = packet.left_drum;
	leftDrum.setControl(&leftDrum, ((int8_t)(leftDrumSpeed - 127)), 0);


	// Set output speeds of the arms
	int8_t leftArmSpeed = packet.left_arm;
	leftArm.setControl(&leftArm, leftArmSpeed, 0);
	int8_t rightArmSpeed = packet.right_arm;
	rightArm.setControl(&rightArm, rightArmSpeed, 0);
}
