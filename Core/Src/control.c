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
	backLeft = TalonFXInit(&hcan1, BACK_LEFT_WHEEL_ID);
	frontRight = TalonFXInit(&hcan1, FRONT_RIGHT_WHEEL_ID);
	backRight = TalonFXInit(&hcan1, BACK_RIGHT_WHEEL_ID);
	bucketDrum = TalonFXInit(&hcan1, BUCKET_DRUM_ID);
	// todo: find optimal PID paameters
	struct slot0Configs pidConfigs = {
		0.25,
		0,
		0,
		0,
		0,
		0,
		0
	};
	frontLeft.applyConfig(&frontLeft, &pidConfigs);
	backLeft.applyConfig(&backLeft, &pidConfigs);
	frontRight.applyConfig(&frontRight, &pidConfigs);
	backRight.applyConfig(&backRight, &pidConfigs);
	bucketDrum.applyConfig(&bucketDrum, &pidConfigs);

	leftActuator = TalonSRXInit(&hcan1, LEFT_ACTUATOR_ID);
	rightActuator = TalonSRXInit(&hcan1, RIGHT_ACTUATOR_ID);
}

// Given packet from Jetson, set outputs of motors and actuators
void directControl(MotorValues packet)
{
	// Send global enable frame (so that Talons actively receive CAN packets)
	sendGlobalEnableFrame(&hcan1);

	// Get motor values
	uint8_t leftSpeed = packet.front_left_wheel; // a value between 0 and 0xff (-127 and 128)
	if (leftSpeed == 0xFF) {
		leftSpeed = leftSpeed - 1; // edge case: 0xFF - 127 evaluates to 128, which overflows and becomes -1
	}
	uint8_t rightSpeed = packet.front_right_wheel;
	if (rightSpeed == 0xFF) {
		rightSpeed = rightSpeed - 1; // edge case: 0xFF - 127 evaluates to 128, which overflows and becomes -1
	}
	int8_t bucketDrumSpeed = packet.drum;
	if (bucketDrumSpeed == 0xFF) {
		bucketDrumSpeed = bucketDrumSpeed - 1; // edge case: 0xFF - 127 evaluates to 128, which overflows and becomes -1
	}
	// TODO: consider scaling up speed values because motors can go faster than 127 turns per second
	frontLeft.setControl(&frontLeft, ((int8_t)(leftSpeed - 127)) * -1, 0); // invert left motors because of the way they're mounted
	backLeft.setControl(&backLeft,((int8_t)(leftSpeed - 127)) * -1, 0); // invert left motors
	frontRight.setControl(&frontRight, ((int8_t)(rightSpeed - 127)), 0);
	backRight.setControl(&backRight, ((int8_t)(rightSpeed - 127)), 0);
	bucketDrum.setControl(&bucketDrum, ((int8_t)(bucketDrumSpeed - 127)), 0);

	// Set outputs of linear actuators
	if (DIRECT_ACTUATOR_CONTROL) { // sets output speed of actuators based on packet value
		// if actuator value is positive, set actuator output to 80% of full output upwards
		float actuatorOutput = 0;
		if (packet.actuator > 127) {
			actuatorOutput = 0.8;
		}
		else if (packet.actuator < 127) { // if actuator value is negative, set actuator output to 80% of full output downwards
			actuatorOutput = -0.8;
		}
		rightActuator.set(&rightActuator, actuatorOutput);
		leftActuator.set(&leftActuator, actuatorOutput);
	}
	else {
		float percentExtension = (float) packet.actuator / 255; // convert byte (0-255) to decimal (0 - 1.0)
		setActuatorLength(leftActuator, rightActuator, percentExtension); // set actuator positions using potentiometer feedback
	}

}
