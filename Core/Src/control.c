#include "control.h"
#include "debug.h"

// Given packet from Jetson, set outputs of motors and actuators
void directControl(SerialPacket packet)
{

	// Set outputs of left motors
	uint8_t leftSpeed = packet.top_left_wheel;

	// Set outputs of right motors
	uint8_t rightSpeed = packet.top_right_wheel;

	// Set outputs of linear actuators
	uint8_t actuatorPosition = packet.actuator;

//	for (int i = 0; i < packet.payload_size; i++)
//	{
//		float command = ((int) packet.payload[i] - 100) / 100.0f;
//
//		switch (i)
//		{
//		case 0:
//			directDriveLeft(command * DRIVETRAIN_SCALE, CONTROL_UPPER_BOUND);
//
//			sprintf(debug_buffer, "Front left: %.6f\r\n", command);
//			writeDebug(debug_buffer, strlen(debug_buffer));
////			writeDebugFormat("Front left: %.6f", command);
//			break;
//		case 1:
//			directDriveRight(command * DRIVETRAIN_SCALE, CONTROL_UPPER_BOUND);
//
//			sprintf(debug_buffer, "Front right: %.6f\r\n", command);
//			writeDebug(debug_buffer, strlen(debug_buffer));
////			writeDebugFormat("Front right: %.6f", command);
//			break;
//		case 2:
//			directDriveLeft(command * DRIVETRAIN_SCALE, CONTROL_UPPER_BOUND);
//
//			sprintf(debug_buffer, "Back left: %.6f\r\n", command);
//			writeDebug(debug_buffer, strlen(debug_buffer));
////			writeDebugFormat("Back left: %.6f", command);
//			break;
//		case 3:
//			directDriveRight(command * DRIVETRAIN_SCALE, CONTROL_UPPER_BOUND);
//
//			sprintf(debug_buffer, "Back left: %.6f\r\n", command);
//			writeDebug(debug_buffer, strlen(debug_buffer));
////			writeDebugFormat("Back right: %.6f", command);
//			break;
//		default:
//			writeDebugString("Unimplemented command written!\r\n");
//			break;
//		}
//	}
}
