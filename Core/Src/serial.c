#include "serial.h"
#include "debug.h"

extern UART_HandleTypeDef huart6;

// reads a single packet from Jetson on UART 6
SerialPacket readFromJetson() {
	uint8_t RxBuffer[7]; // buffer to store received bytes
	uint8_t packetLength = 7; // expected packet length (1 header plus 1 byte for each motor/actuator)

	HAL_StatusTypeDef hal_status = HAL_UART_Receive(&huart6, RxBuffer, packetLength, 0xFFFFFF); // making delay large seems to break CAN communication
	if (hal_status != HAL_OK) {
		writeDebugFormat("Error during UART Receive: %d\r\n");
	    return (SerialPacket) {
	        .invalid = 1
	    };
	}

	return (SerialPacket) {
		.invalid = 0,
		.header = RxBuffer[0],
		.top_left_wheel = RxBuffer[1],
		.back_left_wheel = RxBuffer[2],
		.top_right_wheel  = RxBuffer[3],
		.back_right_wheel = RxBuffer[4],
		.drum  = RxBuffer[5],
		.actuator  = RxBuffer[6],
	};
}

// writes a single packet to Jetson on UART 6
void writeToJetson(uint8_t *packet, uint8_t payload_size)
{
	HAL_UART_Transmit_IT(&huart6, packet, payload_size);
}
