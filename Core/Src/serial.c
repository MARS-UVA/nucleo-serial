#include "serial.h"
#include "debug.h"

extern UART_HandleTypeDef huart2;

// reads a single packet from Jetson on UART 2
SerialPacket readFromJetson() {
	uint8_t RxBuffer[7]; // buffer to store received bytes
	uint8_t packetLength = 7; // expected packet length (1 header plus 1 byte for each motor/actuator)

	HAL_StatusTypeDef hal_status = HAL_UART_Receive(&huart2, RxBuffer, packetLength, 0xFFFFFF); // making delay large seems to break CAN communication
	if (hal_status != HAL_OK) {
//		writeDebugString("Error during UART Receive");
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

void writeToJetson(uint8_t *data, uint8_t payload_size)
{
	// needs to be implemented

//
//	uint8_t length = payload_size + 3;
//	uint8_t *encoded = malloc(length);
//	encoded[0] = 0xff;
//	encoded[1] = payload_size | 0xc0;
//	memcpy(encoded+2, data, payload_size);
//	encoded[length - 1] = calculateChecksum(data, payload_size);
//
//	HAL_StatusTypeDef HALStatus = HAL_UART_Transmit(&huart2, encoded, length, HAL_MAX_DELAY);
//
//	if (HALStatus != HAL_OK)
//		Error_Handler();
//
//	free(encoded);
}
