// Functions that handle serial communication with Jetson

#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f7xx_hal.h"

//for setting motor speeds
typedef struct serialPacket {
   /* uint8_t opcode;
    uint8_t payload_size;
    uint8_t *payload;
    uint8_t invalid;
    */
  int8_t invalid;
  uint8_t header;
  int8_t top_left_wheel;
  int8_t back_left_wheel;
  int8_t top_right_wheel;
  int8_t back_right_wheel;
  int8_t drum;
  int8_t actuator;
} SerialPacket;

SerialPacket readFromJetson();

//pid
typedef struct serialPacket1 {
   /* uint8_t opcode;
    uint8_t payload_size;
    uint8_t *payload;
    uint8_t invalid;
    */
	int8_t invalid;
	uint8_t header;
	uint8_t CAN_ID;
	uint8_t kP;
	uint8_t kI;
	uint8_t kD;
	uint8_t kS;
	uint8_t kV;
	uint8_t kA;
	uint8_t kG;
} SerialPacketPID;

SerialPacketPID readFromJetsonPID();

//current/voltage limit/rampperiod
typedef struct serialPacket2 {
   /* uint8_t opcode;
    uint8_t payload_size;
    uint8_t *payload;
    uint8_t invalid;
    */
  int8_t invalid;
  uint8_t header;
  uint8_t CAN_ID;
  int8_t value;
} SerialPacketCV;

SerialPacketCV readFromJetsonCV();



void writeToJetson(uint8_t *packet, uint8_t payload_size);


#endif
