// Functions that handle serial communication with Jetson

#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f7xx_hal.h"

typedef struct serialPacket {
   /* uint8_t opcode;
    uint8_t payload_size;
    uint8_t *payload;
    uint8_t invalid;
    */
  int8_t invalid;
  uint8_t header;
  uint8_t top_left_wheel;
  uint8_t back_left_wheel;
  uint8_t top_right_wheel;
  uint8_t back_right_wheel;
  uint8_t drum;
  uint8_t actuator;
} SerialPacket;

SerialPacket readFromJetson();
void writeToJetson(uint8_t *packet, uint8_t payload_size);


#endif
