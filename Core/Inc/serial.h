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
  int8_t top_left_wheel;
  int8_t back_left_wheel;
  int8_t top_right_wheel;
  int8_t back_right_wheel;
  int8_t drum;
  int8_t actuator;
} SerialPacket;

SerialPacket readFromJetson();
void writeToJetson(uint8_t *data, uint8_t payload_size);


#endif
