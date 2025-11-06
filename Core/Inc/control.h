// Functions that handle control communication with Jetson

#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f7xx_hal.h"
#include "serial.h"
#include "TalonSRX.h"
#include "TalonFX.h"
#include "debug.h"
#include "can.h"


// Define CAN IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_ID 36
#define BACK_LEFT_WHEEL_ID 37
#define FRONT_RIGHT_WHEEL_ID 38
#define BACK_RIGHT_WHEEL_ID 13
#define BUCKET_DRUM_ID 25
#define BUCKET_DRUM_LEFT_ID 60
#define LEFT_ACTUATOR_ID 0
#define RIGHT_ACTUATOR_ID 1

// Define PDP IDs of each motor
#define FRONT_LEFT_WHEEL_PDP_ID 12
#define BACK_LEFT_WHEEL_PDP_ID 13
#define FRONT_RIGHT_WHEEL_PDP_ID 3
#define BACK_RIGHT_WHEEL_PDP_ID 1
#define BUCKET_DRUM_PDP_ID 0
#define BUCKET_DRUM_LEFT_PDP_ID 2
#define LEFT_ACTUATOR_PDP_ID 15
#define RIGHT_ACTUATOR_PDP_ID 14


void directControl(SerialPacket packet, int enableSync);
void initializeTalons();


#endif
