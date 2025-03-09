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
#define FRONT_LEFT_WHEEL_ID 0
#define BACK_LEFT_WHEEL_ID 1
#define FRONT_RIGHT_WHEEL_ID 2
#define BACK_RIGHT_WHEEL_ID 3
#define BUCKET_DRUM_ID 4
#define LEFT_ACTUATOR_ID 5
#define RIGHT_ACTUATOR_ID 6


void directControl(SerialPacket packet);


#endif
