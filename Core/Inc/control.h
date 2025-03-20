// Functions that handle control communication with Jetson

#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f7xx_hal.h"
#include "serial.h"
#include "TalonSRX.h"
#include "TalonFX.h"
#include "debug.h"
#include "can.h"
#include "actuatorControl.h"


// Define CAN IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_ID 36
#define BACK_LEFT_WHEEL_ID 37
#define FRONT_RIGHT_WHEEL_ID 38
#define BACK_RIGHT_WHEEL_ID 39
#define BUCKET_DRUM_ID 25
#define LEFT_ACTUATOR_ID 0
#define RIGHT_ACTUATOR_ID 1

#define DIRECT_ACTUATOR_CONTROL 1 // 1 if we want to set the outputs of the Talon SRXs directly and bypass the potentiometer feedback


void directControl(SerialPacket packet);
void initializeTalons();


#endif
