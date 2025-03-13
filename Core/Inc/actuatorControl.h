/*
 * actuatorControl
 *
 *  Created on: Mar 4, 2025
 *      Author: rojo
*/

// Functions for synchronizing and controlling track actuators

#ifndef __ACTUATOR_CONTROL_H
#define __ACTUATOR_CONTROL_H

#include "stm32f7xx_hal.h"
#include "TalonSRX.h"
#include "debug.h"

#define TOLERANCE 1.004
#define SLOWFACTOR 0.99


// percent output values for left and right actuators
struct ActuatorValues {
  float left;
  float right;
};


void setActuatorLength(TalonSRX leftActuator, TalonSRX rightActuator, float percentExtension);
struct ActuatorValues syncLinearActuators(float percentOutput);

#endif
