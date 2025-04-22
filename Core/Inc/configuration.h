// Functions that handle control communication with Jetson
#ifndef __CONFIGURATION_H
#define __CONFIGURATION_H

#include "stm32f7xx_hal.h"
#include "serial.h"
#include "TalonSRX.h"
#include "TalonFX.h"
#include "debug.h"
#include "can.h"
#include "actuatorControl.h"


void pid_config(SerialPacketPID packet);
void voltageCycleClosedLoopRampPeriod_config(SerialPacketCV packet);
void appleSupplyCurrentLimit_config(SerialPacketCV packet);

#endif
