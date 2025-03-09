// Functions that handle control communication with Jetson

#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f7xx_hal.h"
#include "serial.h"


void directControl(SerialPacket packet);


#endif
