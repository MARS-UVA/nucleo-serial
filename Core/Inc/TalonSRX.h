/*
 * TalonFX.h
 *
 *  Created on: Jan 17, 2025
 *      Author: diana
 */

#ifndef INC_TALONSRX_H_
#define INC_TALONSRX_H_

#include <stdbool.h>
#include "main.h"
#include "util.h"
#include "can.h"
#include <math.h>

typedef struct talonSRX {
	CAN_HandleTypeDef *hcan;
	int identifier;
	bool inverted;
	void (*setInverted)(struct talonSRX*, bool);
	void (*set)(struct talonSRX*, double);
} TalonSRX;

TalonSRX TalonSRXInit();


#endif /* INC_TALONFX_H_ */
