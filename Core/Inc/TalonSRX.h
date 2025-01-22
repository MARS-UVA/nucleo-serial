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

typedef struct talonSRX {
	CAN_HandleTypeDef *hcan;
	int identifier;
	void (*setInverted)(struct talonSRX*, bool);
	void (*set)(struct talonSRX*, double);
} TalonSRX;

TalonSRX TalonSRXInit();


#endif /* INC_TALONFX_H_ */
