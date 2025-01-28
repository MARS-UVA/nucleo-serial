/*
 * TalonFX.h
 *
 *  Created on: Jan 17, 2025
 *      Author: diana
 */

#ifndef INC_TALONFX_H_
#define INC_TALONFX_H_

#include "main.h"

typedef enum neutralModeValue {
	COAST = 0,
	BRAKE = 1,
} NeutralModeValue;

typedef struct talonFX {
	CAN_HandleTypeDef *hcan;
	int identifier;
	void (*set)(struct talonFX*, double);
	void(*setNeutralMode)(struct talonFX*, enum neutralModeValue);
	void(*applySupplyCurrentLimit)(struct talonFX*, int);
} TalonFX;

TalonFX TalonFXInit();

#endif /* INC_TALONFX_H_ */
