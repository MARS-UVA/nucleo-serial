/*
 * TalonFX.h
 *
 *  Created on: Jan 17, 2025
 *      Author: diana
 */

#ifndef INC_TALONFX_H_
#define INC_TALONFX_H_

#include "main.h"
#include "util.h"
#include "can.h"
#include <math.h>


typedef enum neutralModeValue {
	COAST = 0,
	BRAKE = 1,
} NeutralModeValue;

typedef struct slot0Configs {
	double kP;
	double kI;
	double kD;
	double kS;
	double kV;
	double kA;
	double kG;
} Slot0Configs;

typedef struct talonFX {
	CAN_HandleTypeDef *hcan;
	int identifier;
	void (*set)(struct talonFX*, double);
	void(*setNeutralMode)(struct talonFX*, NeutralModeValue);
	void(*applySupplyCurrentLimit)(struct talonFX*, float);
	void(*applyConfig)(struct talonFX*, Slot0Configs*);
	void(*setControl)(struct talonFX*, int, double);
	void(*voltageCycleClosedLoopRampPeriod) (struct talonFX*, float);
} TalonFX;

TalonFX TalonFXInit();

#endif /* INC_TALONFX_H_ */
