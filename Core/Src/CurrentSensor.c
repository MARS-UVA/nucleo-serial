/*
 * CurrentSensor.c
 *
 *  Created on: Apr 26, 2025
 *      Author: diana
 */

#include "main.h"
#include "CurrentSensor.h"
#include "debug.h"
#include <math.h>

void writeRegisterCurrentSensor(CurrentSensor *cs, uint8_t registerAddress, uint16_t registerValue)
{
	uint8_t data[3];

	data[0] = registerAddress;		// Register address
	data[1] = registerValue >> 8; 	// MSB of 16 bit data
	data[2] = registerValue;		// LSB of 16 bit data

	HAL_StatusTypeDef hal_status = HAL_I2C_Master_Transmit(cs->hi2c, cs->address << 1, data, 3, 100);
	if (hal_status != HAL_OK)
	{
		writeDebugString("I2C write error\r\n");
	}
}

void readRegisterCurrentSensor(CurrentSensor *cs, uint8_t registerAddress)
{
   HAL_StatusTypeDef hal_status;

   // First send the address that we want to read from to the pointer register
   // using interrupt mode to prevent CPU from blocking
   hal_status = HAL_I2C_Master_Transmit_IT(cs->hi2c, cs->address << 1, &registerAddress, 1); // could be optimized for lower power consumption
   if (hal_status != HAL_OK)
   {
     writeDebugString("I2C write error (register address to read from)\r\n");
   }
}

float readCurrentSensor(CurrentSensor* cs) {
	float curr = cs->rmsCurrent;
	curr /= cs->counter;
	curr = sqrt(curr);

	cs->counter = 0;
	cs->rmsCurrent = 0;

	return curr;
}

bool tickCurrentSensor(CurrentSensor* cs) {
	bool canRead = (++(cs->counter)) >= INA219_SAMPLE_COUNT;

	readRegisterCurrentSensor(cs, 0x04);

	float rawValue = (cs->rxBuff[0] << 8) | cs->rxBuff[1];
	cs->rmsCurrent += pow(rawValue * cs->lsb, 2);

	return canRead;
}

void recvCurrentSensor(CurrentSensor* cs) {
	HAL_I2C_Master_Receive_IT(cs->hi2c, cs->address << 1, cs->rxBuff, 2);
}

// todo: create state that stores current value of potentiometer
CurrentSensor CurrentSensorInit(I2C_HandleTypeDef *hi2c, uint16_t devAddress)
{
	CurrentSensor cs = {
		.hi2c = hi2c,
		.address = devAddress,
		.counter = 0,
		.rxBuff = {0, 0},
		.rmsCurrent = 0,
		.lsb = 0.001,
		.tick = tickCurrentSensor,
		.read = readCurrentSensor,
		.recv = recvCurrentSensor,
	};

	writeRegisterCurrentSensor(&cs, 0x00, 0x399F); // CONFIGURATION

	float shuntResistor = 0.1; // 0.1 ohm 1% sense resistor
	float calibrationValue = 0.04096 / (cs.lsb * shuntResistor); // truncated, refer to data sheet equation
	writeDebugFormat("INA219 Calibration Register Value: 0x%x, 0d%f\r\n\n", calibrationValue);

	writeRegisterCurrentSensor(&cs, 0x05, 0x0199); // CALIBRATION (calibration register value: 0d409.6 --> 0d409 --> 0x0199)

	return cs;
}
