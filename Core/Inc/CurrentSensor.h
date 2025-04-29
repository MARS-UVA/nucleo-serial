#ifndef INC_CURRENT_SENSOR_H_
#define INC_CURRENT_SENSOR_H_

#include "main.h"
#include "util.h"
#include <stdbool.h>

#define INA219_SAMPLE_COUNT 50

typedef struct currentSensor {
	I2C_HandleTypeDef *hi2c;
	uint16_t address;
	uint8_t rxBuff[2];
	int counter;
	float rmsCurrent;
	float lsb;
	float lastCurrent;
	bool(*tick)(struct currentSensor*);
	float(*read)(struct currentSensor*);
	void(*recv)(struct currentSensor*);
} CurrentSensor;

CurrentSensor CurrentSensorInit(I2C_HandleTypeDef *hi2c, uint16_t devAddress);

#endif
