#include "actuatorControl.h"
#include "pot.h"
#include "debug.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern Pot leftPot;
extern Pot rightPot;


#define FULL_ADC_RANGE 4096 // todo: check if this value is right
#define POSITION_TOLERANCE 10
#define TOLERANCE 1.01
#define SLOWFACTOR 0.98


// sets actuator length for a length between 0 (fully retracted) and 1 (fully extended)
void setActuatorLength(TalonSRX leftActuator, TalonSRX rightActuator, float percentExtension) {
	// Get current length of actuators
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_Start(&hadc2);
	  HAL_ADC_PollForConversion(&hadc1, 20);
	  uint16_t leftPosition = HAL_ADC_GetValue(&hadc1); // Change to HAL_ADCEx_MultiModeStart_DMA or HAL_ADC_Start_DMA for speed
	  HAL_ADC_PollForConversion(&hadc2, 20);
	  uint16_t rightPosition = HAL_ADC_GetValue(&hadc2); // todo: also check rightPosition once we know this logic works.

	  struct ActuatorValues percentOutputs;
	  uint16_t targetPosition = percentExtension * FULL_ADC_RANGE;
//	  writeDebugFormat("Current position: %d\r\n", leftPosition);
//	  writeDebugFormat("Target position: %d\r\n", targetPosition);
	  // if current actuator length is approximately equal to target length, set Talon SRX outputs to 0
	  if (leftPosition < targetPosition + POSITION_TOLERANCE && leftPosition > targetPosition - POSITION_TOLERANCE) {
		  percentOutputs = (struct ActuatorValues) {
			.left = 0,
			.right = 0,
		  };
	  }
	  else
		  if (leftPosition < targetPosition) {
		  percentOutputs = syncLinearActuators(1); // extend actuators at full speed
	  }
	  else {
		  percentOutputs = syncLinearActuators(-1); // retract actuators at full speed
	  }
	  leftActuator.set(&leftActuator, percentOutputs.left);
	  rightActuator.set(&rightActuator, percentOutputs.right);
}

// given target actuator speed, use potentiometer feedback to ensure actuators are at approximately the same length of extension
// if one actuator is ahead of the other, slow down the faster actuator by a set speed
// returns percent outputs of the TalonSRXs controlling the actuators
struct ActuatorValues syncLinearActuators(float percentOutput) {
  float leftPosition = leftPot.read(&leftPot); // Change to HAL_ADCEx_MultiModeStart_DMA or HAL_ADC_Start_DMA for speed
  float rightPosition = rightPot.read(&rightPot);

  struct ActuatorValues newPercentOutputs;
  newPercentOutputs.left = percentOutput;
  newPercentOutputs.right = percentOutput;

  // if difference between left and right actuator lengths are larger than a certain tolerance, slow down faster actuator
  if ((leftPosition > rightPosition * TOLERANCE && percentOutput > 0) || (leftPosition < rightPosition * TOLERANCE && percentOutput < 0)) {
    newPercentOutputs.left *= SLOWFACTOR;
  } else if ((rightPosition > leftPosition * TOLERANCE && percentOutput > 0) || (rightPosition < leftPosition * TOLERANCE && percentOutput < 0)) {
    newPercentOutputs.right *= SLOWFACTOR;
  }
  writeDebugFormat("Left Actuator new output: %f\r\n", newPercentOutputs.left);
  writeDebugFormat("Right Actuator new output: %f\r\n", newPercentOutputs.right);

  return newPercentOutputs;
}
