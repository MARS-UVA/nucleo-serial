#include "actuatorControl.h"
#include "pot.h"
#include "debug.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern Pot leftPot;
extern Pot rightPot;


#define FULL_ADC_RANGE 4096 // todo: check if this value is right
#define POSITION_TOLERANCE 0.05
#define TOLERANCE 1.05
#define SLOWFACTOR 0.90


// sets actuator length for a length between 0 (fully retracted) and 1 (fully extended)
void setActuatorLength(TalonSRX leftActuator, TalonSRX rightActuator, float percentExtension) {
	  float leftPosition = leftPot.read(&leftPot); // Change to HAL_ADCEx_MultiModeStart_DMA or HAL_ADC_Start_DMA for speed
	  float rightPosition = rightPot.read(&rightPot);

	  struct ActuatorValues percentOutputs;
	  uint16_t targetPosition = percentExtension * FULL_ADC_RANGE;
//	  writeDebugFormat("Current position: %d\r\n", leftPosition);
//	  writeDebugFormat("Target position: %d\r\n", targetPosition);
	  // if current actuator lengtht is approximately equal to target length, set Talon SRX outputs to 0
	  if (leftPosition < targetPosition + POSITION_TOLERANCE && leftPosition > targetPosition - POSITION_TOLERANCE) {
		  percentOutputs = (struct ActuatorValues) {
			.left = 0,
			.right = 0,
		  };
	  }
	  else
		  if (leftPosition < targetPosition) {
		  percentOutputs = syncLinearActuators(1, leftPosition, rightPosition); // extend actuators at full speed
	  }
	  else {
		  percentOutputs = syncLinearActuators(-1, leftPosition, rightPosition); // retract actuators at full speed
	  }
	  leftActuator.set(&leftActuator, percentOutputs.left);
	  rightActuator.set(&rightActuator, percentOutputs.right);
}

// given target actuator speed, use potentiometer feedback to ensure actuators are at approximately the same length of extension
// if one actuator is ahead of the other, slow down the faster actuator by a set speed
// returns percent outputs of the TalonSRXs controlling the actuators
struct ActuatorValues syncLinearActuators(float percentOutput, float leftPosition, float rightPosition) {
  struct ActuatorValues newPercentOutputs;
  newPercentOutputs.left = percentOutput;
  newPercentOutputs.right = percentOutput;

  // if difference between left and right actuator lengths are larger than a certain tolerance, slow down faster actuator
  if ((leftPosition > rightPosition * TOLERANCE && percentOutput > 0) || (leftPosition < rightPosition * TOLERANCE && percentOutput < 0)) {
    newPercentOutputs.left *= SLOWFACTOR;
  } else if ((rightPosition > leftPosition * TOLERANCE && percentOutput > 0) || (rightPosition < leftPosition * TOLERANCE && percentOutput < 0)) {
    newPercentOutputs.right *= SLOWFACTOR;
  }

//  writeDebugFormat("Left Actuator new output: %f\r\n", newPercentOutputs.left);
//  writeDebugFormat("Right Actuator new output: %f\r\n", newPercentOutputs.right);

  return newPercentOutputs;
}

// Returns weight of material in bucket drum (in kg) given a the total current drawn by the 2 actuators
float currentToWeight(float totalCurrent) {
	return 4.315 * totalCurrent - 14.18;
}

