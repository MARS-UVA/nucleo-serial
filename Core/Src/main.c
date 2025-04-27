/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

#include "debug.h"
#include "serial.h"
#include "control.h"
#include "TalonFX.h"
#include "PDP.h"
#include "pot.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DEBUG 1
#define DO_CALIBRATE 1

#define OPCODE_STOP 0
#define OPCODE_DIRECT_CONTROL 1
#define OPCODE_PID_CONTROL 2
#define OPCODE_NOP 3


void can_irq(CAN_HandleTypeDef *pcan);

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
PDP pdp;
Pot leftPot;
Pot rightPot;
extern TalonSRX leftActuator;
extern TalonSRX rightActuator;
int enableSync = 0; // todo; receive a value over serial that enables synchronization


uint8_t rx_buff[7];
SerialPacket motorValues = (SerialPacket) {
	.invalid = 0,
	.header = 0x7F,
	.top_left_wheel = 0x7F,
	.back_left_wheel = 0x7F,
	.top_right_wheel  = 0x7F,
	.back_right_wheel = 0x7F,
	.drum  = 0x7F,
	.actuator  = 0x7F,
};
int count = 0;
int actuatorUpCount = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// stop all systems
void stop()
{
	// TODO: Implement
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  writeDebugString("Starting program!\r\n");
  initializeTalons();
  HAL_UART_Receive_IT(&huart6, rx_buff, 7);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  pdp = PDPInit(&hcan1, 62);
  leftPot = PotInit(&hadc1);
  rightPot = PotInit(&hadc2);

  if (DO_CALIBRATE) {
	  // calibration routine: raise actuator all the way up before calibrating
	  for (actuatorUpCount = 0; actuatorUpCount < 800; actuatorUpCount ++) {
		  sendGlobalEnableFrame(&hcan1);
		  leftActuator.set(&leftActuator, 1);
		  rightActuator.set(&rightActuator, 1);
		  HAL_Delay(10);
	  }
	  calibrateYourMom(&leftPot, &rightPot);
  }


  writeDebugString("Entering while loop!\r\n");


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	if (DEBUG){
//		writeDebugString("Running\r\n");
//		writeDebugFormat("Top Left Wheel Output: %d\r\n", motorValues.top_left_wheel);
//		writeDebugFormat("Top Right Wheel Output: %d\r\n", motorValues.top_right_wheel);
//		writeDebugFormat("Track Actuator Position Output: %d\r\n", motorValues.actuator);

//		writeDebugFormat("Front left current: %f\r\n", pdp.getChannelCurrent(&pdp, FRONT_LEFT_WHEEL_PDP_ID));
//		writeDebugFormat("Back left current: %f\r\n", pdp.getChannelCurrent(&pdp, BACK_LEFT_WHEEL_PDP_ID));
//		writeDebugFormat("Drum current: %f\r\n", pdp.getChannelCurrent(&pdp, BUCKET_DRUM_PDP_ID));
		if (count % 20 == 0) {
//			writeDebugFormat("Left Actuator current: %f\r\n", pdp.getChannelCurrent(&pdp, LEFT_ACTUATOR_PDP_ID));
//			writeDebugFormat("Right Actuator current: %f\r\n", pdp.getChannelCurrent(&pdp, RIGHT_ACTUATOR_PDP_ID));
		}
//		writeDebugFormat("Actuator Position: %f %f %f\r\n", leftPot.read(&leftPot), rightPot.read(&rightPot), (leftPot.read(&leftPot) + rightPot.read(&rightPot)) / 2.0);
	}

	// Receive a packet over serial from the Jetson every 10 loops. This is so that it doesn't mess up the CAN bus timing
//	if (count % 5 == 0) {
//		motorValues = readFromJetson(); // receive a packet from Jetson

//		writeDebugFormat("Top Left Wheel Output: %x\r\n", rx_buff[1]);
	count += 1;
	// After a certain period without receiving packets, stop the robot. todo: ensure this logic is robust
	// right now it stops ~2s after we stop sending packets from the Jetson
	if (count > 10) {
		motorValues = (SerialPacket) {
			.invalid = 0,
			.header = 0x7F,
			.top_left_wheel = 0x7F,
			.back_left_wheel = 0x7F,
			.top_right_wheel  = 0x7F,
			.back_right_wheel = 0x7F,
			.drum  = 0x7F,
			.actuator  = 0x7F,
		};
	}

	// testing code
//
//	if (count > 200) {
//		motorValues.actuator = 0x7F;
//	}
//	if (count > 400) {
//		motorValues.actuator = 0xFE;
//	}
//	if (count > 650) {
//		motorValues.actuator = 0x7F;
//	}
//	if (count > 800) {
//		motorValues.actuator = 0x00;
//	}
//	if (count > 1000) {
//		motorValues.actuator = 0x7F;
//	}
//	if (count > 1200) {
//		motorValues.actuator = 0xFE;
//	}
//	if (count > 1450) {
//		motorValues.actuator = 0x7F;
//	}
//	if (count > 1600) {
//		motorValues.actuator = 0x00;
//	}
//	if (count > 1800) {
//		motorValues.actuator = 0x7F;
//	}
//	if (count > 2000) {
//		motorValues.actuator = 0xFE;
//	}
//	if (count > 2250) {
//		motorValues.actuator = 0x7F;
//	}
//	if (count > 2400) {
//		motorValues.actuator = 0x00;
//	}
//	if (count > 2600) {
//		motorValues.actuator = 0x7F;
//	}
//	if (count > 2800) {
//		motorValues.actuator = 0xFE;
//	}
//	if (count > 3050) {
//		motorValues.actuator = 0x7F;
//	}
//	if (count > 3200) {
//		motorValues.actuator = 0x00;
//	}
//	if (count > 3400) {
//		motorValues.actuator = 0x7F;
//	}
//	if (count > 3600) {
//		motorValues.actuator = 0xFE;
//	}
//	if (count > 3850) {
//		motorValues.actuator = 0x7F;
//	}
//	if (count > 4000) {
//		count = 0;
//	}


	// every 10 cycles, poll motor currents and send to Jetson
	if (count % 10 == 0) {


		pdp.requestCurrentReadings(&pdp);

		for (int i = 0; i < 10; i++)
		{
			if (pdp.receivedNew0 && pdp.receivedNew40 && pdp.receivedNew80)
				break;

			HAL_Delay(1);
		}

		float motorCurrents[8];
//		motorCurrents[0] = pdp.getChannelCurrent(&pdp, FRONT_LEFT_WHEEL_PDP_ID);
		motorCurrents[0] = 1;

//		writeDebugFormat("Front left current: %f\r\n", pdp.getChannelCurrent(&pdp, FRONT_LEFT_WHEEL_PDP_ID));
//		motorCurrents[1] = pdp.getChannelCurrent(&pdp, BACK_LEFT_WHEEL_PDP_ID);
		motorCurrents[1] = 2;

//		writeDebugFormat("Front right current: %f\r\n", pdp.getChannelCurrent(&pdp, FRONT_RIGHT_WHEEL_PDP_ID));
		motorCurrents[2] = pdp.getChannelCurrent(&pdp, FRONT_RIGHT_WHEEL_PDP_ID);
//		writeDebugFormat("Back left current: %f\r\n", pdp.getChannelCurrent(&pdp, BACK_LEFT_WHEEL_PDP_ID));
		motorCurrents[3] = pdp.getChannelCurrent(&pdp, BACK_RIGHT_WHEEL_PDP_ID);
//		writeDebugFormat("Back right current: %f\r\n", pdp.getChannelCurrent(&pdp, BACK_RIGHT_WHEEL_PDP_ID));
		motorCurrents[4] = pdp.getChannelCurrent(&pdp, BUCKET_DRUM_PDP_ID);

		motorCurrents[5] = pdp.getChannelCurrent(&pdp, LEFT_ACTUATOR_PDP_ID);
		motorCurrents[6] = pdp.getChannelCurrent(&pdp, RIGHT_ACTUATOR_PDP_ID);

		motorCurrents[7] = (leftPot.read(&leftPot) + rightPot.read(&rightPot)) / 2.0;

		pdp.receivedNew0 = false;
		pdp.receivedNew40 = false;
		pdp.receivedNew80 = false;

		motorCurrents[7] = 0;

//		// convert floats to bytes, put in packet
	    uint8_t packet[4 + 4 * 8];  // 4-byte header + 5 floats × 4 bytes
	    packet[0] = 0x1; // header 0x1 to indicate motor current feedback
	    for (int i = 0; i < 8; i++) {
	        floatToByteArray(motorCurrents[i], &packet[4 + i * 4]);
//	        writeDebugFormat("b1: %d\r\n", packet[1]);
//	        writeDebugFormat("b2: %d\r\n", packet[2]);
//	        writeDebugFormat("b3: %d\r\n", packet[3]);
//			writeDebugFormat("b4: %d\r\n", packet[4]);

	    }
////		//send packet to Jetson
	    writeToJetson(packet, 4 + 4 * 8);
	}



//	}

	directControl(motorValues, enableSync); // set motor outputs accordingly
	HAL_Delay(1);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef sf;
  sf.FilterMaskIdHigh = 0x0000;
  sf.FilterMaskIdLow = 0x0000;
  sf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sf.FilterBank = 0;
  sf.FilterMode = CAN_FILTERMODE_IDMASK;
  sf.FilterScale = CAN_FILTERSCALE_32BIT;
  sf.FilterActivation = CAN_FILTER_ENABLE;
  if (HAL_CAN_ConfigFilter(&hcan1, &sf) != HAL_OK)
	Error_Handler();
//
  if (HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, can_irq))
	Error_Handler();
//
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
	Error_Handler();
//
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	Error_Handler();
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void can_irq(CAN_HandleTypeDef *pcan)
{
  CAN_RxHeaderTypeDef msg;
  uint64_t data;
  HAL_CAN_GetRxMessage(pcan, CAN_RX_FIFO0, &msg, (uint8_t *) &data);
  if (pdp.receiveCAN)
	  pdp.receiveCAN(&pdp, &msg, &data);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (HAL_UART_Receive_IT(&huart6, rx_buff, 7) != HAL_OK) {
		writeDebugString("ERROR OCCURED DURING UART RX INTERRUPT");
	}

	count = 0;
	motorValues = (SerialPacket) {
		.invalid = 0,
		.header = rx_buff[0],
		.top_left_wheel = rx_buff[1],
		.back_left_wheel = rx_buff[2],
		.top_right_wheel  = rx_buff[3],
		.back_right_wheel = rx_buff[4],
		.drum  = rx_buff[5],
		.actuator  = rx_buff[6],
	};
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
