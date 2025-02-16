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
#include "TalonSRX.h"
#include "TalonFX.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG 1

#define OPCODE_STOP 0
#define OPCODE_DIRECT_CONTROL 1
#define OPCODE_PID_CONTROL 2
#define OPCODE_NOP 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void can_irq(CAN_HandleTypeDef *pcan);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct serialPacket {
	uint8_t opcode;
	uint8_t payload_size;
	uint8_t *payload;
	uint8_t invalid;
} SerialPacket;

uint8_t calculateChecksum(uint8_t *buffer, uint8_t length)
{
	uint8_t checksum = 0;
	for (int i = 0; i < length; i++)
		checksum += buffer[i];
	return checksum;
}

void writeDebug(const char *buffer, uint8_t length)
{
	HAL_UART_Transmit(&huart3, (uint8_t *) buffer, length, HAL_MAX_DELAY);
}

void writeDebugString(const char *buffer)
{
	writeDebug(buffer, strlen(buffer));
}

void writeDebugFormat(const char *format, ...)
{
	va_list args;
	va_start(args, format);

	va_list args_copy;
	va_copy(args_copy, args);
	int buff_size = vsnprintf(NULL, 0, format, args_copy);
	va_end(args_copy);

	char *buff = malloc(buff_size + 1);

	if (buff == NULL)
	{
		va_end(args);
		return;
	}

	vsnprintf(buff, buff_size + 1, format, args);
	writeDebug(buff, buff_size);
	free(buff);

	va_end(args);
}

SerialPacket readFromJetson()
{
	uint8_t read;
	HAL_StatusTypeDef hal_status;

	// read header
	hal_status = HAL_UART_Receive(&huart2, &read, 1, HAL_MAX_DELAY);

	if (hal_status != HAL_OK)
	{
//		Error_Handler();
		return (SerialPacket) {
			.invalid = 1
		};
	}

	// check header valid
	if (read != 0xff)
	{
		writeDebugFormat("Read invalid header: %x\r\n", &read);

//		Error_Handler();
		return (SerialPacket) {
			.invalid = 1
		};
	}

	if (DEBUG)
	{
		writeDebugString("new packet started\r\n");
	}
	// read payload size
	hal_status = HAL_UART_Receive(&huart2, &read, 1, HAL_MAX_DELAY);

	if (hal_status != HAL_OK)
	{
//		Error_Handler();
		return (SerialPacket) {
			.invalid = 1
		};
	}


	uint8_t opcode = (read & 0xc0) >> 6;
	uint8_t payload_size = read & 0x3f;

	// read payload
	uint8_t *buffer = malloc(payload_size);

	hal_status = HAL_UART_Receive(&huart2, buffer, payload_size, HAL_MAX_DELAY);

	if (hal_status != HAL_OK)
	{
//		Error_Handler();
		return (SerialPacket) {
			.invalid = 1
		};
	}
	// read checksum
	hal_status = HAL_UART_Receive(&huart2, &read, 1, HAL_MAX_DELAY);

	if (hal_status != HAL_OK)
	{
//		Error_Handler();
		return (SerialPacket) {
			.invalid = 1
		};
	}

	// check checksum
	if (calculateChecksum(buffer, payload_size) != read)
	{
//		Error_Handler();
		return (SerialPacket) {
			.invalid = 1
		};
	}

	// print whatever was read to the serial console
	if (DEBUG)
	{
		writeDebugString("Buffer read: ");
		writeDebug((char *) buffer, payload_size);
		writeDebugString("\r\n");
	}

	return (SerialPacket) {
		.opcode = opcode,
		.payload_size = payload_size,
		.payload = buffer,
		.invalid = 0
	};
}

void directDriveLeft(float power, float upperbound) {}
void directDriveRight(float power, float upperbound) {}

// stop all systems
void stop()
{
	// TODO: Implement
}

#define CONTROL_UPPER_BOUND 1.0f
#define DRIVETRAIN_SCALE 0.5f

#define DEBUG_BUFFER_LENGTH 64

void directControl(SerialPacket packet)
{
	char debug_buffer[DEBUG_BUFFER_LENGTH] = {0};
	for (int i = 0; i < packet.payload_size; i++)
	{
		float command = ((int) packet.payload[i] - 100) / 100.0f;

		switch (i)
		{
		case 0:
			directDriveLeft(command * DRIVETRAIN_SCALE, CONTROL_UPPER_BOUND);

			sprintf(debug_buffer, "Front left: %.6f\r\n", command);
			writeDebug(debug_buffer, strlen(debug_buffer));
//			writeDebugFormat("Front left: %.6f", command);
			break;
		case 1:
			directDriveRight(command * DRIVETRAIN_SCALE, CONTROL_UPPER_BOUND);

			sprintf(debug_buffer, "Front right: %.6f\r\n", command);
			writeDebug(debug_buffer, strlen(debug_buffer));
//			writeDebugFormat("Front right: %.6f", command);
			break;
		case 2:
			directDriveLeft(command * DRIVETRAIN_SCALE, CONTROL_UPPER_BOUND);

			sprintf(debug_buffer, "Back left: %.6f\r\n", command);
			writeDebug(debug_buffer, strlen(debug_buffer));
//			writeDebugFormat("Back left: %.6f", command);
			break;
		case 3:
			directDriveRight(command * DRIVETRAIN_SCALE, CONTROL_UPPER_BOUND);

			sprintf(debug_buffer, "Back left: %.6f\r\n", command);
			writeDebug(debug_buffer, strlen(debug_buffer));
//			writeDebugFormat("Back right: %.6f", command);
			break;
		default:
			writeDebugString("Unimplemented command written!\r\n");
			break;
		}
	}
}

void pidControl(SerialPacket packet)
{
	// TODO: Implement
}

void readAction(SerialPacket packet)
{
	switch (packet.opcode)
	{
	case OPCODE_STOP:
		stop();
		break;
	case OPCODE_DIRECT_CONTROL:
		directControl(packet);
		break;
	case OPCODE_PID_CONTROL:
		pidControl(packet);
		break;
	case OPCODE_NOP:
		break;
	}
}

void writeToJetson(uint8_t *data, uint8_t payload_size)
{
	uint8_t length = payload_size + 3;
	uint8_t *encoded = malloc(length);
	encoded[0] = 0xff;
	encoded[1] = payload_size | 0xc0;
	memcpy(encoded+2, data, payload_size);
	encoded[length - 1] = calculateChecksum(data, payload_size);

	HAL_StatusTypeDef HALStatus = HAL_UART_Transmit(&huart2, encoded, length, HAL_MAX_DELAY);

	if (HALStatus != HAL_OK)
		Error_Handler();

	free(encoded);
}

void sendGlobalEnableFrame()
{
	  uint32_t mb;
	  CAN_TxHeaderTypeDef hdr;

	  hdr.ExtId = 0x401bf;
	  hdr.IDE = CAN_ID_EXT;
	  hdr.RTR = CAN_RTR_DATA;
	  hdr.DLC = 2;
	  hdr.TransmitGlobalTime = DISABLE;

	  if (HAL_CAN_AddTxMessage(&hcan1, &hdr, (unsigned char *) "\x01\x00", &mb) != HAL_OK)
		Error_Handler();
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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  MX_CAN1_Init();

  writeDebugString("???\r\n");
  TalonSRX talonSRX = TalonSRXInit(&hcan1, 1);
  TalonFX talonFX = TalonFXInit(&hcan1, 39);
  sendGlobalEnableFrame();
//  talonSRX.setInverted(&talonSRX, true);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  writeDebugString("ily\r\n");
  while (1)
//  for (int j = 0; j < 100; j++)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  sendGlobalEnableFrame();

	  talonSRX.set(&talonSRX, -1);
	  talonFX.set(&talonFX, 0.2);

	  HAL_Delay(10);
//	if (DEBUG)
//		writeDebugString("hi\r\n");
//	SerialPacket packet = readFromJetson();
//	if (!packet.invalid)
//	{
//		writeDebugString("starting action\r\n");
//		readAction(packet);
//	}
//	else
//	{
//		writeDebugString("invalid packet read\r\n");
//	}
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

  if (HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, can_irq))
	Error_Handler();

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
	Error_Handler();

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	Error_Handler();
  /* USER CODE END CAN1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void can_irq(CAN_HandleTypeDef *pcan) {
  CAN_RxHeaderTypeDef msg;
  uint8_t data[8];
  HAL_CAN_GetRxMessage(pcan, CAN_RX_FIFO0, &msg, data);
  writeDebugString(data);
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
