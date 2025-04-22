/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <math.h>
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

#define OPCODE_STOP 0
#define OPCODE_DIRECT_CONTROL 1
#define OPCODE_PID_CONTROL 2
#define OPCODE_NOP 3

#define INA219_SAMPLE_COUNT 500
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct serialPacket {
   /* uint8_t opcode;
    uint8_t payload_size;
    uint8_t *payload;
    uint8_t invalid;
    */
  uint8_t invalid;
  uint8_t header;
  uint8_t top_left_wheel;
  uint8_t back_left_wheel;
  uint8_t top_right_wheel;
  uint8_t back_right_wheel;
  uint8_t drum;
  uint8_t actuator;
} SerialPacket;


typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint8_t *tx_buf;
  uint8_t *rx_buf;
} I2C_Context;


// Probably unnecessary with new protocol
/*
uint8_t calculateChecksum(uint8_t *buffer, uint8_t length)
{
    uint8_t checksum = 0;
    for (int i = 0; i < length; i++)
        checksum += buffer[i];
    return checksum;
}
*/

void writeDebug(const char *buffer, uint8_t length)
{
    HAL_UART_Transmit(&huart3, (uint8_t *) buffer, length, HAL_MAX_DELAY);
}

void writeDebugString(const char *buffer)
{
    writeDebug(buffer, strlen(buffer));
}

// Do we need this?
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

// TODO: update protocol
SerialPacket readFromJetson()
{
  // HEADER BYTE
  uint8_t header;
  HAL_StatusTypeDef hal_status = HAL_UART_Receive(&huart2, &header, 1, HAL_MAX_DELAY);
  if (hal_status != HAL_OK)
  {
    // Error_Handler();
    writeDebugString("invalid packet");
    return (SerialPacket) {
        .invalid = 1
    };
  }
  if (DEBUG)
  {
    writeDebugString("header: ");
    char print[5];
    sprintf(print, "%d", header);
    writeDebugString(print);
    writeDebugString("\r\n");
  }

  // TOP LEFT WHEEL BYTE
  uint8_t top_left_wheel;
  hal_status = HAL_UART_Receive(&huart2, &top_left_wheel, 1, HAL_MAX_DELAY);
  if (hal_status != HAL_OK)
  {
    // Error_Handler();
    writeDebugString("invalid packet");
    return (SerialPacket) {
        .invalid = 1
    };
  }
  if (DEBUG)
  {
    writeDebugString("top left wheel: ");
    char print[5];
    sprintf(print, "%d", top_left_wheel);
    writeDebugString(print);
    writeDebugString("\r\n");
  }

  // BACK LEFT WHEEL BYTE
  uint8_t back_left_wheel;
  hal_status = HAL_UART_Receive(&huart2, &back_left_wheel, 1, HAL_MAX_DELAY);
  if (hal_status != HAL_OK)
  {
    // Error_Handler();
    writeDebugString("invalid packet");
    return (SerialPacket) {
        .invalid = 1
    };
  }
  if (DEBUG)
  {
    writeDebugString("back left wheel: ");
    char print[5];
    sprintf(print, "%d", back_left_wheel);
    writeDebugString(print);
    writeDebugString("\r\n");
  }

  // TOP RIGHT WHEEL BYTE
  uint8_t top_right_wheel;
  hal_status = HAL_UART_Receive(&huart2, &top_right_wheel, 1, HAL_MAX_DELAY);
  if (hal_status != HAL_OK)
  {
    // Error_Handler();
    writeDebugString("invalid packet");
    return (SerialPacket) {
        .invalid = 1
    };
  }
  if (DEBUG)
  {
    writeDebugString("top right wheel: ");
    char print[5];
    sprintf(print, "%d", top_right_wheel);
    writeDebugString(print);
    writeDebugString("\r\n");
  }

  // BACK RIGHT WHEEL BYTE
  uint8_t back_right_wheel;
  hal_status = HAL_UART_Receive(&huart2, &back_right_wheel, 1, HAL_MAX_DELAY);
  if (hal_status != HAL_OK)
  {
    // Error_Handler();
    writeDebugString("invalid packet");
    return (SerialPacket) {
        .invalid = 1
    };
  }
  if (DEBUG)
  {
    writeDebugString("back right wheel: ");
    char print[5];
    sprintf(print, "%d", back_right_wheel);
    writeDebugString(print);
    writeDebugString("\r\n");
  }

  // DRUM BYTE
  uint8_t drum;
  hal_status = HAL_UART_Receive(&huart2, &drum, 1, HAL_MAX_DELAY);
  if (hal_status != HAL_OK)
  {
    // Error_Handler();
    writeDebugString("invalid packet");
    return (SerialPacket) {
        .invalid = 1
    };
  }
  if (DEBUG)
  {
    writeDebugString("drum: ");
    char print[5];
    sprintf(print, "%d", drum);
    writeDebugString(print);
    writeDebugString("\r\n");
  }

  // ACTUATOR BYTE
  uint8_t actuator;
  hal_status = HAL_UART_Receive(&huart2, &actuator, 1, HAL_MAX_DELAY);
  if (hal_status != HAL_OK)
  {
    // Error_Handler();
    writeDebugString("invalid packet");
    return (SerialPacket) {
        .invalid = 1
    };
  }
  if (DEBUG)
  {
    writeDebugString("actuator: ");
    char print[5];
    sprintf(print, "%d", actuator);
    writeDebugString(print);
    writeDebugString("\r\n");
  }

  return (SerialPacket) {
    .invalid = 0,
    .header = header,
    .top_left_wheel = top_left_wheel,
    .back_left_wheel = back_left_wheel,
    .top_right_wheel = top_right_wheel,
    .back_right_wheel = back_right_wheel,
    .drum = drum,
    .actuator = actuator,
  };
}
/*
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

// TODO: Update to match new protocol
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
//          writeDebugFormat("Front left: %.6f", command);
            break;
        case 1:
            directDriveRight(command * DRIVETRAIN_SCALE, CONTROL_UPPER_BOUND);

            sprintf(debug_buffer, "Front right: %.6f\r\n", command);
            writeDebug(debug_buffer, strlen(debug_buffer));
//          writeDebugFormat("Front right: %.6f", command);
            break;
        case 2:
            directDriveLeft(command * DRIVETRAIN_SCALE, CONTROL_UPPER_BOUND);

            sprintf(debug_buffer, "Back left: %.6f\r\n", command);
            writeDebug(debug_buffer, strlen(debug_buffer));
//          writeDebugFormat("Back left: %.6f", command);
            break;
        case 3:
            directDriveRight(command * DRIVETRAIN_SCALE, CONTROL_UPPER_BOUND);

            sprintf(debug_buffer, "Back left: %.6f\r\n", command);
            writeDebug(debug_buffer, strlen(debug_buffer));
//          writeDebugFormat("Back right: %.6f", command);
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

// TODO: Update to match new protocol
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

// Did protocol for writing to Jetson change?
void writeToJetson(uint8_t *data, uint8_t payload_size)
{

    uint8_t length = payload_size + 3;
    uint8_t *encoded = malloc(length);
    encoded[0] = 0xff;
    encoded[1] = payload_size | 0xc0;
    memcpy(encoded+2, data, payload_size);
    encoded[length - 1] = calculateChecksum(data, payload_size);

    HALStatus = HAL_UART_Transmit(&huart2, encoded, length, HAL_MAX_DELAY);

    if (HALStatus != HAL_OK)
        Error_Handler();

    free(encoded);

}
*/

I2C_Context i2c_ctx;
/**
  * @brief  Simplified I2C data transmission to the INA219 Current Sensor
  * 	in blocking (polling) mode. The 8-bit device address (0x80) is taken
  * 	by shifting the 7-bit default slave address (0x40) to the left.
  * @param  registerAddress Address of the register to be written to
  * @param  registerValue Data to be written into the register
  * @retval none
  */
void writeRegister(uint8_t registerAddress, uint16_t registerValue)
{
	uint8_t data[3];

	data[0] = registerAddress;		// Register address
	data[1] = registerValue >> 8; 	// MSB of 16 bit data
	data[2] = registerValue;		// LSB of 16 bit data

	HAL_StatusTypeDef hal_status = HAL_I2C_Master_Transmit(&hi2c1, 0x0080, data, 3, 100);
	if (hal_status != HAL_OK)
	{
		writeDebugString("I2C write error\r\n");
	}
}

/**
  * @brief  Simplified I2C data reading of the INA219 Current Sensor
  * 	in blocking (polling) mode. The 8-bit device address (0x80) is taken
  * 	by shifting the 7-bit default slave address (0x40) to the left.
  * @param  registerAddress Address of the register to be read from
  * @param  receiveBuffer Location to store the read data
  * @retval none
  */
void readRegister(uint8_t registerAddress)
{
	HAL_StatusTypeDef hal_status;

	// First send the address that we want to read from to the pointer register
  // using interrupt mode to prevent CPU from blocking
  // 
	hal_status = HAL_I2C_Master_Transmit_IT(&hi2c1, 0x0080, &registerAddress, 1); // could be optimized for lower power consumption
	if (hal_status != HAL_OK)
	{
		//writeDebugString("I2C write error (register address to read from)\r\n");
	}


}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {

  HAL_StatusTypeDef hal_status;
  //writeDebugString("callback called\r\n");
  // Then read the 2 bytes from the register and store in receiveBuffer
	hal_status = HAL_I2C_Master_Receive_IT(&hi2c1, 0x0080, i2c_ctx.rx_buf, 2);
	if (hal_status != HAL_OK)
	{
		writeDebugString("I2C read error\r\n");
	}
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /*** I2C Current Sensing ***/
  // 7-bit slave address = 0x40 (default when pins A0,A1 = GND)
  // 8-bit device address = 0x80 (used in the HAL_I2C_Transmit/Receive function)
  uint8_t buffer[2]; // for I2C reading, data storage
  int16_t rawValue;
  float currentValue;
  float rmsCurrent;

  //pass buffer to i2c callback context for use in the tx and rx callbacks 
  i2c_ctx.hi2c = &hi2c1;
  //i2c_ctx.tx_buf = tx_buf;
  i2c_ctx.rx_buf = buffer;


  writeRegister(0x00, 0x399F); // CONFIGURATION

  float LSB = 0.001; // LSB scaling factor: milliAmperes
  float shuntResistor = 0.1; // 0.1 ohm 1% sense resistor
  float calibrationValue = 0.04096 / (LSB * shuntResistor); // truncated, refer to data sheet equation
  writeDebugFormat("INA219 Calibration Register Value: 0x%x, 0d%d\r\n\n", calibrationValue);

  writeRegister(0x05, 0x0199); // CALIBRATION (calibration register value: 0d409.6 --> 0d409 --> 0x0199)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  for (int sample = 0; sample < INA219_SAMPLE_COUNT; sample++) {
	  		  readRegister(0x04); // MEASUREMENT (of the current register)

	  		  // CONVERSION of the current register value to Amperes
	  		  rawValue = (buffer[0] << 8) | buffer[1]; // Combine MSB and LSB to form raw current value
	  		  currentValue = rawValue * LSB; // Undo "LSB" scaling factor to get Ampere units

	  		  rmsCurrent += pow(currentValue, 2); // RMS step 1: sum up the squares
	  	  }

	  	  rmsCurrent /= INA219_SAMPLE_COUNT; // RMS step 2: divide by the # samples
	  	  rmsCurrent = sqrt(rmsCurrent); // RMS step 3: take the square root


	  	  //writeDebugFormat("%x raw \r\n", rawValue);
	  	  writeDebugFormat("%.6f Amps\r\n", rmsCurrent);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
