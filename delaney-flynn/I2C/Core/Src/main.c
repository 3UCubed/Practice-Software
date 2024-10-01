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
#include "string.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static const uint8_t ADT_ADDR1 = 0x48 << 1;
static const uint8_t ADT_ADDR2 = 0x49 << 1;
static const uint8_t ADT_ADDR3 = 0x4A << 1;
static const uint8_t ADT_ADDR4 = 0x4B << 1;
static const uint8_t REG_TEMP = 0x00;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	HAL_StatusTypeDef ret;
	uint8_t buf1[48];
	uint8_t buf2[48];
	uint8_t buf3[48];
	uint8_t buf4[48];
	int16_t val;
	float temp_c;
  /* USER CODE END 1 */

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* SENSOR 1 CODE */
	  buf1[0] = REG_TEMP;
	  ret = HAL_I2C_Master_Transmit(&hi2c1, ADT_ADDR1, buf1, 1, HAL_MAX_DELAY);
	  if (ret != HAL_OK) {
		  strcpy((char*)buf1, "Error Tx\r\n");
	  } else {
		  ret = HAL_I2C_Master_Receive(&hi2c1, ADT_ADDR1, buf1, 2, HAL_MAX_DELAY);
		  if (ret != HAL_OK) {
		  		  strcpy((char*)buf1, "Error Rx\r\n");
		  } else {
			  val = ((int16_t)buf1[0] << 4 | buf1[1] >> 4);

			  if (val > 0x7FF) {
				  val |= 0xF000;
			  }

			  temp_c = val * 0.0625;

			  temp_c *= 100;
			  sprintf((char*)buf1, "ADT7410 Readings\r\n | 1: %u.%02uC |\r\n",
					  ((unsigned int)temp_c / 100),
					  ((unsigned int)temp_c %100));
		  }
	  }

	  /* SENSOR 2 CODE */
	  buf2[0] = REG_TEMP;
	  ret = HAL_I2C_Master_Transmit(&hi2c1, ADT_ADDR2, buf2, 1, HAL_MAX_DELAY);
	  if (ret != HAL_OK) {
		  strcpy((char*)buf2, "Error Tx\r\n");
	  } else {
		  ret = HAL_I2C_Master_Receive(&hi2c1, ADT_ADDR2, buf2, 2, HAL_MAX_DELAY);
		  if (ret != HAL_OK) {
				  strcpy((char*)buf2, "Error Rx\r\n");
		  } else {
			  val = ((int16_t)buf2[0] << 4 | buf2[1] >> 4);

			  if (val > 0x7FF) {
				  val |= 0xF000;
			  }

			  temp_c = val * 0.0625;

			  temp_c *= 100;
			  sprintf((char*)buf2, " | 2: %u.%02uC |\r\n",
					  ((unsigned int)temp_c / 100),
					  ((unsigned int)temp_c %100));
		  }
	  }

	  /* SENSOR 3 CODE */
	  buf3[0] = REG_TEMP;
	  ret = HAL_I2C_Master_Transmit(&hi2c1, ADT_ADDR3, buf3, 1, HAL_MAX_DELAY);
	  if (ret != HAL_OK) {
		  strcpy((char*)buf3, "Error Tx\r\n");
	  } else {
		  ret = HAL_I2C_Master_Receive(&hi2c1, ADT_ADDR3, buf3, 2, HAL_MAX_DELAY);
		  if (ret != HAL_OK) {
				  strcpy((char*)buf3, "Error Rx\r\n");
		  } else {
			  val = ((int16_t)buf3[0] << 4 | buf3[1] >> 4);

			  if (val > 0x7FF) {
				  val |= 0xF000;
			  }

			  temp_c = val * 0.0625;

			  temp_c *= 100;
			  sprintf((char*)buf3, " | 3: %u.%02uC |\r\n",
					  ((unsigned int)temp_c / 100),
					  ((unsigned int)temp_c %100));
		  }
	  }

	  /* SENSOR 4 CODE */
	  buf4[0] = REG_TEMP;
	  ret = HAL_I2C_Master_Transmit(&hi2c1, ADT_ADDR4, buf4, 1, HAL_MAX_DELAY);
	  if (ret != HAL_OK) {
		  strcpy((char*)buf4, "Error Tx\r\n");
	  } else {
		  ret = HAL_I2C_Master_Receive(&hi2c1, ADT_ADDR4, buf4, 2, HAL_MAX_DELAY);
		  if (ret != HAL_OK) {
				  strcpy((char*)buf4, "Error Rx\r\n");
		  } else {
			  val = ((int16_t)buf4[0] << 4 | buf4[1] >> 4);

			  if (val > 0x7FF) {
				  val |= 0xF000;
			  }

			  temp_c = val * 0.0625;

			  temp_c *= 100;
			  sprintf((char*)buf4, " | 4: %u.%02uC |\r\n\r\n",
					  ((unsigned int)temp_c / 100),
					  ((unsigned int)temp_c %100));
		  }
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_UART_Transmit(&huart1, buf1, strlen((char*)buf1), HAL_MAX_DELAY);
	  HAL_UART_Transmit(&huart1, buf2, strlen((char*)buf2), HAL_MAX_DELAY);
	  HAL_UART_Transmit(&huart1, buf3, strlen((char*)buf3), HAL_MAX_DELAY);
	  HAL_UART_Transmit(&huart1, buf4, strlen((char*)buf4), HAL_MAX_DELAY);
	  HAL_Delay(500);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x2000090E;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
