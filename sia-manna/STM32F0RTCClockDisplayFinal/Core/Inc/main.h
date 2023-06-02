/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Audio_IN_Pin GPIO_PIN_0
#define Audio_IN_GPIO_Port GPIOC
#define Potentiometer_Pin GPIO_PIN_1
#define Potentiometer_GPIO_Port GPIOC
#define MC_HeatsinkTemperature_Pin GPIO_PIN_2
#define MC_HeatsinkTemperature_GPIO_Port GPIOC
#define MC_BEMF_C_Pin GPIO_PIN_3
#define MC_BEMF_C_GPIO_Port GPIOC
#define JOY_SEL_Pin GPIO_PIN_0
#define JOY_SEL_GPIO_Port GPIOA
#define LDR_OUT_Pin GPIO_PIN_1
#define LDR_OUT_GPIO_Port GPIOA
#define MC_CurrentA_Pin GPIO_PIN_2
#define MC_CurrentA_GPIO_Port GPIOA
#define MC_CurrentB2_Pin GPIO_PIN_3
#define MC_CurrentB2_GPIO_Port GPIOA
#define LCD_CS_OD_Pin GPIO_PIN_4
#define LCD_CS_OD_GPIO_Port GPIOF
#define MicroSD_CS_OD_Pin GPIO_PIN_5
#define MicroSD_CS_OD_GPIO_Port GPIOF
#define Audio_OUT_Pin GPIO_PIN_4
#define Audio_OUT_GPIO_Port GPIOA
#define MC_NTC_Pin GPIO_PIN_6
#define MC_NTC_GPIO_Port GPIOA
#define MC_CurrentC_Pin GPIO_PIN_4
#define MC_CurrentC_GPIO_Port GPIOC
#define MC_PFCpwm_Pin GPIO_PIN_1
#define MC_PFCpwm_GPIO_Port GPIOB
#define SPI1_MOSI_DIR_Pin GPIO_PIN_2
#define SPI1_MOSI_DIR_GPIO_Port GPIOB
#define HDMI_CEC_OD_Pin GPIO_PIN_10
#define HDMI_CEC_OD_GPIO_Port GPIOB
#define SDcard_detect_Pin GPIO_PIN_15
#define SDcard_detect_GPIO_Port GPIOB
#define JOY_UP_Pin GPIO_PIN_6
#define JOY_UP_GPIO_Port GPIOC
#define JOY_DOWN_Pin GPIO_PIN_7
#define JOY_DOWN_GPIO_Port GPIOC
#define JOY_RIGHT_Pin GPIO_PIN_8
#define JOY_RIGHT_GPIO_Port GPIOC
#define JOY_LEFT_Pin GPIO_PIN_9
#define JOY_LEFT_GPIO_Port GPIOC
#define HDMI_HPD_Source_5V_Pin GPIO_PIN_8
#define HDMI_HPD_Source_5V_GPIO_Port GPIOA
#define RS485_DIR_Pin GPIO_PIN_12
#define RS485_DIR_GPIO_Port GPIOA
#define MC_EnA_Pin GPIO_PIN_15
#define MC_EnA_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_12
#define LED3_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_2
#define LED4_GPIO_Port GPIOD
#define IR_IN_Pin GPIO_PIN_3
#define IR_IN_GPIO_Port GPIOB
#define User_Button_Pin GPIO_PIN_8
#define User_Button_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
