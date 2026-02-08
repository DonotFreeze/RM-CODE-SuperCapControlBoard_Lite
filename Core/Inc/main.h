/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "SuperCapCtrl.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
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
#define TEST_IN_Pin GPIO_PIN_10
#define TEST_IN_GPIO_Port GPIOG
#define Ibat_Pin GPIO_PIN_1
#define Ibat_GPIO_Port GPIOA
#define CAP_CTRL_Pin GPIO_PIN_4
#define CAP_CTRL_GPIO_Port GPIOA
#define Vbat_Pin GPIO_PIN_5
#define Vbat_GPIO_Port GPIOA
#define Vcap_Pin GPIO_PIN_6
#define Vcap_GPIO_Port GPIOA
#define Icap_Pin GPIO_PIN_0
#define Icap_GPIO_Port GPIOB
#define T_cap_Pin GPIO_PIN_1
#define T_cap_GPIO_Port GPIOB
#define T_mos_Pin GPIO_PIN_2
#define T_mos_GPIO_Port GPIOB
#define TIM1_Break_CTRL_Pin GPIO_PIN_11
#define TIM1_Break_CTRL_GPIO_Port GPIOB
#define TIM1_BKIN_Pin GPIO_PIN_12
#define TIM1_BKIN_GPIO_Port GPIOB
#define TEST_OUT_Pin GPIO_PIN_13
#define TEST_OUT_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_15
#define LED_BLUE_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_8
#define LED_GREEN_GPIO_Port GPIOA
#define TEST_MODE_Pin GPIO_PIN_9
#define TEST_MODE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LITE

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
