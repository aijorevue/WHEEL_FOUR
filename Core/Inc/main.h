/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define Avoid_L_Pin GPIO_PIN_13
#define Avoid_L_GPIO_Port GPIOC
#define Avoid_R_Pin GPIO_PIN_14
#define Avoid_R_GPIO_Port GPIOC
#define OLED_SCL_Pin GPIO_PIN_2
#define OLED_SCL_GPIO_Port GPIOA
#define OLED_SDA_Pin GPIO_PIN_3
#define OLED_SDA_GPIO_Port GPIOA
#define IN6_Pin GPIO_PIN_6
#define IN6_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_12
#define BIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_13
#define BIN1_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_14
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_15
#define AIN2_GPIO_Port GPIOB
#define Track_1_Pin GPIO_PIN_8
#define Track_1_GPIO_Port GPIOA
#define Track_2_Pin GPIO_PIN_9
#define Track_2_GPIO_Port GPIOA
#define Track_3_Pin GPIO_PIN_10
#define Track_3_GPIO_Port GPIOA
#define Track_4_Pin GPIO_PIN_11
#define Track_4_GPIO_Port GPIOA
#define Track_5_Pin GPIO_PIN_12
#define Track_5_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
