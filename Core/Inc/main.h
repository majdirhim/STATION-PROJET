/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

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
#define RAIN_Pin GPIO_PIN_15
#define RAIN_GPIO_Port GPIOA
#define RAIN_EXTI_IRQn EXTI15_10_IRQn
#define LD_Pin GPIO_PIN_1
#define LD_GPIO_Port GPIOI
#define BTN_USER_Pin GPIO_PIN_11
#define BTN_USER_GPIO_Port GPIOI
#define BTN_USER_EXTI_IRQn EXTI15_10_IRQn
#define TOUCH_Pin GPIO_PIN_13
#define TOUCH_GPIO_Port GPIOI
#define TOUCH_EXTI_IRQn EXTI15_10_IRQn
#define LD_B_Pin GPIO_PIN_9
#define LD_B_GPIO_Port GPIOH
#define LD_R_Pin GPIO_PIN_11
#define LD_R_GPIO_Port GPIOH
#define LD_G_Pin GPIO_PIN_10
#define LD_G_GPIO_Port GPIOH
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
