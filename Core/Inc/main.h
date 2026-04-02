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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BIS2_Pin GPIO_PIN_2
#define BIS2_GPIO_Port GPIOC
#define BIS1_Pin GPIO_PIN_3
#define BIS1_GPIO_Port GPIOC
#define AIS1_Pin GPIO_PIN_0
#define AIS1_GPIO_Port GPIOA
#define AIS2_Pin GPIO_PIN_1
#define AIS2_GPIO_Port GPIOA
#define VBUS_Pin GPIO_PIN_4
#define VBUS_GPIO_Port GPIOA
#define HALL1_B_Pin GPIO_PIN_10
#define HALL1_B_GPIO_Port GPIOB
#define HALL1_B_EXTI_IRQn EXTI15_10_IRQn
#define HALL1_A_Pin GPIO_PIN_11
#define HALL1_A_GPIO_Port GPIOB
#define HALL1_A_EXTI_IRQn EXTI15_10_IRQn
#define MOTOR2_P_Pin GPIO_PIN_6
#define MOTOR2_P_GPIO_Port GPIOC
#define MOTOR2_N_Pin GPIO_PIN_7
#define MOTOR2_N_GPIO_Port GPIOC
#define MOTOR1_P_Pin GPIO_PIN_8
#define MOTOR1_P_GPIO_Port GPIOC
#define MOTOR1_N_Pin GPIO_PIN_9
#define MOTOR1_N_GPIO_Port GPIOC
#define LED_RUN_Pin GPIO_PIN_8
#define LED_RUN_GPIO_Port GPIOA
#define EN_2_Pin GPIO_PIN_9
#define EN_2_GPIO_Port GPIOA
#define EN_1_Pin GPIO_PIN_10
#define EN_1_GPIO_Port GPIOA
#define HALL2_B_Pin GPIO_PIN_6
#define HALL2_B_GPIO_Port GPIOB
#define HALL2_B_EXTI_IRQn EXTI9_5_IRQn
#define HALL2_A_Pin GPIO_PIN_7
#define HALL2_A_GPIO_Port GPIOB
#define HALL2_A_EXTI_IRQn EXTI9_5_IRQn
#define LIMIT2_Pin GPIO_PIN_8
#define LIMIT2_GPIO_Port GPIOB
#define LIMIT2_EXTI_IRQn EXTI9_5_IRQn
#define LIMIT1_Pin GPIO_PIN_9
#define LIMIT1_GPIO_Port GPIOB
#define LIMIT1_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
