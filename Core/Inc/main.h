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
#include "stm32f4xx_hal.h"

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
#define EN_2_Pin GPIO_PIN_1
#define EN_2_GPIO_Port GPIOC
#define EN_2C3_Pin GPIO_PIN_3
#define EN_2C3_GPIO_Port GPIOC
#define PWM_MOTOR_2_CH1_Pin GPIO_PIN_0
#define PWM_MOTOR_2_CH1_GPIO_Port GPIOA
#define PWM_MOTOR_2_CH_2_Pin GPIO_PIN_1
#define PWM_MOTOR_2_CH_2_GPIO_Port GPIOA
#define ENCODER_2_CH_1_Pin GPIO_PIN_6
#define ENCODER_2_CH_1_GPIO_Port GPIOA
#define ENCODER_2_CH_2_Pin GPIO_PIN_7
#define ENCODER_2_CH_2_GPIO_Port GPIOA
#define EN_1_Pin GPIO_PIN_8
#define EN_1_GPIO_Port GPIOE
#define EN_1E10_Pin GPIO_PIN_10
#define EN_1E10_GPIO_Port GPIOE
#define PWM_MOTOR_1_CH_3_Pin GPIO_PIN_13
#define PWM_MOTOR_1_CH_3_GPIO_Port GPIOE
#define PWM_MOTOR_1_CH_4_Pin GPIO_PIN_14
#define PWM_MOTOR_1_CH_4_GPIO_Port GPIOE
#define ENCODER_1_CH_1_Pin GPIO_PIN_12
#define ENCODER_1_CH_1_GPIO_Port GPIOD
#define ENCODER_1_CH_2_Pin GPIO_PIN_13
#define ENCODER_1_CH_2_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
