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
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_main.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Motor_PWM_ST_Pin GPIO_PIN_5
#define Motor_PWM_ST_GPIO_Port GPIOE
#define Enc_fl_A_Pin GPIO_PIN_0
#define Enc_fl_A_GPIO_Port GPIOA
#define Enc_fl_B_Pin GPIO_PIN_1
#define Enc_fl_B_GPIO_Port GPIOA
#define Motor_PWM_FL_Pin GPIO_PIN_9
#define Motor_PWM_FL_GPIO_Port GPIOE
#define Motor_PWM_FR_Pin GPIO_PIN_11
#define Motor_PWM_FR_GPIO_Port GPIOE
#define Motor_PWM_RL_Pin GPIO_PIN_13
#define Motor_PWM_RL_GPIO_Port GPIOE
#define Motor_PWM_RR_Pin GPIO_PIN_14
#define Motor_PWM_RR_GPIO_Port GPIOE
#define Enc_rl_A_Pin GPIO_PIN_12
#define Enc_rl_A_GPIO_Port GPIOD
#define Enc_rl_B_Pin GPIO_PIN_13
#define Enc_rl_B_GPIO_Port GPIOD
#define Enc_rr_A_Pin GPIO_PIN_6
#define Enc_rr_A_GPIO_Port GPIOC
#define Enc_rr_B_Pin GPIO_PIN_7
#define Enc_rr_B_GPIO_Port GPIOC
#define Enc_fr_A_Pin GPIO_PIN_4
#define Enc_fr_A_GPIO_Port GPIOB
#define Enc_fr_B_Pin GPIO_PIN_5
#define Enc_fr_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
