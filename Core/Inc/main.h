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
#define ST_PWM_in_Pin GPIO_PIN_5
#define ST_PWM_in_GPIO_Port GPIOE
#define MotorEnc_FR_A_Pin GPIO_PIN_0
#define MotorEnc_FR_A_GPIO_Port GPIOA
#define MotorEnc_FR_B_Pin GPIO_PIN_1
#define MotorEnc_FR_B_GPIO_Port GPIOA
#define MotorCurrentSens_FL_Pin GPIO_PIN_2
#define MotorCurrentSens_FL_GPIO_Port GPIOA
#define MotorCurrentSens_FR_Pin GPIO_PIN_3
#define MotorCurrentSens_FR_GPIO_Port GPIOA
#define MotorCurrentSens_ST_Pin GPIO_PIN_4
#define MotorCurrentSens_ST_GPIO_Port GPIOA
#define MotorCurrentSens_RL_Pin GPIO_PIN_5
#define MotorCurrentSens_RL_GPIO_Port GPIOA
#define MotorCurrentSens_RR_Pin GPIO_PIN_6
#define MotorCurrentSens_RR_GPIO_Port GPIOA
#define BattVoltageSens_Pin GPIO_PIN_7
#define BattVoltageSens_GPIO_Port GPIOA
#define ST_Volume_Pin GPIO_PIN_4
#define ST_Volume_GPIO_Port GPIOC
#define LED6_Pin GPIO_PIN_1
#define LED6_GPIO_Port GPIOB
#define LED7_Pin GPIO_PIN_2
#define LED7_GPIO_Port GPIOB
#define LED8_Pin GPIO_PIN_7
#define LED8_GPIO_Port GPIOE
#define LED9_Pin GPIO_PIN_8
#define LED9_GPIO_Port GPIOE
#define LED10_Pin GPIO_PIN_9
#define LED10_GPIO_Port GPIOE
#define YOBI4_Pin GPIO_PIN_10
#define YOBI4_GPIO_Port GPIOE
#define YOBI3_Pin GPIO_PIN_11
#define YOBI3_GPIO_Port GPIOE
#define YOBI2_Pin GPIO_PIN_12
#define YOBI2_GPIO_Port GPIOE
#define YOBI1_Pin GPIO_PIN_13
#define YOBI1_GPIO_Port GPIOE
#define ST_Direction_in_Pin GPIO_PIN_14
#define ST_Direction_in_GPIO_Port GPIOE
#define Motor_RR_PWM_Pin GPIO_PIN_14
#define Motor_RR_PWM_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_15
#define LED5_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_8
#define LED4_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_9
#define LED3_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_11
#define LED1_GPIO_Port GPIOD
#define MotorEnc_RR_A_Pin GPIO_PIN_12
#define MotorEnc_RR_A_GPIO_Port GPIOD
#define MotorEnc_RR_B_Pin GPIO_PIN_13
#define MotorEnc_RR_B_GPIO_Port GPIOD
#define Motor_RR_SR_Pin GPIO_PIN_14
#define Motor_RR_SR_GPIO_Port GPIOD
#define Motor_RR_PHASE_Pin GPIO_PIN_15
#define Motor_RR_PHASE_GPIO_Port GPIOD
#define MotorEnc_RL_A_Pin GPIO_PIN_6
#define MotorEnc_RL_A_GPIO_Port GPIOC
#define MotorEnc_RL_B_Pin GPIO_PIN_7
#define MotorEnc_RL_B_GPIO_Port GPIOC
#define Motor_RL_SR_Pin GPIO_PIN_8
#define Motor_RL_SR_GPIO_Port GPIOC
#define Motor_RL_PHASE_Pin GPIO_PIN_9
#define Motor_RL_PHASE_GPIO_Port GPIOC
#define Motor_RL_PWM_Pin GPIO_PIN_8
#define Motor_RL_PWM_GPIO_Port GPIOA
#define Motor_FR_PWM_Pin GPIO_PIN_9
#define Motor_FR_PWM_GPIO_Port GPIOA
#define Motor_ST_PWM_Pin GPIO_PIN_10
#define Motor_ST_PWM_GPIO_Port GPIOA
#define Motor_FL_PWM_Pin GPIO_PIN_11
#define Motor_FL_PWM_GPIO_Port GPIOA
#define Motor_FR_SR_Pin GPIO_PIN_1
#define Motor_FR_SR_GPIO_Port GPIOD
#define Motor_FR_PHASE_Pin GPIO_PIN_2
#define Motor_FR_PHASE_GPIO_Port GPIOD
#define Motor_ST_SR_Pin GPIO_PIN_3
#define Motor_ST_SR_GPIO_Port GPIOD
#define Motor_ST_PHASE_Pin GPIO_PIN_4
#define Motor_ST_PHASE_GPIO_Port GPIOD
#define Motor_FL_SR_Pin GPIO_PIN_5
#define Motor_FL_SR_GPIO_Port GPIOD
#define Motor_FL_PHASE_Pin GPIO_PIN_6
#define Motor_FL_PHASE_GPIO_Port GPIOD
#define MotorEnc_FL_A_Pin GPIO_PIN_4
#define MotorEnc_FL_A_GPIO_Port GPIOB
#define MotorEnc_FL_B_Pin GPIO_PIN_5
#define MotorEnc_FL_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
