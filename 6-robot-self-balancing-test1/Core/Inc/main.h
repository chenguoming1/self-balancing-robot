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

/* USER CODE BEGIN Private defines */
/* ── Peripheral Handles (defined in main.c) ─────────────────────────── */
extern I2C_HandleTypeDef  hi2c1;   // MPU6050         – PB6/PB7
extern UART_HandleTypeDef huart2;  // JDY-23 BT       – PA2/PA3  (USART2!)
extern TIM_HandleTypeDef  htim1;   // Motor PWM        – PA8(CH1) PA11(CH4)
extern TIM_HandleTypeDef  htim2;   // Left  encoder    – PA0(CH1) PA1(CH2)
extern TIM_HandleTypeDef  htim3;   // Right encoder    – PA6(CH1) PA7(CH2)
extern TIM_HandleTypeDef  htim4;   // Control loop 200 Hz ticker

/* ── MPU6050 Interrupt Pin ───────────────────────────────────────────── */
#define MPU_INT_PIN      GPIO_PIN_5
#define MPU_INT_PORT     GPIOB
#define MPU_INT_IRQn     EXTI9_5_IRQn   // PB5 → EXTI line 5 → EXTI9_5

/* ── TB6612FNG – Motor A  (LEFT wheel) ──────────────────────────────── */
/*    AIN1=PB13  AIN2=PB12  PWMA=PA11 (TIM1_CH4)                        */
#define MOTOR_A_IN1_PIN   GPIO_PIN_13
#define MOTOR_A_IN1_PORT  GPIOB
#define MOTOR_A_IN2_PIN   GPIO_PIN_12
#define MOTOR_A_IN2_PORT  GPIOB
#define MOTOR_A_PWM_CH    TIM_CHANNEL_4   // TIM1 CH4 → PA11

/* ── TB6612FNG – Motor B  (RIGHT wheel) ─────────────────────────────── */
/*    BIN1=PB14  BIN2=PB15  PWMB=PA8  (TIM1_CH1)                        */
#define MOTOR_B_IN1_PIN   GPIO_PIN_14
#define MOTOR_B_IN1_PORT  GPIOB
#define MOTOR_B_IN2_PIN   GPIO_PIN_15
#define MOTOR_B_IN2_PORT  GPIOB
#define MOTOR_B_PWM_CH    TIM_CHANNEL_1   // TIM1 CH1 → PA8

/* ── PWM Resolution ──────────────────────────────────────────────────── */
/* TIM1: 72 MHz / 72 / 1000 = 1 kHz PWM.  Duty 0–999.                   */
#define MOTOR_PWM_MAX     999u

/* ── Control Loop Timing ─────────────────────────────────────────────── */
#define CONTROL_LOOP_HZ   200u
#define CONTROL_LOOP_DT   (1.0f / (float)CONTROL_LOOP_HZ)   // 0.005 s
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
