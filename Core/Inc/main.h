/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
extern int IN_W ,IN_H;
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_FLASH_Pin GPIO_PIN_0
#define CS_FLASH_GPIO_Port GPIOB
#define LED_Capture_Pin GPIO_PIN_8
#define LED_Capture_GPIO_Port GPIOA
#define LED_Update_Pin GPIO_PIN_10
#define LED_Update_GPIO_Port GPIOA
#define LED_Mode_Pin GPIO_PIN_12
#define LED_Mode_GPIO_Port GPIOA
#define rst_tof_Pin GPIO_PIN_7
#define rst_tof_GPIO_Port GPIOB
#define lp_tof_Pin GPIO_PIN_8
#define lp_tof_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void bilinear_init_q15(void);
void load_input_int16_to_q15(const int16_t raw[IN_W * IN_H]);
void bilinear_8x8_to_16x16_q15(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
