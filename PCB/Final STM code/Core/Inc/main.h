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
#include "stm32h5xx_hal.h"

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
#define STM_ESP_Pin GPIO_PIN_13
#define STM_ESP_GPIO_Port GPIOC
#define K_BUS_SLP_Pin GPIO_PIN_15
#define K_BUS_SLP_GPIO_Port GPIOC
#define BACKLIGHT_Pin GPIO_PIN_4
#define BACKLIGHT_GPIO_Port GPIOA
#define BATT_CHARGE_LIGHT_Pin GPIO_PIN_5
#define BATT_CHARGE_LIGHT_GPIO_Port GPIOA
#define OIL_LIGHT_Pin GPIO_PIN_6
#define OIL_LIGHT_GPIO_Port GPIOA
#define BRAKE_FLU_LIGHT_Pin GPIO_PIN_7
#define BRAKE_FLU_LIGHT_GPIO_Port GPIOA
#define ABS_Pin GPIO_PIN_1
#define ABS_GPIO_Port GPIOB
#define PARKING_BRAKE_Pin GPIO_PIN_2
#define PARKING_BRAKE_GPIO_Port GPIOB
#define BRAKE_WEAR_SENS_Pin GPIO_PIN_10
#define BRAKE_WEAR_SENS_GPIO_Port GPIOB
#define COOLANT_LVL_SENS_Pin GPIO_PIN_12
#define COOLANT_LVL_SENS_GPIO_Port GPIOB
#define WASHER_FLU_LVL_Pin GPIO_PIN_13
#define WASHER_FLU_LVL_GPIO_Port GPIOB
#define TRCVR_MODE_Pin GPIO_PIN_10
#define TRCVR_MODE_GPIO_Port GPIOA
#define D3_Pin GPIO_PIN_15
#define D3_GPIO_Port GPIOA
#define D2_Pin GPIO_PIN_3
#define D2_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_4
#define D1_GPIO_Port GPIOB
#define Fuel_HVC_Pin GPIO_PIN_5
#define Fuel_HVC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
