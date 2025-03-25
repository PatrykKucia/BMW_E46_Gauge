/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PH0-OSC_IN(PH0)   ------> RCC_OSC_IN
     PH1-OSC_OUT(PH1)   ------> RCC_OSC_OUT
     PA13(JTMS/SWDIO)   ------> DEBUG_JTMS-SWDIO
     PA14(JTCK/SWCLK)   ------> DEBUG_JTCK-SWCLK
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(K_BUS_SLP_GPIO_Port, K_BUS_SLP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BACKLIGHT_Pin|BATT_CHARGE_LIGHT_Pin|OIL_LIGHT_Pin|BRAKE_FLU_LIGHT_Pin
                          |TRCVR_MODE_Pin|D3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ABS_Pin|PARKING_BRAKE_Pin|BRAKE_WEAR_SENS_Pin|COOLANT_LVL_SENS_Pin
                          |WASHER_FLU_LVL_Pin|D2_Pin|D1_Pin|Fuel_HVC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : STM_ESP_Pin */
  GPIO_InitStruct.Pin = STM_ESP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STM_ESP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : K_BUS_SLP_Pin */
  GPIO_InitStruct.Pin = K_BUS_SLP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(K_BUS_SLP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BACKLIGHT_Pin BATT_CHARGE_LIGHT_Pin OIL_LIGHT_Pin BRAKE_FLU_LIGHT_Pin
                           TRCVR_MODE_Pin D3_Pin */
  GPIO_InitStruct.Pin = BACKLIGHT_Pin|BATT_CHARGE_LIGHT_Pin|OIL_LIGHT_Pin|BRAKE_FLU_LIGHT_Pin
                          |TRCVR_MODE_Pin|D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ABS_Pin PARKING_BRAKE_Pin BRAKE_WEAR_SENS_Pin COOLANT_LVL_SENS_Pin
                           WASHER_FLU_LVL_Pin D2_Pin D1_Pin Fuel_HVC_Pin */
  GPIO_InitStruct.Pin = ABS_Pin|PARKING_BRAKE_Pin|BRAKE_WEAR_SENS_Pin|COOLANT_LVL_SENS_Pin
                          |WASHER_FLU_LVL_Pin|D2_Pin|D1_Pin|Fuel_HVC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
