/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdadc.c
  * @brief   This file provides code for the configuration
  *          of the SDADC instances.
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
#include "sdadc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SDADC_HandleTypeDef hsdadc1;
SDADC_HandleTypeDef hsdadc2;

/* SDADC1 init function */
void MX_SDADC1_Init(void)
{

  /* USER CODE BEGIN SDADC1_Init 0 */

  /* USER CODE END SDADC1_Init 0 */

  SDADC_ConfParamTypeDef ConfParamStruct = {0};

  /* USER CODE BEGIN SDADC1_Init 1 */

  /* USER CODE END SDADC1_Init 1 */

  /** Configure the SDADC low power mode, fast conversion mode,
  slow clock mode and SDADC1 reference voltage
  */
  hsdadc1.Instance = SDADC1;
  hsdadc1.Init.IdleLowPowerMode = SDADC_LOWPOWER_NONE;
  hsdadc1.Init.FastConversionMode = SDADC_FAST_CONV_DISABLE;
  hsdadc1.Init.SlowClockMode = SDADC_SLOW_CLOCK_DISABLE;
  hsdadc1.Init.ReferenceVoltage = SDADC_VREF_EXT;
  if (HAL_SDADC_Init(&hsdadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure The Regular Mode
  */
  if (HAL_SDADC_SelectRegularTrigger(&hsdadc1, SDADC_SOFTWARE_TRIGGER) != HAL_OK)
  {
    Error_Handler();
  }

  /** Set parameters for SDADC configuration 0 Register
  */
  ConfParamStruct.InputMode = SDADC_INPUT_MODE_SE_ZERO_REFERENCE;
  ConfParamStruct.Gain = SDADC_GAIN_1;
  ConfParamStruct.CommonMode = SDADC_COMMON_MODE_VSSA;
  ConfParamStruct.Offset = 0;
  if (HAL_SDADC_PrepareChannelConfig(&hsdadc1, SDADC_CONF_INDEX_0, &ConfParamStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDADC1_Init 2 */

  /* USER CODE END SDADC1_Init 2 */

}
/* SDADC2 init function */
void MX_SDADC2_Init(void)
{

  /* USER CODE BEGIN SDADC2_Init 0 */

  /* USER CODE END SDADC2_Init 0 */

  SDADC_ConfParamTypeDef ConfParamStruct = {0};

  /* USER CODE BEGIN SDADC2_Init 1 */

  /* USER CODE END SDADC2_Init 1 */

  /** Configure the SDADC low power mode, fast conversion mode,
  slow clock mode and SDADC1 reference voltage
  */
  hsdadc2.Instance = SDADC2;
  hsdadc2.Init.IdleLowPowerMode = SDADC_LOWPOWER_NONE;
  hsdadc2.Init.FastConversionMode = SDADC_FAST_CONV_DISABLE;
  hsdadc2.Init.SlowClockMode = SDADC_SLOW_CLOCK_DISABLE;
  hsdadc2.Init.ReferenceVoltage = SDADC_VREF_EXT;
  if (HAL_SDADC_Init(&hsdadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Set parameters for SDADC configuration 0 Register
  */
  ConfParamStruct.InputMode = SDADC_INPUT_MODE_SE_ZERO_REFERENCE;
  ConfParamStruct.Gain = SDADC_GAIN_1;
  ConfParamStruct.CommonMode = SDADC_COMMON_MODE_VDDA;
  ConfParamStruct.Offset = 0;
  if (HAL_SDADC_PrepareChannelConfig(&hsdadc2, SDADC_CONF_INDEX_0, &ConfParamStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDADC2_Init 2 */

  /* USER CODE END SDADC2_Init 2 */

}

void HAL_SDADC_MspInit(SDADC_HandleTypeDef* sdadcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(sdadcHandle->Instance==SDADC1)
  {
  /* USER CODE BEGIN SDADC1_MspInit 0 */

  /* USER CODE END SDADC1_MspInit 0 */
    /* SDADC1 clock enable */
    __HAL_RCC_SDADC1_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**SDADC1 GPIO Configuration
    PE8     ------> SDADC1_AIN8P
    */
    GPIO_InitStruct.Pin = Temp_SDADC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(Temp_SDADC_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN SDADC1_MspInit 1 */

  /* USER CODE END SDADC1_MspInit 1 */
  }
  else if(sdadcHandle->Instance==SDADC2)
  {
  /* USER CODE BEGIN SDADC2_MspInit 0 */

  /* USER CODE END SDADC2_MspInit 0 */
    /* SDADC2 clock enable */
    __HAL_RCC_SDADC2_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**SDADC2 GPIO Configuration
    PE9     ------> SDADC2_AIN7P
    */
    GPIO_InitStruct.Pin = Humid_SDADC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(Humid_SDADC_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN SDADC2_MspInit 1 */

  /* USER CODE END SDADC2_MspInit 1 */
  }
}

void HAL_SDADC_MspDeInit(SDADC_HandleTypeDef* sdadcHandle)
{

  if(sdadcHandle->Instance==SDADC1)
  {
  /* USER CODE BEGIN SDADC1_MspDeInit 0 */

  /* USER CODE END SDADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SDADC1_CLK_DISABLE();

    /**SDADC1 GPIO Configuration
    PE8     ------> SDADC1_AIN8P
    */
    HAL_GPIO_DeInit(Temp_SDADC_GPIO_Port, Temp_SDADC_Pin);

  /* USER CODE BEGIN SDADC1_MspDeInit 1 */

  /* USER CODE END SDADC1_MspDeInit 1 */
  }
  else if(sdadcHandle->Instance==SDADC2)
  {
  /* USER CODE BEGIN SDADC2_MspDeInit 0 */

  /* USER CODE END SDADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SDADC2_CLK_DISABLE();

    /**SDADC2 GPIO Configuration
    PE9     ------> SDADC2_AIN7P
    */
    HAL_GPIO_DeInit(Humid_SDADC_GPIO_Port, Humid_SDADC_Pin);

  /* USER CODE BEGIN SDADC2_MspDeInit 1 */

  /* USER CODE END SDADC2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
