#include "sdadc.h"
#include "globals.h"
#include "libs/misc.h"

// STM32CubeMX generated code follows
void MX_SDADC1_Init(){

  SDADC_ConfParamTypeDef ConfParamStruct = {};
  hsdadc1.Instance = SDADC1;
  hsdadc1.Init.IdleLowPowerMode = SDADC_LOWPOWER_NONE;
  hsdadc1.Init.FastConversionMode = SDADC_FAST_CONV_DISABLE;
  hsdadc1.Init.SlowClockMode = SDADC_SLOW_CLOCK_DISABLE;
  hsdadc1.Init.ReferenceVoltage = SDADC_VREF_VDDA;
  if (HAL_SDADC_Init(&hsdadc1) != HAL_OK)
  {
    errorHandler(8);
  }
  if (HAL_SDADC_SelectRegularTrigger(&hsdadc1, SDADC_SOFTWARE_TRIGGER) != HAL_OK)
  {
    errorHandler(1);
  }
  ConfParamStruct.InputMode = SDADC_INPUT_MODE_SE_OFFSET;
  ConfParamStruct.Gain = SDADC_GAIN_1;
  ConfParamStruct.CommonMode = SDADC_COMMON_MODE_VDDA;
  ConfParamStruct.Offset = 0;
  if (HAL_SDADC_PrepareChannelConfig(&hsdadc1, SDADC_CONF_INDEX_0, &ConfParamStruct) != HAL_OK)
  {
    errorHandler(1);
  }
}
void MX_SDADC2_Init(){
  SDADC_ConfParamTypeDef ConfParamStruct = {};
  hsdadc2.Instance = SDADC2;
  hsdadc2.Init.IdleLowPowerMode = SDADC_LOWPOWER_NONE;
  hsdadc2.Init.FastConversionMode = SDADC_FAST_CONV_DISABLE;
  hsdadc2.Init.SlowClockMode = SDADC_SLOW_CLOCK_DISABLE;
  hsdadc2.Init.ReferenceVoltage = SDADC_VREF_VDDA;
  if (HAL_SDADC_Init(&hsdadc2) != HAL_OK)
  {
    errorHandler(8);
  }

  ConfParamStruct.InputMode = SDADC_INPUT_MODE_SE_OFFSET;
  ConfParamStruct.Gain = SDADC_GAIN_1_2;
  ConfParamStruct.CommonMode = SDADC_COMMON_MODE_VDDA;
  ConfParamStruct.Offset = 0;
  if (HAL_SDADC_PrepareChannelConfig(&hsdadc2, SDADC_CONF_INDEX_0, &ConfParamStruct) != HAL_OK)
  {
    errorHandler(1);
  }
}

void HAL_SDADC_MspInit(SDADC_HandleTypeDef* sdadcHandle) {

  GPIO_InitTypeDef GPIO_InitStruct = {};
  if(sdadcHandle->Instance==SDADC1)
  {
    /* SDADC1 clock enable */
    __HAL_RCC_SDADC1_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**SDADC1 GPIO Configuration
    PE8     ------> SDADC1_AIN8P
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }
  else if(sdadcHandle->Instance==SDADC2)
  {
    /* SDADC2 clock enable */
    __HAL_RCC_SDADC2_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**SDADC2 GPIO Configuration
    PE9     ------> SDADC2_AIN7P
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }
}
void HAL_SDADC_MspDeInit(SDADC_HandleTypeDef* sdadcHandle){

  if(sdadcHandle->Instance==SDADC1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_SDADC1_CLK_DISABLE();

    /**SDADC1 GPIO Configuration
    PE8     ------> SDADC1_AIN8P
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_8);
  }
  else if(sdadcHandle->Instance==SDADC2)
  {
    /* Peripheral clock disable */
    __HAL_RCC_SDADC2_CLK_DISABLE();

    /**SDADC2 GPIO Configuration
    PE9     ------> SDADC2_AIN7P
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_9);
  }
}

void initSDADC() { //during debugging this i have hugged my wife for a total of: 8 hours
  // Enable SDADC1 and SDADC2 power domains
  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG1 | PWR_SDADC_ANALOG2);

  MX_SDADC1_Init();
  // Select configuration index 0 for SDADC1
  if (HAL_SDADC_SelectRegularTrigger(&hsdadc1, SDADC_SOFTWARE_TRIGGER) != HAL_OK) errorHandler(1);
  if (HAL_SDADC_CalibrationStart(&hsdadc1, SDADC_CALIBRATION_SEQ_1) != HAL_OK) errorHandler(6);
  if (HAL_SDADC_PollForCalibEvent(&hsdadc1, 10000) == HAL_OK) {
    #ifdef debug
    extUART.println("SDADC1 initialized and calibrated.");
    #endif
  } else errorHandler(7);
  if (HAL_SDADC_AssociateChannelConfig(&hsdadc1, SDADC_CHANNEL_8, SDADC_CONF_INDEX_0) != HAL_OK) errorHandler(1);
  if (HAL_SDADC_ConfigChannel(&hsdadc1, SDADC_CHANNEL_8, SDADC_CONTINUOUS_CONV_ON) != HAL_OK) errorHandler(1);
  HAL_SDADC_SelectRegularTrigger(&hsdadc1, SDADC_SOFTWARE_TRIGGER);
  HAL_SDADC_Start(&hsdadc1);

  MX_SDADC2_Init();
  // Select configuration index 0 for SDADC2
  if (HAL_SDADC_SelectRegularTrigger(&hsdadc2, SDADC_SOFTWARE_TRIGGER) != HAL_OK) errorHandler(1);
  if (HAL_SDADC_CalibrationStart(&hsdadc2, SDADC_CALIBRATION_SEQ_1) != HAL_OK) errorHandler(6);
  if (HAL_SDADC_PollForCalibEvent(&hsdadc2, 10000) == HAL_OK) {
    #ifdef debug
    extUART.println("SDADC2 initialized and calibrated.");
    #endif
  } else errorHandler(7);
  if (HAL_SDADC_AssociateChannelConfig(&hsdadc2, SDADC_CHANNEL_7, SDADC_CONF_INDEX_0) != HAL_OK) errorHandler(1);
  if (HAL_SDADC_ConfigChannel(&hsdadc2, SDADC_CHANNEL_7, SDADC_CONTINUOUS_CONV_ON) != HAL_OK) errorHandler(1);
  HAL_SDADC_SelectRegularTrigger(&hsdadc2, SDADC_SOFTWARE_TRIGGER);
  HAL_SDADC_Start(&hsdadc2);
}
