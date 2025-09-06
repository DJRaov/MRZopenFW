#ifndef SDADC_H
#define SDADC_H

#include <Arduino.h>

void MX_SDADC1_Init(void);
void MX_SDADC2_Init(void);

void HAL_SDADC_MspInit(SDADC_HandleTypeDef* sdadcHandle);
void HAL_SDADC_MspDeInit(SDADC_HandleTypeDef* sdadcHandle);

void initSDADC(void);

#endif