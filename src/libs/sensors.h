#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <stm32f3xx_hal.h>
#include <stm32f3xx_hal_sdadc.h>
#include "src/globals.h"  // For temp, hmd, Vbat, hsdadc1/2, extUART

float convertTemperature(int16_t rawSDADC);
void fetchADC();

#endif // SENSORS_H
