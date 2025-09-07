/*
 * MRZopenFW v.0.069a
 * Copyright (C) 2025 Raov
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program incorporates:
 * - Project Horus FEC library (GPL 3.0)
 * 
 * See LICENSE files for complete terms.
 */

/*
STM32F373 pinout:
PA1: ADF7012B MuxOUT
PA3: ADF7012B TXData
PA4: Internal LED
PA9: External UART TX
PA10: External UART RX
PA13: SWDIO
PA14: SWCLK
PB0: Vbat ADC Input
PB3: GNSS UART TX
PB4: GNSS UART RX
PB8: ADF7012B ChipEnable
PB9: ADF7012B LoadEnable
PC13: ADF7012B CFGCLK
PC14: ADF7012B CFGDATA
PE8: Temperature SDADC
PE9: Humidity SDADC (linearized, thankfully)
*/

#include <Arduino.h>
#include <Math.h>
#include <HardwareSerial.h>
#include <HardwareTimer.h>
#include "src/libs/micronmea/MicroNMEA.h"
#include "src/libs/horusv2/horus_l2.h"
//#include <STM32LowPower.h> soon:tm:
//#include <low_power.h>

#include <stm32f3xx.h>
#include <stm32f3xx_hal.h>
#include <stm32f3xx_hal_sdadc.h>

#include "buildconfig.h"
#include "src/libs/adf7012.h"
#include "src/sdadc.h"
#include "src/libs/misc.h"
#include "src/gnss.h"
#include "src/libs/sensors.h"
#include "src/protocols.h"
#include "src/hal_init.h"

//================================ Variables =================================

uint16_t payloadID = 0;
uint32_t txFreq = 437600000; //in Hz

//(dont) enter A, B and C SDADC cal values here (yet, not implemented)
float ADCa = 0;
float ADCb = 0;
float ADCc = 0;

//============================================================================
//=========================== Internal variables =============================
//============================================================================

//sensor vars
float temp = 0;
float hmd = 0;
float Vbat = 0;

uint8_t rawBuffer[sizeof(TelemetryFrame)];        // Raw telemetry data
uint8_t codedBuffer[sizeof(TelemetryFrame)*2];    // FEC-encoded output buffer
uint16_t encodedLength;                           // encoded frame length
//============================================================================




HardwareTimer *okLEDtimer = new HardwareTimer(TIM3);
HardwareTimer *dataUpdate = new HardwareTimer(TIM6);
HardwareTimer *bitTXtimer = new HardwareTimer(TIM7);

HardwareSerial extUART(PA10, PA9);
HardwareSerial gpsUART(PB4, PB3);

SDADC_HandleTypeDef hsdadc1;
SDADC_HandleTypeDef hsdadc2;

TelemetryFrame tlmFrame;

#if (defined(modHorus)+defined(modAPRS)+defined(modRTTY)) > 1
#error "Transmitting with multiple modes is currently not supported!"
#elif (defined(modHorus)+defined(modAPRS)+defined(modRTTY)) < 1
#error "No modes are defined. Did you forget to uncomment a mode?"
#endif

//NMEA Parser setup
char gnssFrameBuffer[]; //okay maybe going with 255 bytes was a bit too much
MicroNMEA nmea(gnssFrameBuffer, sizeof(gnssFrameBuffer));

void setup() {
  SystemClock_Config();  // Initialize system clock

  pinMode(okLED, OUTPUT);
  pinMode(adfChipEN, OUTPUT);
  pinMode(adfLoadEN, OUTPUT);
  pinMode(adfCfgClk, OUTPUT);
  pinMode(adfCfgData, OUTPUT);
  pinMode(adfTXdata, OUTPUT);
  analogReadResolution(12);

  //start initializing stuff
  GPIOB->BSRR = (1U << (8 + 16U)); //ADF chipEN low
  GPIOA->BSRR = (1U << 4); //turns on LED; we fast like that
  extUART.begin(115200);
  gpsUART.begin(9600);

  #ifdef debug
  extUART.print("MRZopenFW ver.whatever\n\n");
  #endif

  //Telemetry bit transmit interrupt (Reading the GNSS frame seems to be blocking, woulda done it in loop() otherwise)
  #ifdef modHorus
  rCountDivRatio = 8;
  modDev = 3;
  bitTXtimer->setOverflow(100, HERTZ_FORMAT);
  bitTXtimer->attachInterrupt(txNext4FSKSymbol);
  #endif

  #ifdef modAPRS
  bitTXtimer->setOverflow(1200, HERTZ_FORMAT);
  bitTXtimer->attachInterrupt(txAPRS);
  #endif

  #ifdef modRTTY
  bitTXtimer->setOverflow(50, HERTZ_FORMAT);
  bitTXtimer->attachInterrupt(txRTTY);
  #endif

  //Data (temp+humid) update timer
  dataUpdate->setOverflow(1, HERTZ_FORMAT);
  dataUpdate->attachInterrupt(fetchADC);

  okLEDtimer->pause();

  gnssCheck();
  initTX();
  initSDADC();
  lockVCO();
  #ifdef modHorus
  updateHorusFrame();
  #endif
  #if defined(modAPRS) || defined(modRTTY)
  updateUKHASframe();
  #endif
  bitTXtimer->resume();
  dataUpdate->resume();
}

void loop() {  //will have IWDG refresh later(tm)
  parseGNSSframe();

  //check if horus frame needs updating
  #ifdef modHorus
  if (frameSent) updateHorusFrame();
  #endif
}