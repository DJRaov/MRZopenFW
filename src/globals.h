#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <HardwareTimer.h>
#include "src/libs/micronmea/MicroNMEA.h"
#include <stm32f3xx_hal.h>
#include <stm32f3xx_hal_sdadc.h>

// Extern declarations for global variables defined in MRZopenFW.ino

extern float ADCa;
extern float ADCb;
extern float ADCc;

extern float temp;
extern float hmd;
extern float Vbat;

struct __attribute__((packed)) TelemetryFrame {
  uint16_t payloadID;
  uint16_t seqNum;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  float lat;
  float lon;
  uint16_t alt;
  uint8_t spd;
  uint8_t satCount;
  int8_t temp;
  uint8_t vbat; //vbat*50
  uint8_t custom1;
  uint64_t custom2;
  uint16_t crc;
};

extern uint8_t rawBuffer[sizeof(TelemetryFrame)];
extern uint8_t codedBuffer[sizeof(TelemetryFrame)*2];
extern uint16_t encodedLength;

extern HardwareTimer *okLEDtimer;
extern HardwareTimer *dataUpdate;
extern HardwareTimer *bitTXtimer;

extern HardwareSerial extUART;
extern HardwareSerial gpsUART;

extern SDADC_HandleTypeDef hsdadc1;
extern SDADC_HandleTypeDef hsdadc2;

extern TelemetryFrame tlmFrame;

extern char gnssFrameBuffer[127];
extern MicroNMEA nmea;

#endif // GLOBALS_H
