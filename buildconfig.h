#ifndef BUILDCONFIG_H
#define BUILDCONFIG_H

#include <stdint.h>

/* Pins */
#define okLED      PA4
#define adfChipEN  PB8
#define adfLoadEN  PB9
#define adfCfgClk  PC13
#define adfCfgData PC14
#define adfMuxOut  PA1
#define adfTXdata  PA3
#define VbatSense  PB0
#define tempSDADC  PE_8
#define hmdSDADC   PE_9

/* Debug options */
#define debug
// #define debugHorus
// #define debugSensors
// #define debugGNSS

/* Protocols (compile-time) */
#define modHorus
// #define modAPRS
// #define modRTTY

/* Sensors */
#define stockBoom
// #define bmp180

extern uint16_t payloadID;
extern uint32_t txFreq;

#endif
