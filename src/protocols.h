#ifndef PROTOCOLS_H
#define PROTOCOLS_H

#include <Arduino.h>
#include "globals.h"  // For tlmFrame, rawBuffer, codedBuffer, encodedLength, extUART, nmea, temp, Vbat
#include "src/libs/horusv2/horus_l2.h"
#include "src/libs/adf7012.h"

void updateUKHASframe();
void txRTTY();
void txAPRS();
void updateHorusFrame();
void txNext4FSKSymbol();
uint16_t crc16_ccitt(const uint8_t *data, size_t length);
void printStructHex(const void* ptr, size_t len, HardwareSerial& serial);
extern uint16_t bitIndex;
extern bool frameSent;

#endif // PROTOCOLS_H
