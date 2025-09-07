#ifndef GNSS_H
#define GNSS_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "src/libs/micronmea/MicroNMEA.h"
#include "src/globals.h"  // For extUART, gpsUART, nmea if needed

void gnssCheck();
void parseGNSSframe();

#endif // GNSS_H
