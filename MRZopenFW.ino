//MRZ-N1-SIM68 openFW v0.1
//by Raov 2024

/*
STM32F373 pinout:
PA1: ADF7012B MuxOUT
PA3: ADF7012B TXData
PA4: Internal LED
PA9: External UART TX
PA10: External UART RX
PA13: SWD IO
PA14: SWD Clock
PB0: Vbat SDADC Input
PB3: GNSS UART TX
PB4: GNSS UART RX
PB8: ADF7012B ChipEnable
PB9: ADF7012B LoadEnable(?)
PC13: ADF7012B CFGCLK
PC14: ADF7012B CFGDATA
PE8: Sensor 1 SDADC (temp?)
PE9: Sensor 2 SDADC (humidity?)
*/


#include <HardwareSerial.h>
#include <HardwareTimer.h>
#include "Arduino.h"
#include <SPI.h>
#include <MicroNMEA.h>

#define okLED PA4

HardwareSerial extUART(PA10, PA9);
HardwareSerial gpsUART(PB4, PB3);

//================================ Variables =================================
bool gpsAlive = 0;
int gpsSats = 0;
bool gpsValid = 0;
int gpsH = 0;
int gpsM = 0;
int gpsS = 0;
float gpsLat = 0;
float gpsLon = 0;
float gpsAlt = 0;
float gpsSpeed = 0;
float gpsCourse = 0;
//============================================================================

HardwareTimer *okLEDtimer = new HardwareTimer(TIM3);
HardwareTimer *GNSSframeRead = new HardwareTimer(TIM6);

//NMEA Parser setup
char buffer[85];
MicroNMEA nmea(buffer, sizeof(buffer));

void setup() {
  pinMode(okLED, OUTPUT);
  extUART.begin(115200);
  gpsUART.begin(9600);

  //GNSS preamble check timer
  GNSSframeRead->attachInterrupt(parseGNSSframe);
  GNSSframeRead->setOverflow(20, HERTZ_FORMAT);
  GNSSframeRead->resume();

  //OK LED feedback timer
  okLEDtimer->setPWM(2, okLED, 4, 50);  //4hz = no GPS lock, 2hz = 2D lock, solid = 3D lock; 15hz = borked gnss

  //GNSS Configuration
  MicroNMEA::sendSentence(gpsUART, "$PMTK353,1,1*37");
  digitalWrite(okLED, HIGH);
  while (millis() <= 10000) {
    if (millis() == 10000) {
      if (gpsAlive == 0) {
        okLEDtimer->setPWM(1, okLED, 15, 50);
        okLEDtimer->resume();
        while (true) {
          extUART.write("No GNSS detected! Cannot continue!\n");
          delay(4000);
        }
      }
    }
    if (gpsAlive == 1) {
      okLEDtimer->resume();
      extUART.write("GNSS found and responding!\n");
      break;
    }
  }
}

void loop() {
}

void parseGNSSframe() {
  long alt;
  while (gpsUART.available()) {
    char c = gpsUART.read();
    if (nmea.process(c)) {
      gpsAlive = 1;
      if (nmea.getNumSatellites() == 3) {
        okLEDtimer->setPWM(1, okLED, 2, 50);
      } else if (nmea.getNumSatellites() >= 4) {
        okLEDtimer->pause();
        digitalWrite(okLED, HIGH);
      }
      else {
        okLEDtimer->setPWM(2, okLED, 4, 50);
      }
      gpsValid = bool(nmea.isValid());
      gpsSats = int(nmea.getNumSatellites());
      gpsH = int(nmea.getHour());
      gpsM = int(nmea.getMinute());
      gpsS = int(nmea.getSecond());
      gpsLat = int(nmea.getLatitude()) / 1000000;
      gpsLon = int(nmea.getLongitude()) / 1000000;
      if (nmea.getAltitude(alt)) {
        gpsAlt = alt / 1000;
      } else {
        gpsAlt = -1;
      }
      gpsSpeed = int(nmea.getSpeed()) / 1000;
      gpsCourse = int(nmea.getCourse()) / 1000;
      /*extUART.write(gpsSats.convertToCharArray());
      extUART.write(" sats\n");
      extUART.write(gpsH,DEC);
      extUART.write(":");
      extUART.write(gpsM,DEC);
      extUART.write(":");
      extUART.write(gpsS,DEC);
      extUART.write(" | ");
      extUART.write(gpsLat,DEC);
      extUART.write(",");
      extUART.write(gpsLon,DEC);
      extUART.write("\n");*/
      break;
    }
  }
}