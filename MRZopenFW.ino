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
//ADF7012 reg0
int fCountOffset = 0;     //F-Counter Offset [-1024 - 1023]
byte rCountDivRatio = 1;  //RF R Counter Divide Ratio [1 - 15]
bool xtalDoubler = 0;     //Crystal Doubler
bool intXOSC = 0;         //Internal XOSC

//ADF7012 reg2
byte modulation = 0;    //[FSK(0), GFSK(1), ASK(2), OOK(3)] (GFSK needs HW mod, refer to ADF7012B datasheet)
bool gaussOOK = 0;      //Needs HW mod, refer to ADF7012B datasheet
int paLevel = 0;        //[0 - 31]
int modDev = 0;         //Refer to datasheet
byte gfskModCtl = 0;    //[0 - 7] (GFSK only)
byte indexCounter = 0;  //[16,32,64,128]

//ADF7012 reg3
bool PLLenable = 1;     
bool PAenable = 1;      
bool clkOutEnable = 0;  
bool dataInvert = 0;    


//=========================== Internal variables =============================
//GNSS vars
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

//ADF7012 vars
//reg0
byte clkOutDivRatio = 16;  //[2 - 30]
byte vcoAdjust = 0;        //[0 - 4]
int outputDiv = 0;         //[0 - 8, even only]

//reg1
int modDivRatio = 0;      //[0 - 4095]
byte NcountDivRatio = 0;  //[0 - 255]
bool prescaler = 0;       //[4/5(0) or 8/9(1)]

//reg3
byte chargePumpI = 2;  //[0 - 3]
bool bleedUp = 0;
bool bleedDown = 0;
bool vcoEnable = 0;
byte muxOut = 0;  //Refer to ADF7012B datasheet
bool ldPrecision = 0;
byte vcoBiasI = 7;  //[1 - 15]
byte paBias = 5;    //[0 - 7]

char err[32];
//============================================================================

HardwareTimer *okLEDtimer = new HardwareTimer(TIM3);
HardwareTimer *gnssFrameRead = new HardwareTimer(TIM6);
HardwareTimer *TXcfgClock = new HardwareTimer(TIM7);

//NMEA Parser setup
char buffer[85];
MicroNMEA nmea(buffer, sizeof(buffer));

void setup() {
  pinMode(okLED, OUTPUT);
  extUART.begin(115200);
  gpsUART.begin(9600);

  //GNSS HS mode
  MicroNMEA::sendSentence(gpsUART, "$PMTK251,38400*27");
  delay(100);
  gpsUART.flush();
  gpsUART.begin(38400);
  MicroNMEA::sendSentence(gpsUART, "$PMTK101*30"); //hot reset Just in case(tm)

  //GNSS preamble check timer
  gnssFrameRead->attachInterrupt(parseGNSSframe);
  gnssFrameRead->setOverflow(1000, HERTZ_FORMAT); //PPS is not connected, gotta make do.

  //OK LED feedback timer
  okLEDtimer->setPWM(2, okLED, 4, 50);  //4hz = no GPS lock, 2hz = 2D lock, solid = 3D lock; 15hz = borked gnss
  digitalWrite(okLED, HIGH);

  while (millis() <= 10000) {  //GNSS presence check
    if (millis() == 10000) {
      if (gpsAlive == 0) {
        //err = "gps";
        errorHandler();
      }
    }
    if (char(gpsUART.read()) == 0x24) {
      gpsAlive = 1;
    }
    if (gpsAlive == 1) {
      okLEDtimer->resume();
      extUART.write("GNSS found and responding! Sending init commands...\n");
      MicroNMEA::sendSentence(gpsUART, "$PMTK353,1,1*37");
      delay(100);
      MicroNMEA::sendSentence(gpsUART, "$PMTK352,0*2B");
      gnssFrameRead->resume();
      //MicroNMEA::sendSentence(gpsUART, ); //gonna be used later(tm)
      break;
    }
  }
}
void loop() {
}

void parseGNSSframe() { //GNSS frame parser
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
      } else {
        okLEDtimer->setPWM(2, okLED, 4, 50);
      }
      gpsValid = bool(nmea.isValid());
      gpsSats = int(nmea.getNumSatellites());
      gpsH = int(nmea.getHour());
      gpsM = int(nmea.getMinute());
      gpsS = int(nmea.getSecond());
      gpsLat = float(nmea.getLatitude()) / 1000000;
      gpsLon = float(nmea.getLongitude()) / 1000000;
      if (nmea.getAltitude(alt)) {
        gpsAlt = alt / 1000;
      } else {
        gpsAlt = -1;
      }
      gpsSpeed = float(nmea.getSpeed()) / 1000;
      gpsCourse = int(nmea.getCourse()) / 1000;
      extUART.print("MSGID: ");
      extUART.print(nmea.getMessageID());
      extUART.print(" | ");
      extUART.print(gpsSats);
      extUART.print(" sats | ");
      extUART.print(gpsH);
      extUART.print(":");
      extUART.print(gpsM);
      extUART.print(":");
      extUART.print(gpsS);
      extUART.print(" | ");
      extUART.print(gpsLat, 4);
      extUART.print(",");
      extUART.print(gpsLon, 4);
      extUART.print(" ");
      extUART.print(gpsAlt);
      extUART.print("m");
      extUART.print(" | ");
      extUART.print(gpsSpeed);
      extUART.print("km/h | ");
      extUART.print(gpsCourse);
      extUART.print("* | ");
      extUART.print(nmea.getSentence());
      extUART.print("\n");
      break;
    }
  }
}

void initTX() { //ADF7012B init void
  bool readyToSend = 0;
  unsigned long cfgFrame = 0;
  for (byte regCount = 0; regCount <= 3; regCount++) {
    switch (regCount) {
      case 0:
        {
          cfgFrame = 
          (0) |
          ((unsigned long)(fCountOffset & 0x7FF) << 2)|
          ((unsigned long)(rCountDivRatio & 0xF) << 13)|
          ((unsigned long)(xtalDoubler & 0x1) << 17)|
          ((unsigned long)(intXOSC & 0x1) << 18)|
          ((unsigned long)(clkOutDivRatio & 0xF) << 19)|
          ((unsigned long)(vcoAdjust & 0x3) << 23)|
          ((unsigned long)(outputDiv & 0x3) << 25);
          extUART.print("Register 0: ");
          extUART.println(cfgFrame, BIN);
          readyToSend=1;
          break;
        }
      case 1:
        {
          cfgFrame = 
          (1) |
          ((unsigned long)(modDivRatio & 0xFFF) << 2)|
          ((unsigned long)(NcountDivRatio & 0xFF) << 14)|
          ((unsigned long)(prescaler & 0x1) << 22);
          extUART.print("Register 1: ");
          extUART.println(cfgFrame, BIN);
          readyToSend=1;
          break;
        }
      case 2:
        {
          cfgFrame = 
          (2) |
          ((unsigned long)(modulation & 0x3) << 2)|
          ((unsigned long)(gaussOOK & 0x1) << 4)|
          ((unsigned long)(paLevel & 0x3F) << 5)|
          ((unsigned long)(modDev & 0x1FF) << 11)|
          ((unsigned long)(gfskModCtl & 0x7) << 20)|
          ((unsigned long)(indexCounter & 0x3) << 23);
          extUART.print("Register 2: ");
          extUART.print(cfgFrame, BIN);
          break;
        }
      case 3:
        {
          cfgFrame =
          (3) |
          ((unsigned long)(PLLenable & 0x1) << 2)|
          ((unsigned long)(PAenable & 0x1) << 3)|
          ((unsigned long)(clkOutEnable & 0x1) << 4)|
          ((unsigned long)(dataInvert & 0x1) << 5)|
          ((unsigned long)(chargePumpI & 0x3) << 6)|
          ((unsigned long)(bleedUp & 0x1) << 8)|
          ((unsigned long)(bleedDown & 0xF) << 9)|
          ((unsigned long)(!vcoEnable & 0x1) << 10)|
          ((unsigned long)(muxOut & 0xF) << 11)|
          ((unsigned long)(ldPrecision & 0x1F) << 15)|
          ((unsigned long)(vcoBiasI & 0xF) << 16)|
          ((unsigned long)(paBias & 0xF) << 20);
          extUART.print("Register 3: ");
          extUART.print(cfgFrame, BIN); 
          break;
        }
    }
    /*if (readyToSend == 1) {
      TXcfgClock->resume();
      noInterrupts();
      // insert writing routine here
      digitalWrite(PB8, HIGH);
      delay(50);
      digitalWrite(PB8, LOW);
    } else {
      TXcfgClock->pause();
      interrupts();
    }*/
  }
}

void errorHandler() {
  okLEDtimer->setPWM(1, okLED, 15, 50);
  okLEDtimer->resume();
  while (true) {
    if (err == "gps") {
      extUART.write("ERR: No GNSS detected! Cannot continue!\n");
      delay(4000);
    }
    if (err == "adfVCOLock"){
      extUART.write("ERR: Could not lock VCO! Cannot continue!");
      delay(4000);
    }
    if (err == "adfPower"){
      extUART.write("ERR: ADF7012 not powered! Cannot continue!");
      delay(4000);
    }
  }
}