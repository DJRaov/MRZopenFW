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
#define adfChipEN PB8
#define adfLoadEN PB9
#define adfCfgClk PC13
#define adfCfgData PC14
#define adfMuxOut PA1
#define adfTXdata PA3

HardwareSerial extUART(PA10, PA9);
HardwareSerial gpsUART(PB4, PB3);

//================================ Variables =================================
//ADF7012 reg0
int fCountOffset = 0;      //F-Counter Offset [-1024 - 1023]
byte rCountDivRatio = 15;  //RF R Counter Divide Ratio [1 - 15]
bool xtalDoubler = 0;      //Crystal Doubler
bool intXOSC = 0;          //Internal XOSC

//ADF7012 reg2
byte modulation = 0;    //[FSK(0), GFSK(1), ASK(2), OOK(3)] (GFSK needs HW mod, refer to ADF7012B datasheet)
bool gaussOOK = 0;      //Needs HW mod, refer to ADF7012B datasheet
int paLevel = 32;       //[0 - 63]
int modDev = 15;        //Refer to datasheet
byte gfskModCtl = 0;    //[0 - 7] (GFSK only)
byte indexCounter = 0;  //[16,32,64,128]

//ADF7012 reg3
bool PLLenable = 1;
bool PAenable = 0;
bool clkOutEnable = 0;
bool dataInvert = 0;

int txFreq = 403000000;  //UHF recommended due to output filter characteristics
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
byte clkOutDivRatio = 8;  //[0 - 15, will be doubled later]
byte vcoAdjust = 0;       //[0 - 4]
int outputDiv = 1;        //[0 - 3, even only; set to 1 if freq <410MHz]

//reg1
int modDivRatio = 0;      //[0 - 4095]
byte NcountDivRatio = 0;  //[0 - 255]
bool prescaler = 0;       //[4/5(0) or 8/9(1)]

//reg3
byte chargePumpI = 3;  //[0 - 3]
bool bleedUp = 0;
bool bleedDown = 0;
bool vcoDisable = 0;
byte muxOut = 0b11;  //Refer to ADF7012B datasheet
bool ldPrecision = 0;
byte vcoBiasI = 7;  //[1 - 15]
byte paBias = 4;    //[0 - 7]

int err = 0;

unsigned long cfgFrameR0 = 0;
unsigned long cfgFrameR1 = 0;
unsigned long cfgFrameR2 = 0;
unsigned long cfgFrameR3 = 0;
//============================================================================

HardwareTimer *okLEDtimer = new HardwareTimer(TIM3);
HardwareTimer *gnssFrameRead = new HardwareTimer(TIM6);

//NMEA Parser setup
char gnssFrameBuffer[85];
MicroNMEA nmea(gnssFrameBuffer, sizeof(gnssFrameBuffer));

void setup() {
  pinMode(okLED, OUTPUT);
  pinMode(adfChipEN, OUTPUT);
  pinMode(adfLoadEN, OUTPUT);
  pinMode(adfCfgClk, OUTPUT);
  pinMode(adfCfgData, OUTPUT);
  pinMode(adfTXdata, OUTPUT);

  digitalWrite(adfChipEN, LOW);
  extUART.begin(115200);
  gpsUART.begin(9600);
  delay(250);
  //GNSS HS mode
  MicroNMEA::sendSentence(gpsUART, "$PMTK101*30");
  MicroNMEA::sendSentence(gpsUART, "$PMTK251,38400*27");
  delay(100);
  gpsUART.flush();
  gpsUART.begin(38400);
  MicroNMEA::sendSentence(gpsUART, "$PMTK101*30");  //hot reset Just in case(tm)

  //GNSS preamble check timer
  gnssFrameRead->attachInterrupt(parseGNSSframe);
  gnssFrameRead->setOverflow(1000, HERTZ_FORMAT);  //PPS is not connected, gotta make do.

  //OK LED feedback timer
  okLEDtimer->setPWM(2, okLED, 4, 50);  //4hz = no GPS lock, 2hz = 2D lock, solid = 3D lock; 15hz = borked gnss
  digitalWrite(okLED, HIGH);
  okLEDtimer->pause();
  gnssFrameRead->pause();

  gnssCheck();
  initTX();
  lockVCO();
  if (err == 0) {
    okLEDtimer->resume();
    //gnssFrameRead->resume();
  }
}
void loop() {
}

void gnssCheck() {             //GNSS presence check routine
  while (millis() <= 10000) {  //GNSS presence check
    if (millis() == 10000) {
      if (gpsAlive == 0) {
        err = 2;
        errorHandler();
      }
    }
    if (char(gpsUART.read()) == 0x24) {
      gpsAlive = 1;
    }
    if (gpsAlive == 1) {
      extUART.write("GNSS found and responding! Sending init commands...\n");
      MicroNMEA::sendSentence(gpsUART, "$PMTK353,1,1*37");
      delay(100);
      MicroNMEA::sendSentence(gpsUART, "$PMTK352,0*2B");
      //MicroNMEA::sendSentence(gpsUART, ); //gonna be used later(tm)
      break;
    }
  }
}

void parseGNSSframe() {  //GNSS frame parser
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

void initTX() {  //ADF7012B initialization routine
  bool readyToSend = 0;
  unsigned long cfgFrame = 0;
  bool adfFail = 0;

  //frequency calculation
  unsigned long f_pfd = 16384000 / (outputDiv * 2);
  NcountDivRatio = (unsigned int)(txFreq / f_pfd);
  float ratio = (float)txFreq / (float)f_pfd;
  float rest = ratio - (float)NcountDivRatio;
  modDivRatio = (unsigned long)(rest * 4096);

  //check for ADF presence
  digitalWrite(adfLoadEN, HIGH);
  delay(5);
  digitalWrite(adfLoadEN, LOW);
  digitalWrite(adfChipEN, HIGH);
  sendADFregister(0);
  sendADFregister(1);
  sendADFregister(2);
  sendADFregister(3);
  delay(100);
  muxOut = 1;
  sendADFregister(3);
  delay(10);
  extUART.println(analogRead(adfMuxOut));
  if (analogRead(adfMuxOut) < 500) {
    err = 5;
    errorHandler();
  }
  muxOut = 0;
  sendADFregister(3);
  delay(10);
  if (analogRead(adfMuxOut) > 500) {
    err = 5;
    errorHandler();
  }
  extUART.println(analogRead(adfMuxOut));
}
void sendADFregister(int regNum) {
  int i = 0;
  unsigned long frame = 0;

  cfgFrameR0 =
    (0) | ((unsigned long)(fCountOffset & 0x7FF) << 2) | ((unsigned long)(rCountDivRatio & 0xF) << 13) | ((unsigned long)(xtalDoubler & 0x1) << 17) | ((unsigned long)(!intXOSC & 0x1) << 18) | ((unsigned long)(clkOutDivRatio & 0xF) << 19) | ((unsigned long)(vcoAdjust & 0x3) << 23) | ((unsigned long)(outputDiv & 0x3) << 25);
  cfgFrameR1 =
    (1) | ((unsigned long)(modDivRatio & 0xFFF) << 2) | ((unsigned long)(NcountDivRatio & 0xFF) << 14) | ((unsigned long)(prescaler & 0x1) << 22);
  cfgFrameR2 =
    (2) | ((unsigned long)(modulation & 0x3) << 2) | ((unsigned long)(gaussOOK & 0x1) << 4) | ((unsigned long)(paLevel & 0x3F) << 5) | ((unsigned long)(modDev & 0x1FF) << 11) | ((unsigned long)(gfskModCtl & 0x7) << 20) | ((unsigned long)(indexCounter & 0x3) << 23);
  cfgFrameR3 =
    (3) | ((unsigned long)(PLLenable & 0x1) << 2) | ((unsigned long)(PAenable & 0x1) << 3) | ((unsigned long)(clkOutEnable & 0x1) << 4) | ((unsigned long)(dataInvert & 0x1) << 5) | ((unsigned long)(chargePumpI & 0x3) << 6) | ((unsigned long)(bleedUp & 0x1) << 8) | ((unsigned long)(bleedDown & 0xF) << 9) | ((unsigned long)(vcoDisable & 0x1) << 10) | ((unsigned long)(muxOut & 0xF) << 11) | ((unsigned long)(ldPrecision & 0x1F) << 15) | ((unsigned long)(vcoBiasI & 0xF) << 16) | ((unsigned long)(paBias & 0xF) << 20);

  digitalWrite(adfChipEN, HIGH);
  digitalWrite(adfLoadEN, HIGH);
  digitalWrite(adfCfgData, LOW);
  digitalWrite(adfCfgClk, LOW);
  delay(2);
  digitalWrite(adfLoadEN, LOW);
  extUART.print("FRAME TX: ");

  switch (regNum) {
    case 0:
      frame = cfgFrameR0;
      break;
    case 1:
      frame = cfgFrameR1;
      break;
    case 2:
      frame = cfgFrameR2;
      break;
    case 3:
      frame = cfgFrameR3;
      break;
  }

  for (i = 31; i >= 0; i--) {
    if ((frame & (unsigned long)(1UL << i)) >> i) {
      digitalWrite(adfCfgData, HIGH);
      extUART.print("1");
    } else {
      digitalWrite(adfCfgData, LOW);
      extUART.print("0");
    }
    delayMicroseconds(10);
    digitalWrite(adfCfgClk, HIGH);
    delayMicroseconds(1);
    digitalWrite(adfCfgClk, LOW);
    delayMicroseconds(1);
  }
  delay(5);
  digitalWrite(adfLoadEN, HIGH);
  extUART.println();
}

void lockVCO() {   //VCO lock algo (yoinked straight from PecanPico)
  muxOut = 0b101;  //analog lock detect
  int adfLocked = 0;
  delay(5);
  unsigned long calcFreq = (16384000 / (outputDiv * 2)) * ((float)NcountDivRatio + ((float)modDivRatio / 4096.0));
  extUART.print("Configuring ADF7012 VCO for ");
  extUART.print(calcFreq);
  extUART.println("Hz");

  for (vcoAdjust = 0; vcoAdjust <= 4; vcoAdjust++) {
    for (vcoBiasI = 1; vcoBiasI <= 15; vcoBiasI++) {
      sendADFregister(0);
      sendADFregister(3);
      delay(150);
      long pllLockSum = 0;
      extUART.print("ADF7012 PLL Lock: ");
      for (int i = 0; i < 100; i++) {  //make sure we are ACTUALLY locked
        extUART.print(", ");
        int pllCurrent = analogRead(adfMuxOut);
        pllLockSum = pllLockSum + pllCurrent;
        extUART.print(pllCurrent);
        delay(1);
      }
      extUART.println();
      extUART.print("Average: ");
      extUART.println(pllLockSum / 100);

      if (pllLockSum / 100 > 850) {
        adfLocked = 1;
        extUART.println("ADF LOCKED!");
        PAenable = 1;
        sendADFregister(3);
        break;
      }
      if (vcoBiasI == 15 && vcoAdjust == 4 && adfLocked == 0) {
        err = 3;
        errorHandler();
        break;
      }
    }
    if (adfLocked == 1) {
      extUART.print("ADF7012 VCO Lock: ");
      extUART.println(analogRead(adfMuxOut));
      break;
    }
  }
}

void errorHandler() {  //basic error handler
  while (true) {
    if (err == 0) {
      extUART.println("Error handler called without an error present! Continuing...");
      delay(1000);
      break;
    }
    int i;
    okLEDtimer->pause();
    delay(300);
    okLEDtimer->setPWM(2, okLED, 3, 50);
    okLEDtimer->resume();
    
    if (err == 2) {
      extUART.println("ERR: No GNSS detected! Cannot continue!");
    }
    if (err == 3) {
      extUART.println("ERR: Could not lock ADF7012 VCO! Cannot continue!");
    }
    if (err == 4) {
      extUART.println("ERR: ADF7012 not powered! Cannot continue!");
    }
    if (err == 5) {
      extUART.println("ERR: ADF7012 is not responding! Cannot continue!");
    }

    delay(int(333.3333333*err));
    okLEDtimer->setPWM(2, okLED, 15, 50);
    okLEDtimer->resume();
    delay(2000);
  }
}