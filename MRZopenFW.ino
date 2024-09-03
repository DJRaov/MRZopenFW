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
PB9: ADF7012B LoadEnable
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
int fCountOffset = 0;  //F-Counter Offset [-1024 - 1023]

//ADF7012 reg2
byte modulation = 0;    //[FSK(0), GFSK(1), ASK(2), OOK(3)] (GFSK needs HW mod, refer to ADF7012B datasheet)
bool gaussOOK = 0;      //Needs HW mod, refer to ADF7012B datasheet
byte paLevel = 3;       //[0 - 63]
byte modDev = 5;        //Refer to datasheet
byte gfskModCtl = 0;    //[0 - 7] (GFSK only)
byte indexCounter = 0;  //[16,32,64,128]

unsigned long txFreq = 404500000;  //UHF recommended due to output filter characteristics
//=========================== Internal variables =============================
//ADF7012 vars
//reg0
byte rCountDivRatio = 2;  //keep at 2
bool xtalDoubler = 0;
bool crystalOSC = 0;      //keep disabled unless you want err3 to occur
byte clkOutDivRatio = 8;  //[0 - 15, will be doubled later]
byte vcoAdjust = 0;       //[0 - 4]
byte outputDiv = 0;       //[0 - 3, even only]

//reg1
int modDivRatio = 0;      //[0 - 4095]
byte NcountDivRatio = 0;  //[0 - 255]
bool prescaler = 0;       //[4/5(0) or 8/9(1)]

//reg3
bool PLLenable = 0;
bool PAenable = 0;
bool clkOutEnable = 0;
bool dataInvert = 0;
byte chargePumpI = 0;  //[0 - 3], large currents not needed
bool bleedUp = 0;
bool bleedDown = 0;
bool vcoDisable = 0;
byte muxOut = 0b11;  //Refer to ADF7012B datasheet
bool ldPrecision = 0;
byte vcoBiasI = 7;  //[1 - 15]
byte paBias = 4;    //[0 - 7]

int err = 0;
//============================================================================

HardwareTimer *okLEDtimer = new HardwareTimer(TIM3);
//HardwareTimer *gnssFrameRead = new HardwareTimer(TIM6);
HardwareTimer *bitTXtimer = new HardwareTimer(TIM7);

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

  //start initializing stuff
  digitalWrite(adfChipEN, LOW);
  extUART.begin(115200);
  gpsUART.begin(9600);
  delay(250);  //wait for the gnss to launch

  //GNSS preamble check timer
  /*gnssFrameRead->attachInterrupt(parseGNSSframe);
  gnssFrameRead->setOverflow(100, HERTZ_FORMAT);  //PPS is not connected, gotta make do.*/

  //Telemetry bit transmit interrupt (Reading the GNSS frame seems to be blocking, woulda done it in loop() otherwise)
  bitTXtimer->setOverflow(2400, HERTZ_FORMAT);
  bitTXtimer->attachInterrupt(txNextTlmBit);

  //OK LED feedback timer
  okLEDtimer->setPWM(2, okLED, 4, 50);  //4hz = no GPS lock, 2hz = 2D lock, solid = 3D lock; 15hz = borked gnss
  digitalWrite(okLED, HIGH);
  okLEDtimer->pause();
  //gnssFrameRead->pause();

  gnssCheck();
  initTX();
  lockVCO();
  if (err == 0) {
    okLEDtimer->resume();
    bitTXtimer->resume();
    //gnssFrameRead->resume();
  }
}
void loop() {  //empty for now, will have IWDG refresh later(tm)
  parseGNSSframe();
}

void gnssCheck() {  //GNSS presence check routine
  bool gpsAlive = 0;
  while (millis() <= 3000) {
    bool gnssHS = 0;
    /*if (millis() == 1000 && gpsAlive == 0) {
      extUART.println("GNSS not found at 9600bd, trying 230400bd... (Hot start?)");
      gpsUART.flush();
      gpsUART.begin(230400);
      gnssHS = 1;
    }*/
    if (millis() == 3000) {
      if (gpsAlive == 0) {
        err = 2;
        errorHandler();
      }
    }
    if (char(gpsUART.read()) == 0x24) {
      gpsAlive = 1;
      extUART.write("GNSS found and responding! Sending init commands...\n");

      //GNSS HS mode
      /*if (gnssHS == 0) {
        MicroNMEA::sendSentence(gpsUART, "$PMTK251,230400*1D");
        delay(100);
        gpsUART.flush();
        gpsUART.begin(230400);
      }*/

      MicroNMEA::sendSentence(gpsUART, "$PMTK101*30");  //hot reset Just in case(tm)
      MicroNMEA::sendSentence(gpsUART, "$PMTK353,1,1*37");
      delay(100);
      MicroNMEA::sendSentence(gpsUART, "$PMTK352,0*2B");
      delay(100);
      //MicroNMEA::sendSentence(gpsUART, "$PMTK314,0,1,0,0,5,5,0,0,0,0,0,0,0,0,0,0,0,0*35");
      break;
    }
  }
}

void initTX() {  //ADF7012B initialization routine
  //frequency calculation
  unsigned long f_pfd = 16384000 / rCountDivRatio;
  float ratio = (float)txFreq / (float)f_pfd;
  float rest = ratio - (float)NcountDivRatio;

  NcountDivRatio = (unsigned int)(txFreq / f_pfd);
  modDivRatio = (unsigned long)(rest * 4096);

  //check for ADF presence
  digitalWrite(adfChipEN, HIGH);
  digitalWrite(adfLoadEN, HIGH);
  delay(5);
  digitalWrite(adfLoadEN, LOW);
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

  unsigned long cfgFrameR0 =
    (0) | ((unsigned long)(fCountOffset & 0x7FF) << 2) | ((unsigned long)(rCountDivRatio & 0xF) << 13) | ((unsigned long)(xtalDoubler & 0x1) << 17) | ((unsigned long)(!crystalOSC & 0x1) << 18) | ((unsigned long)(clkOutDivRatio & 0xF) << 19) | ((unsigned long)(vcoAdjust & 0x3) << 23) | ((unsigned long)(outputDiv & 0x3) << 25);
  unsigned long cfgFrameR1 =
    (1) | ((unsigned long)(modDivRatio & 0xFFF) << 2) | ((unsigned long)(NcountDivRatio & 0xFF) << 14) | ((unsigned long)(prescaler & 0x1) << 22);
  unsigned long cfgFrameR2 =
    (2) | ((unsigned long)(modulation & 0x3) << 2) | ((unsigned long)(gaussOOK & 0x1) << 4) | ((unsigned long)(paLevel & 0x3F) << 5) | ((unsigned long)(modDev & 0x1FF) << 11) | ((unsigned long)(gfskModCtl & 0x7) << 20) | ((unsigned long)(indexCounter & 0x3) << 23);
  unsigned long cfgFrameR3 =
    (3) | ((unsigned long)(PLLenable & 0x1) << 2) | ((unsigned long)(PAenable & 0x1) << 3) | ((unsigned long)(clkOutEnable & 0x1) << 4) | ((unsigned long)(dataInvert & 0x1) << 5) | ((unsigned long)(chargePumpI & 0x3) << 6) | ((unsigned long)(bleedUp & 0x1) << 8) | ((unsigned long)(bleedDown & 0xF) << 9) | ((unsigned long)(vcoDisable & 0x1) << 10) | ((unsigned long)(muxOut & 0xF) << 11) | ((unsigned long)(ldPrecision & 0x1F) << 15) | ((unsigned long)(vcoBiasI & 0xF) << 16) | ((unsigned long)(paBias & 0xF) << 20);

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

  if (regNum == 1) {
    for (i = 23; i >= 0; i--) {
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
  } else {
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
  }
  delay(5);
  digitalWrite(adfLoadEN, HIGH);
  extUART.println();
}
void lockVCO() {   //VCO lock algo (yoinked straight from PecanPico)
  muxOut = 0b101;  //analog lock detect
  int adfLocked = 0;
  delay(5);
  PLLenable = 1;
  unsigned long calcFreq = (16384000 / rCountDivRatio) * ((float)NcountDivRatio + ((float)modDivRatio / 4096.0));  //borked, but ADF locks fine
  extUART.print("Locking ADF7012 VCO for ");
  extUART.print(calcFreq);
  extUART.println("Hz");

  for (vcoAdjust = 0; vcoAdjust <= 4; vcoAdjust++) {
    for (vcoBiasI = 1; vcoBiasI <= 15; vcoBiasI++) {
      sendADFregister(0);
      sendADFregister(3);
      long pllLockSum = 0;
      extUART.print("ADF7012 PLL Lock: ");
      for (int i = 0; i < 100; i++) {  //make sure we are ACTUALLY locked in
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
        extUART.println("ADF VCO LOCKED!");
        PAenable = 1;
        sendADFregister(3);
        break;
      }
      if (vcoBiasI == 15 && vcoAdjust == 4 && adfLocked == 0) {
        err = 3;
        errorHandler();
      }
    }
    if (adfLocked == 1) {
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
      extUART.println("ERR: No GNSS detected. Cannot continue.");
    }
    if (err == 3) {
      extUART.println("ERR: Could not lock ADF7012 VCO. Cannot continue.");
    }
    if (err == 4) {
      extUART.println("ERR: ADF7012 not powered. Cannot continue.");
    }
    if (err == 5) {
      extUART.println("ERR: ADF7012 is not responding. Cannot continue.");
    }

    delay(int(333.3333333 * err));
    okLEDtimer->setPWM(2, okLED, 15, 50);
    okLEDtimer->resume();
    delay(2000);
  }
}

void txNextTlmBit() {
  static byte frameCnt;
  static byte i;
  static byte partCount;
  static uint32_t tlmFrame;
  static uint32_t tlmFrame1 =
    (uint32_t(0x55aa) << 16) | ((uint32_t(frameCnt) & 0xFF) << 8);
  static uint64_t tlmFrame2 =
    ((uint32_t(nmea.getHour()) & 0x1F) << 27) | ((uint32_t(nmea.getMinute()) & 0x3F) << 21) | ((uint32_t(nmea.getSecond()) & 0x3F) << 15) | ((uint32_t(nmea.getDay()) & 0x1F) << 10) | ((uint32_t(nmea.getMonth()) & 0xF) << 6) | (uint32_t(nmea.getYear()) & 0x3F);
  static uint32_t tlmFrame3 =
    (nmea.getLatitude());
  static uint32_t tlmFrame4 =
    (nmea.getLongitude());
  static uint32_t tlmFrame5 =
    ((uint32_t(nmea.isValid()) & 0x1) << 32) | ((uint32_t(nmea.getNumSatellites()) & 0x1F) << 27);


  switch (partCount) {
    case 0:
      tlmFrame = tlmFrame1;
      break;

    case 1:
      tlmFrame = tlmFrame2;
      break;

    case 2:
      tlmFrame = tlmFrame3;
      break;

    case 3:
      tlmFrame = tlmFrame4;
      break;

    case 4:
      tlmFrame = tlmFrame5;
  }

  if ((tlmFrame & uint32_t(1U << i)) >> i) {
    digitalWrite(adfTXdata, HIGH);
  } else {
    digitalWrite(adfTXdata, LOW);
  }
  i++;
  if (i >= 33) {
    i = 0;
    partCount++;
  };
  if (partCount > 4) {
    partCount = 0;
    frameCnt++;
  }
}
void parseGNSSframe() {  //GNSS frame parser
  while (gpsUART.available()) {
    char c = gpsUART.read();
    if (nmea.process(c)) {
      bool gpsValid = 0;
      int gpsAlt = 0;
      float gpsSpeed = 0;
      int gpsCourse = 0;
      long alt;
      if (nmea.getNumSatellites() == 3) {
        okLEDtimer->setPWM(1, okLED, 2, 50);
      } else if (nmea.getNumSatellites() >= 4) {
        okLEDtimer->pause();
        digitalWrite(okLED, HIGH);
      } else {
        okLEDtimer->setPWM(2, okLED, 4, 50);
      }
      gpsValid = nmea.isValid();
      if (nmea.getAltitude(alt)) {
        gpsAlt = alt / 1000;
      } else {
        gpsAlt = -1;
      }
      gpsSpeed = float(nmea.getSpeed() / 1000);
      gpsCourse = nmea.getCourse() / 1000;
      extUART.print("MSGID: ");
      extUART.print(nmea.getMessageID());
      extUART.print(" | ");
      extUART.print(nmea.getNumSatellites());
      extUART.print(" sats | ");
      extUART.print(nmea.getHour());
      extUART.print(":");
      extUART.print(nmea.getMinute());
      extUART.print(":");
      extUART.print(nmea.getSecond());
      extUART.print(" | ");
      extUART.print(float(nmea.getLatitude() / 1000000), 4);
      extUART.print(",");
      extUART.print(float(nmea.getLongitude() / 1000000), 4);
      extUART.print(" ");
      extUART.print(gpsAlt);
      extUART.print("m");
      extUART.print(" | ");
      extUART.print(nmea.getSpeed() / 1000, 2);
      extUART.print("km/h | ");
      extUART.print(gpsCourse);
      extUART.print("* | ");
      extUART.print(nmea.getSentence());
      extUART.print("\n");
    }
  }
}
