//MRZ-N1-SIM68 openFW v0.069 Alpha
//by Raov 2025

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
#include <MicroNMEA.h>

#include <stm32f3xx.h>
#include <stm32f3xx_hal.h>
#include <stm32f3xx_hal_sdadc.h>

#define okLED PA4
#define adfChipEN PB8
#define adfLoadEN PB9
#define adfCfgClk PC13
#define adfCfgData PC14
#define adfMuxOut PA1
#define adfTXdata PA3
#define VbatSense PB0
#define tempSDADC PE_8
#define hmdSDADC PE_9

//================================ Variables =================================
#define debug
//#define debugGNSS

#define stockBoom
//#define bmp280

//ADF7012 reg0
int fCountOffset = 0;  //F-Counter Offset [-1024 - 1023]

//ADF7012 reg2
byte modulation = 0;    //[FSK(0), GFSK(1), ASK(2), OOK(3)] (GFSK needs HW mod, refer to mods/readme)
bool gaussOOK = 0;      //Needs HW mod, refer to mods/readme
byte paLevel = 3;       //[0 - 63]
int modDev = 3;         //[0 - 511] (500hz/step with rCountDivRatio=2; Divider factor [0-127] if GFSK selected)
byte gfskModCtl = 0;    //[0 - 7] (GFSK only)
byte indexCounter = 0;  //[16,32,64,128] (GFSK only)

//(dont) enter A, B and C SDADC cal values here (yet, not implemented)
float ADCa = 0;
float ADCb = 0;
float ADCc = 0;

unsigned long txFreq = 404500000;  //UHF recommended due to output filter characteristics
//============================================================================
//=========================== Internal variables =============================

//ADF7012 vars
//reg0
uint8_t rCountDivRatio = 4;  //values over 12 seem to upset the VCO
bool xtalDoubler = 0;        //doubles XO, but who needs that?
bool crystalOSC = 0;         //keep disabled unless you want err3 to occur
uint8_t clkOutDivRatio = 8;  //[0 - 15, unused]
uint8_t vcoAdjust = 0;       //[0 - 4]
uint8_t outputDiv = 0;       //[0 - 3, even only]

//reg1
int modDivRatio = 0;         //[0 - 4095]
uint8_t NcountDivRatio = 0;  //[0 - 255]
bool prescaler = 0;          //[4/5(0) or 8/9(1)]

//reg3
bool PLLenable = 0;
bool PAenable = 0;
bool clkOutEnable = 0;
bool dataInvert = 0;
uint8_t chargePumpI = 0;  //[0 - 3], large currents not needed
bool bleedUp = 0;
bool bleedDown = 0;
bool vcoDisable = 0;
uint8_t muxOut = 0b11;  //Refer to ADF7012B datasheet
bool ldPrecision = 0;
uint8_t vcoBiasI = 7;  //[1 - 15]
uint8_t paBias = 4;    //[0 - 7]

//sensor vars
float temp = 0;
float hmd = 0;
float Vbat = 0;
//============================================================================


HardwareTimer *okLEDtimer = new HardwareTimer(TIM3);
HardwareTimer *dataUpdate = new HardwareTimer(TIM6);
HardwareTimer *bitTXtimer = new HardwareTimer(TIM7);

HardwareSerial extUART(PA10, PA9);
HardwareSerial gpsUART(PB4, PB3);

 SDADC_HandleTypeDef hsdadc1;
 SDADC_HandleTypeDef hsdadc2;

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
  analogReadResolution(12);

  //start initializing stuff
  digitalWrite(adfChipEN, LOW);
  extUART.begin(115200);
  gpsUART.begin(9600);
  HAL_Delay(150);  //wait for the gnss to launch

  //Telemetry bit transmit interrupt (Reading the GNSS frame seems to be blocking, woulda done it in loop() otherwise)
  bitTXtimer->setOverflow(300, HERTZ_FORMAT);
  bitTXtimer->attachInterrupt(txNextTlmBit);

  //Data (temp+humid) update timer
  dataUpdate->setOverflow(1, HERTZ_FORMAT);
  dataUpdate->attachInterrupt(fetchADC);

  //OK LED feedback timer
  okLEDtimer->setPWM(2, okLED, 4, 50);  //4hz = no GPS lock, 2hz = 2D lock, solid = 3D lock; 15hz = borked gnss
  digitalWrite(okLED, HIGH);
  okLEDtimer->pause();

  gnssCheck();
  initTX();
  initSDADC();
  lockVCO();
  okLEDtimer->resume();
  bitTXtimer->resume();
  dataUpdate->resume();
}
void loop() {  //empty for now, will have IWDG refresh later(tm)
  parseGNSSframe();
}

void gnssCheck() {  //GNSS presence check routine
  bool gpsAlive = 0;
  while (HAL_GetTick() <= 3000) {
    if (HAL_GetTick() == 3000) {
      if (gpsAlive == 0) {
        errorHandler(2);
      }
    }
    if (char(gpsUART.read()) == 0x24) {
      gpsAlive = 1;
      #ifdef debug
      extUART.write("GNSS found. Sending init commands...\n");
      #endif
      MicroNMEA::sendSentence(gpsUART, "$PMTK101");  //hot reset Just in case(tm)
      //delay(50);
      MicroNMEA::sendSentence(gpsUART, "$PMTK353,1,1");
      //delay(50);
      MicroNMEA::sendSentence(gpsUART, "$PMTK352,0");
      //delay(50);
      //MicroNMEA::sendSentence(gpsUART, "$PMTK314,0,1,0,0,5,5,0,0,0,0,0,0,0,0,0,0,0,0*35"); //send less, interfere less (TODO: fix UART interrupt and TX timer interference)
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
  HAL_Delay(5);
  digitalWrite(adfLoadEN, LOW);
  sendADFregister(0);
  sendADFregister(1);
  sendADFregister(2);
  muxOut = 1;
  sendADFregister(3);
  HAL_Delay(10);
  if (analogRead(adfMuxOut) < 500) {
    errorHandler(5);
  }
  muxOut = 0;
  sendADFregister(3);
  HAL_Delay(10);
  if (analogRead(adfMuxOut) > 500) {
    errorHandler(5);
  }
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
  HAL_Delay(2);
  digitalWrite(adfLoadEN, LOW);

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
      } else {
        digitalWrite(adfCfgData, LOW);
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
      } else {
        digitalWrite(adfCfgData, LOW);
      }
      delayMicroseconds(10);
      digitalWrite(adfCfgClk, HIGH);
      delayMicroseconds(1);
      digitalWrite(adfCfgClk, LOW);
      delayMicroseconds(1);
    }
  }
  HAL_Delay(5);
  digitalWrite(adfLoadEN, HIGH);
}
void lockVCO() {   //VCO lock algo (yoinked straight from PecanPico)
  muxOut = 0b101;  //analog lock detect
  bool adfLocked = 0;
  PLLenable = 1;

  for (vcoAdjust = 0; vcoAdjust <= 4; vcoAdjust++) {
    for (vcoBiasI = 1; vcoBiasI <= 15; vcoBiasI++) {
      sendADFregister(0);
      sendADFregister(3);
      long pllLockSum = 0;

      for (int i = 0; i < 100; i++) {  //make sure we are ACTUALLY locked in
        int pllCurrent = analogRead(adfMuxOut);
        pllLockSum = pllLockSum + pllCurrent;
        HAL_Delay(1);
      }

      if (pllLockSum / 100 > 850) {
        adfLocked = 1;
        PAenable = 1;
        sendADFregister(3);
        break;
      }
      if (vcoBiasI == 15 && vcoAdjust == 4 && adfLocked == 0) {
        errorHandler(3);
      }
    }
    if (adfLocked == 1) {
      break;
    }
  }
}

void txNextTlmBit() {
  static byte frameCnt;
  static byte i;
  static byte partCount;
  static uint32_t tlmFrame;
  
  //format very much subject to change, nothing is set in stone yet
  static uint32_t tlmFrame1 =
    ((uint32_t(0x55aa) & 0xFFFF) << 16) | ((uint32_t(frameCnt) & 0xFF) << 8);
  static uint32_t tlmFrame2 =
    ((uint32_t(nmea.getHour()) & 0x1F) << 27) | ((uint32_t(nmea.getMinute()) & 0x3F) << 21) | ((uint32_t(nmea.getSecond()) & 0x3F) << 15) | ((uint32_t(nmea.getDay()) & 0x1F) << 10) | ((uint32_t(nmea.getMonth()) & 0xF) << 6) | (uint32_t(nmea.getYear()) & 0x3F);
  static uint32_t tlmFrame3 =
    (nmea.getLatitude());
  static uint32_t tlmFrame4 =
    (nmea.getLongitude());
  static uint32_t tlmFrame5 =
    ((uint32_t(nmea.isValid()) & 0x1) << 31) | ((uint32_t(nmea.getNumSatellites()) & 0x1F) << 26) | ((uint32_t(0xFF) & 0xFF) << 20) | ((uint32_t(Vbat * 100) & 0x3FF) << 10);
  static uint32_t tlmFrame6 =
    ((uint32_t(0x5555) & 0xFFFF) << 16) | (uint32_t(0x5555) & 0xFFFF);

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
      break;

    case 5:
      tlmFrame = tlmFrame6;
      break;
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
  if (partCount > 5) {
    partCount = 0;
    frameCnt++;
  }
}

void parseGNSSframe() {  //GNSS frame parser
  while (gpsUART.available()) {
    char c = gpsUART.read();
    if (nmea.process(c)) {
      int gpsAlt = 0;
      long alt;
      if (nmea.getNumSatellites() == 3) {
        okLEDtimer->setPWM(1, okLED, 2, 50);
      } else if (nmea.getNumSatellites() >= 4) {
        okLEDtimer->pause();
        digitalWrite(okLED, HIGH);
      } else {
        okLEDtimer->setPWM(2, okLED, 4, 50);
      }
      if (nmea.getAltitude(alt)) {
        gpsAlt = alt / 1000;
      } else {
        gpsAlt = -1;
      }
      #ifdef debugGNSS
      extUART.println("MSGID: " + String(nmea.getMessageID()) + " | " + String(nmea.getNumSatellites()) + " sats | " + String(nmea.getHour()) + ":" + String(nmea.getMinute()) + ":" + String(nmea.getSecond()) + " | " + String(float(nmea.getLatitude() / 1000000), 4) + "," + String(float(nmea.getLongitude() / 1000000), 4) + " " + String(gpsAlt) + "m | " + String(nmea.getSpeed() / 1000, 2) + "km/h | " + String(gpsCourse) + "* | " + nmea.getSentence());
      #endif
    }
  }
}

void fetchADC() {
  HAL_SDADC_PollForConversion(&hsdadc1, HAL_MAX_DELAY);
  uint32_t rawTemp32 = HAL_SDADC_GetValue(&hsdadc1);
  uint16_t rawTemp = (uint16_t)(rawTemp32 & 0xFFFF); //stm32duino why are you like this. why must you torture me.

  HAL_SDADC_PollForConversion(&hsdadc2, HAL_MAX_DELAY);
  uint32_t rawHumid = HAL_SDADC_GetValue(&hsdadc2);

  int rawVbat = analogRead(VbatSense);

  temp = convertTemperature(rawTemp); //very not metrology-grade, beware
  hmd = 112.5*(((float(rawHumid)/32767.0f)*3.3f)/3.3);
  Vbat = (rawVbat * 1.6117216117) / 1000;
  #ifdef debug
  extUART.println("tempADC raw: " + String(rawTemp32) + " | Temperature: " + String(temp) + "c");
  extUART.println("humidADC raw: " + String(rawHumid) + " | Humidity: " + String(hmd) + "%");
  extUART.println("Vbat raw: " + String(rawVbat) + " | Battery voltage: " + String(Vbat) + "v");
  #endif

}

float convertTemperature(int16_t rawSDADC) {
    //NTC coeffs
    const float calA = 0.01563;      //NTC A
    const float calB = 4684.40039;   //NTC B  
    const float calC = 52.38283;     //NTC C
    
    //ADC coeffs
    const float A_adcT = 0.0;        //ADC A
    const float B_adcT = 0.97501;    //ADC B
    const float C_adcT = 1024.18994; //ADC C
    
    const float ADC_MAX = 32767.0;
    
    uint32_t rawU = (int32_t)rawSDADC;
    float adc_t = rawU / 3.0f;  //winged it, should become more accurate soon(tm)

    float poly1 = adc_t * adc_t * A_adcT + adc_t * B_adcT + C_adcT;
    float Rt = 100000.0f * poly1 / (ADC_MAX - poly1);

    if (Rt > 0.0f && calA > 0.0f) {
        float t = calB / logf(Rt / calA) - calC - 273.15f;
        if (t < -120.0f || t > 120.0f) return -273.15f;
        return t;
    }
    return -273.15f;
}

void errorHandler(uint8_t err) {  //basic error handler
  while (true) {
    okLEDtimer->pause();
    okLEDtimer->setPWM(2, okLED, 3, 50);
    okLEDtimer->resume();

    switch (err) {
      case 1:
        extUART.println("ERR: A generic STM32 HAL error has occurred. Cannot continue.");
        break;

      case 2:
        extUART.println("ERR: No GNSS detected. Cannot continue.");
        break;

      case 3:
        extUART.println("ERR: Could not lock ADF7012 VCO. Cannot continue.");
        break;

      case 4:
        extUART.println("ERR: ADF7012 not powered. Cannot continue.");
        break;

      case 5:
        extUART.println("ERR: ADF7012 is not responding. Cannot continue.");
        break;

      case 6:
        extUART.println("ERR: Failed to start SDADC calibration. Cannot continue.");
        break;

      case 7:
        extUART.println("ERR: One of SDADCs failed to calibrate within specified time frame. Cannot continue.");
        break;

      default:
        extUART.println("An undefined error has been passed. This should not happen. Passed error code: " + err);
        break;
    }

    HAL_Delay(int(333.3333333 * err - 1));
    okLEDtimer->setPWM(2, okLED, 15, 50);
    okLEDtimer->resume();
    HAL_Delay(3000);
  }
}

//during debugging this i have hugged my wife for a total of: 7 hours
void initSDADC() {
  // Enable SDADC1 and SDADC2 power domains
  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG1 | PWR_SDADC_ANALOG2);

  MX_SDADC1_Init();
  // Select configuration index 0 for SDADC1
  if (HAL_SDADC_SelectRegularTrigger(&hsdadc1, SDADC_SOFTWARE_TRIGGER) != HAL_OK) errorHandler(1);
  if (HAL_SDADC_CalibrationStart(&hsdadc1, SDADC_CALIBRATION_SEQ_1) != HAL_OK) errorHandler(6);
  if (HAL_SDADC_PollForCalibEvent(&hsdadc1, 10000) == HAL_OK) {
    #ifdef debug
    extUART.println("SDADC1 initialized and calibrated.");
    #endif
  } else errorHandler(7);
  if (HAL_SDADC_AssociateChannelConfig(&hsdadc1, SDADC_CHANNEL_8, SDADC_CONF_INDEX_0) != HAL_OK) errorHandler(1);
  if (HAL_SDADC_ConfigChannel(&hsdadc1, SDADC_CHANNEL_8, SDADC_CONTINUOUS_CONV_ON) != HAL_OK) errorHandler(1);
  HAL_SDADC_SelectRegularTrigger(&hsdadc1, SDADC_SOFTWARE_TRIGGER);
  HAL_SDADC_Start(&hsdadc1);

  MX_SDADC2_Init();
  // Select configuration index 0 for SDADC2
  if (HAL_SDADC_SelectRegularTrigger(&hsdadc2, SDADC_SOFTWARE_TRIGGER) != HAL_OK) errorHandler(1);
  if (HAL_SDADC_CalibrationStart(&hsdadc2, SDADC_CALIBRATION_SEQ_1) != HAL_OK) errorHandler(6);
  if (HAL_SDADC_PollForCalibEvent(&hsdadc2, 10000) == HAL_OK) {
    #ifdef debug
    extUART.println("SDADC2 initialized and calibrated.");
    #endif
  } else errorHandler(7);
  if (HAL_SDADC_AssociateChannelConfig(&hsdadc2, SDADC_CHANNEL_7, SDADC_CONF_INDEX_0) != HAL_OK) errorHandler(1);
  if (HAL_SDADC_ConfigChannel(&hsdadc2, SDADC_CHANNEL_7, SDADC_CONTINUOUS_CONV_ON) != HAL_OK) errorHandler(1);
  HAL_SDADC_SelectRegularTrigger(&hsdadc2, SDADC_SOFTWARE_TRIGGER);
  HAL_SDADC_Start(&hsdadc2);
}

// STM32CubeMX generated code follows
/* SDADC1 init function */
void MX_SDADC1_Init(){

  SDADC_ConfParamTypeDef ConfParamStruct = {};
  hsdadc1.Instance = SDADC1;
  hsdadc1.Init.IdleLowPowerMode = SDADC_LOWPOWER_NONE;
  hsdadc1.Init.FastConversionMode = SDADC_FAST_CONV_DISABLE;
  hsdadc1.Init.SlowClockMode = SDADC_SLOW_CLOCK_DISABLE;
  hsdadc1.Init.ReferenceVoltage = SDADC_VREF_EXT;
  if (HAL_SDADC_Init(&hsdadc1) != HAL_OK)
  {
    errorHandler(1);
  }

  ConfParamStruct.InputMode = SDADC_INPUT_MODE_SE_OFFSET;
  ConfParamStruct.Gain = SDADC_GAIN_1;
  ConfParamStruct.CommonMode = SDADC_COMMON_MODE_VDDA;
  ConfParamStruct.Offset = 0;
  if (HAL_SDADC_PrepareChannelConfig(&hsdadc1, SDADC_CONF_INDEX_0, &ConfParamStruct) != HAL_OK)
  {
    errorHandler(1);
  }
}
/* SDADC2 init function */
void MX_SDADC2_Init(){
  SDADC_ConfParamTypeDef ConfParamStruct = {};
  hsdadc2.Instance = SDADC2;
  hsdadc2.Init.IdleLowPowerMode = SDADC_LOWPOWER_NONE;
  hsdadc2.Init.FastConversionMode = SDADC_FAST_CONV_DISABLE;
  hsdadc2.Init.SlowClockMode = SDADC_SLOW_CLOCK_DISABLE;
  hsdadc2.Init.ReferenceVoltage = SDADC_VREF_VDDA;
  if (HAL_SDADC_Init(&hsdadc2) != HAL_OK)
  {
    errorHandler(1);
  }

  ConfParamStruct.InputMode = SDADC_INPUT_MODE_SE_OFFSET;
  ConfParamStruct.Gain = SDADC_GAIN_1_2;
  ConfParamStruct.CommonMode = SDADC_COMMON_MODE_VDDA;
  ConfParamStruct.Offset = 0;
  if (HAL_SDADC_PrepareChannelConfig(&hsdadc2, SDADC_CONF_INDEX_0, &ConfParamStruct) != HAL_OK)
  {
    errorHandler(1);
  }
}

void HAL_SDADC_MspInit(SDADC_HandleTypeDef* sdadcHandle) {

  GPIO_InitTypeDef GPIO_InitStruct = {};
  if(sdadcHandle->Instance==SDADC1)
  {
    /* SDADC1 clock enable */
    __HAL_RCC_SDADC1_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**SDADC1 GPIO Configuration
    PE8     ------> SDADC1_AIN8P
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }
  else if(sdadcHandle->Instance==SDADC2)
  {
    /* SDADC2 clock enable */
    __HAL_RCC_SDADC2_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**SDADC2 GPIO Configuration
    PE9     ------> SDADC2_AIN7P
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }
}

void HAL_SDADC_MspDeInit(SDADC_HandleTypeDef* sdadcHandle){

  if(sdadcHandle->Instance==SDADC1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_SDADC1_CLK_DISABLE();

    /**SDADC1 GPIO Configuration
    PE8     ------> SDADC1_AIN8P
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_8);
  }
  else if(sdadcHandle->Instance==SDADC2)
  {
    /* Peripheral clock disable */
    __HAL_RCC_SDADC2_CLK_DISABLE();

    /**SDADC2 GPIO Configuration
    PE9     ------> SDADC2_AIN7P
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_9);
  }
}

void SystemClock_Config(){
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_ADC1|RCC_PERIPHCLK_SDADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
  PeriphClkInit.SdadcClockSelection = RCC_SDADCSYSCLK_DIV8;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PCLK2_DIV2;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG1);
  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG2);
}