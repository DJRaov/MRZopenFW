#include "adf7012.h"

//ADF7012B initialization
//reg0
int fCountOffset = 0;      //F-Counter Offset [-1024 - 1023]
uint8_t rCountDivRatio = 4;  //values over 12 seem to upset the VCO
bool xtalDoubler = 0;        //doubles XO, not needed
bool crystalOSC = 0;         //keep disabled unless you want err3 to occur
uint8_t clkOutDivRatio = 8;  //[0 - 15, unused]
uint8_t vcoAdjust = 0;       //[0 - 4]
uint8_t outputDiv = 0;       //[0 - 3, even only]

//reg1
uint16_t modDivRatio = 0;         //[0 - 4095]
uint8_t NcountDivRatio = 0;  //[0 - 255]
bool prescaler = 0;          //[4/5(0) or 8/9(1)]

//reg2
uint8_t modulation = 0;    //[FSK(0), GFSK(1)] (GFSK needs HW mod, refer to mods/readme)
bool gaussOOK = 0;         //Needs HW mod, refer to mods/readme
uint8_t paLevel = 3;       //[0 - 63]
int modDev = 3;            //[0 - 511] (500hz/step with rCountDivRatio=2; Divider factor [0-127] if GFSK selected)
uint8_t gfskModCtl = 0;    //[0 - 7] (GFSK only)
uint8_t indexCounter = 0;  //[16,32,64,128] (GFSK only)

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

void sendADFregister(int regNum) {
  int8_t i = 0;
  uint32_t frame = 0;

  uint32_t cfgFrameR0 =
    (0) | ((uint32_t)(fCountOffset & 0x7FF) << 2) | ((uint32_t)(rCountDivRatio & 0xF) << 13) | ((uint32_t)(xtalDoubler & 0x1) << 17) | ((uint32_t)(!crystalOSC & 0x1) << 18) | ((uint32_t)(clkOutDivRatio & 0xF) << 19) | ((uint32_t)(vcoAdjust & 0x3) << 23) | ((uint32_t)(outputDiv & 0x3) << 25);
  uint32_t cfgFrameR1 =
    (1) | ((uint32_t)(modDivRatio & 0xFFF) << 2) | ((uint32_t)(NcountDivRatio & 0xFF) << 14) | ((uint32_t)(prescaler & 0x1) << 22);
  uint32_t cfgFrameR2 =
    (2) | ((uint32_t)(modulation & 0x3) << 2) | ((uint32_t)(0 & 0x1) << 4) | ((uint32_t)(paLevel & 0x3F) << 5) | ((uint32_t)(modDev & 0x1FF) << 11) | ((uint32_t)(gfskModCtl & 0x7) << 20) | ((uint32_t)(indexCounter & 0x3) << 23);
  uint32_t cfgFrameR3 =
    (3) | ((uint32_t)(PLLenable & 0x1) << 2) | ((uint32_t)(PAenable & 0x1) << 3) | ((uint32_t)(clkOutEnable & 0x1) << 4) | ((uint32_t)(dataInvert & 0x1) << 5) | ((uint32_t)(chargePumpI & 0x3) << 6) | ((uint32_t)(bleedUp & 0x1) << 8) | ((uint32_t)(bleedDown & 0xF) << 9) | ((uint32_t)(vcoDisable & 0x1) << 10) | ((uint32_t)(muxOut & 0xF) << 11) | ((uint32_t)(ldPrecision & 0x1F) << 15) | ((uint32_t)(vcoBiasI & 0xF) << 16) | ((uint32_t)(paBias & 0xF) << 20);

  GPIOB->BSRR = (1U << 9);              //set adfLoadEN high
  GPIOC->BSRR = (1U << 14+16U);         //set adfCfgData low
  GPIOC->BSRR = (1U << 13+16U);         //set adfCfgClk low
  GPIOB->BSRR = (1U << 9+16U);          //set adfLoadEN low

  switch (regNum) {
    case 0: frame = cfgFrameR0; break;
    case 1: frame = cfgFrameR1; break;
    case 2: frame = cfgFrameR2; break;
    case 3: frame = cfgFrameR3; break;
  }

  if (regNum == 1) {
    for (i = 23; i >= 0; i--) {
      if ((frame & (uint32_t)(1UL << i)) >> i) {
        GPIOC->BSRR = (1U << 14);       //set adfCfgData high
      } else {
        GPIOC->BSRR = (1U << 14+16U);   //set adfCfgData low
      }
      GPIOC->BSRR = (1U << 13);         //clock the clock
      GPIOC->BSRR = (1U << 13+16U);
    }
  } else {
    for (i = 31; i >= 0; i--) {
      if ((frame & (uint32_t)(1UL << i)) >> i) {
        GPIOC->BSRR = (1U << 14);       //set adfCfgData high
      } else {
        GPIOC->BSRR = (1U << 14+16U);   //set adfCfgData low
      }
      GPIOC->BSRR = (1U << 13);         //clock the clock
      GPIOC->BSRR = (1U << 13+16U);     //clock the clock
    }
  }
  GPIOB->BSRR = (1U << 9);              //set adfLoadEN high
}
void initTX() {  //ADF7012B initialization routine
  //frequency calculation
  uint32_t f_pfd = 16384000 / rCountDivRatio;
  float ratio = (float)txFreq / (float)f_pfd;
  float rest = ratio - (float)NcountDivRatio;

  NcountDivRatio = (unsigned int)(txFreq / f_pfd);
  modDivRatio = (uint32_t)(rest * 4096);

  //check for ADF presence
  digitalWrite(adfChipEN, 1);
  digitalWrite(adfLoadEN, 1);
  HAL_Delay(5);
  digitalWrite(adfLoadEN, 0);
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

void lockVCO() {   //VCO lock algo (yoinked straight from PecanPico)
  muxOut = 0b101;  //set analog lock detect
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
