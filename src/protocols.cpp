#include "protocols.h"
#include "buildconfig.h"  // If needed for any defines

void updateUKHASframe() { //TODO: UKHAS stuff
}

void txRTTY() { //TODO: rework, start using structs
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

void txAPRS() { //TODO: APRS
}

//Horus v2 stuff
#ifdef modHorus
uint16_t bitIndex = 0;
bool frameSent = 0;
void updateHorusFrame() {
  #ifdef debugHorus
  uint32_t startTime = micros();
  #endif

  static uint16_t frameCounter;
  long alt;
  long gpsAlt;

  GPIOA->BSRR=(1U << 4); //heartbeat
  tlmFrame.payloadID = payloadID;
  tlmFrame.seqNum = frameCounter;
  tlmFrame.hour = (nmea.getHour() > 24) ? 0 : nmea.getHour();
  tlmFrame.min = (nmea.getMinute() > 60) ? 0 : nmea.getMinute();
  tlmFrame.sec = (nmea.getSecond() > 60) ? 0 : nmea.getSecond();
  tlmFrame.lat = (nmea.getLatitude()/1000000.0f > 180) ? 0 : nmea.getLatitude()/1000000.0f;
  tlmFrame.lon = (nmea.getLongitude()/1000000.0f > 180) ? 0 : nmea.getLongitude()/1000000.0f;
  if (nmea.getAltitude(alt)) {
    gpsAlt = alt / 1000;
  } else {
    gpsAlt = -1;
  }
  tlmFrame.alt = gpsAlt;
  tlmFrame.satCount = nmea.getNumSatellites();
  tlmFrame.temp = int8_t(temp);
  tlmFrame.vbat = Vbat*50;
  tlmFrame.custom1 = 0x00; //empty
  tlmFrame.custom2 = 0x0699B5218A471E01; //placeholder, replace as you wish
  tlmFrame.crc = crc16_ccitt((uint8_t*)&tlmFrame, sizeof(TelemetryFrame) - sizeof(uint16_t));
  frameCounter++;
  memcpy(rawBuffer, &tlmFrame, sizeof(TelemetryFrame));
  encodedLength = horus_l2_encode_tx_packet(codedBuffer, rawBuffer, sizeof(TelemetryFrame));
  frameSent = 0;
  GPIOA->BSRR=(1U << (4 + 16U)); //heartbeat

  #ifdef debug
  extUART.print("Horus v2 frame refreshed!\n");
  #endif
  #ifdef debugHorus
  extUART.println("Raw length: " + String(sizeof(TelemetryFrame)));
  extUART.println("Encoded length: " + String(encodedLength));
  extUART.print("Frame: ");
  printStructHex(&codedBuffer, sizeof(codedBuffer), extUART);
  uint32_t duration = micros() - startTime;
  if (duration > 100) {
      extUART.println("ISR duration: " + String(duration) + "µs");
  }
  #endif
}
void txNext4FSKSymbol() { //hacky, but works surprisingly well
  // Symbol to modDev and outputBit lookup tables
  const uint8_t symbolModDev[4] = {3, 1, 1, 3};      // 00->3, 01->1, 10->1, 11->3
  const uint8_t symbolOutputBit[4] = {0, 0, 1, 1};   // 00->0, 01->0, 10->1, 11->1
  uint16_t totalBits = encodedLength * 8;
  if (bitIndex >= totalBits) {
    bitIndex = 0;
    frameSent = 1;
    return;
  }
  
  // Extract next 2 bits from encoded buffer (MSB first)
  uint8_t symbol = 0;
  for (int i = 0; i < 2; i++) {
    if (bitIndex < totalBits) {
      uint16_t byteIndex = bitIndex / 8;
      uint8_t bitInByte = 7 - (bitIndex % 8);
      uint8_t bit = (codedBuffer[byteIndex] >> bitInByte) & 0x01;
      symbol |= (bit << (1 - i));  // Build 2-bit symbol
      bitIndex++;
    }
  }
  
  // Get modDev and outputBit for this symbol
  uint8_t modDevSteps = symbolModDev[symbol];
  uint8_t outputBit = symbolOutputBit[symbol];

  modDev = modDevSteps;
  sendADFregister(2);
  digitalWrite(adfTXdata, outputBit ? HIGH : LOW); //change to HAL actuation (PA3)
}
uint16_t crc16_ccitt(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;  // Initial value for CCITT
    
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;  // CCITT polynomial
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}
#ifdef debugHorus
void printStructHex(const void* ptr, size_t len, HardwareSerial& serial) {
  const uint8_t* bytes = (const uint8_t*)ptr;
  for (size_t i = 0; i < len; i++) {
    if (bytes[i] < 0x10) serial.print('0');  // leading zero for single-digit
    serial.print(bytes[i], HEX);
  }
  serial.println();
}
#endif
#endif
