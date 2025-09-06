#include "gnss.h"
#include "libs/misc.h"  // For errorHandler

void gnssCheck() { //GNSS presence check
  bool gpsAlive = 0;
  bool warmGNSS = 0;
  uint32_t startTime = HAL_GetTick();
  uint32_t warmBootTime = startTime + 3000;
  uint32_t timeoutTime = startTime + 6000;
  
  while (HAL_GetTick() < timeoutTime && !gpsAlive) {
      if (HAL_GetTick() >= warmBootTime && !warmGNSS) {
          #ifdef debug
          extUART.println("GNSS not found at 9600 baud. Warm boot? Trying 115200 baud...");
          #endif
          gpsUART.end();
          HAL_Delay(100);
          gpsUART.begin(115200);
          warmGNSS = 1;
      }
      if (gpsUART.available()) {
          if (gpsUART.read() == 0x24) {
              gpsAlive = 1;
              #ifdef debug
              if (!warmGNSS) {
                  extUART.println("Cold GNSS found. Sending init commands...");
              } else {
                  extUART.println("Hot GNSS found. Skipping init.");
              }
              #endif
              
              if (!warmGNSS) {
                  MicroNMEA::sendSentence(gpsUART, "$PMTK301,2"); 
                  HAL_Delay(50);
                  MicroNMEA::sendSentence(gpsUART, "$PMTK313,1"); //enable SBAS
                  HAL_Delay(50);
                  MicroNMEA::sendSentence(gpsUART, "$PMTK352,0"); //look for QZSS
                  HAL_Delay(50);
                  MicroNMEA::sendSentence(gpsUART, "$PMTK353,1,1,1,0,0"); //GPS+GLONASS+Galileo
                  HAL_Delay(50);
                  MicroNMEA::sendSentence(gpsUART, "$PMTK314,0,1,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"); // GGA every fix, RMC every 5 fixes
                  HAL_Delay(50);
                  MicroNMEA::sendSentence(gpsUART, "$PMTK251,115200"); //switch into maximum overdrive
                  gpsUART.flush();
                  gpsUART.end();
                  HAL_Delay(300);
                  gpsUART.begin(115200);
              }
              break;
          }
      }
      HAL_Delay(10); // Small delay to prevent tight polling
  }
  if (!gpsAlive) {
      errorHandler(2);
  }
}

void parseGNSSframe() {  //GNSS frame parser
  while (gpsUART.available()) {
    char c = gpsUART.read();
    if (nmea.process(c)) {
#ifdef debugGNSS
      int gpsAlt = 0;
      long alt;
      if (nmea.getAltitude(alt)) {
        gpsAlt = alt / 1000;
      } else {
        gpsAlt = -1;
      }
      extUART.println("MSGID: " + String(nmea.getMessageID()) + " | " + String(nmea.getNumSatellites()) + " sats | " + String(nmea.getHour()) + ":" + String(nmea.getMinute()) + ":" + String(nmea.getSecond()) + " | " + String(float(nmea.getLatitude() / 1000000), 4) + "," + String(float(nmea.getLongitude() / 1000000), 4) + " " + String(gpsAlt) + "m | " + String(nmea.getSpeed() / 1000) + "km/h | " + nmea.getSentence());
#endif
    }
  }
}
