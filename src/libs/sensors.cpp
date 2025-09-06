#include "sensors.h"
#include "buildconfig.h"  // For VbatSense

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
  #ifdef debugSensors
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
