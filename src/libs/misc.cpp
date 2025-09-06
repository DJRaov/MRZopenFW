#include "misc.h"

void errorHandler(uint8_t err) {  //basic error handler
  dataUpdate->pause();
  bitTXtimer->pause();
  while (true) {
    okLEDtimer->pause();
    okLEDtimer->setPWM(2, okLED, 3, 50);
    GPIOA->BSRR=(1U << (4 + 16U)); //turn okLED off
    HAL_Delay(100);
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
        extUART.println("ERR: One of the SDADCs failed to calibrate within specified time frame. Cannot continue.");
        break;

      case 8:
        extUART.println("ERR: Failed to initialize one of the SDADCs. Cannot continue.");
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
