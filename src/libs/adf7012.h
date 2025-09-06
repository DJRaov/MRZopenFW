#ifndef ADF7012_H
#define ADF7012_H

#include <Arduino.h>
#include <stdint.h>
#include "buildconfig.h"
#include "src/globals.h"
#include "src/libs/misc.h"

extern uint8_t  rCountDivRatio;  // e.g., defined and assigned in MRZopenFW.ino
extern int      modDev;          // deviation/steps expected by reg2 encoding

void initTX(void);
void lockVCO(void);
void sendADFregister(int regnum);

#endif
