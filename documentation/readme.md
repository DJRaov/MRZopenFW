# MRZ‑N1 Radiosonde Hardware

## Overview & Variants

- MRZ-N1 (v.2)
  * STM32F373C8T6 MCU
  * ADF7012B
  * U-blox NEO-M8N GNSS
  * Unknown PA
- MRZ-N1-SIM68 (v.3)
  * STM32F373C8T6 MCU
  * ADF7012B
  * SIMCom SIM68M GNSS
  * Unknown PA
- MRZ-N1-ART68 (v.4)
  * AT32F413CBT7 MCU
  * ADF7012B
  * SIMCom SIM68M GNSS
  * Unknown PA

## System Block (text)

- Power: 3×AA lithium -> LM2733Y Boost converter (8V out) -> 78L05 -> PA
                                                          -> LM1117-3.3 -> 3V3 rail
- RF: ADF7012B -> PA (top‑mark “49N”, 6‑pin) on 5V -> LC filter -> 1/4‑λ whip antenna.
- Sensors: NTC bead and SHT31‑ARP analog humidity sensor into SDADC.

[TODO: Add a drawn block diagram once close‑ups confirm exact power routing and references.]


## MCUs
### STM32F373 pinout:
* PA1: ADF7012B MuxOUT
* PA3: ADF7012B TXData
* PA4: Internal LED
* PA9: External UART TX
* PA10: External UART RX
* PA13: SWDIO
* PA14: SWCLK
* PB0: Vbat ADC Input
* PB3: GNSS UART TX
* PB4: GNSS UART RX
* PB8: ADF7012B ChipEnable
* PB9: ADF7012B LoadEnable
* PC13: ADF7012B CFGCLK
* PC14: ADF7012B CFGDATA
* PE8: Temperature SDADC
* PE9: Humidity SDADC
### AT32F413 pinout:
[TODO]

- SWD: Pads near MCU; SWD left open (stock firmware locked, but SWD access available).
- MCU differences STM32 vs AT32: minor placement shifts only (status LED, ADF ref XO). No strap differences observed.

## GNSS Daughterboards

Common mechanical:
- Daughterboard: 59×59 mm, 2‑layer. Patch antenna on the outside‑facing side; module on inside‑facing side.
- Interconnect to main via 20×1 header at 90°; most pins are ground; power and UART present.

u‑blox (mrz‑n1):
- Module: NEO‑M8N.
- Signals routed: 3V3, GND, TX, RX. (likely more)

SIMCom (mrz‑n1‑sim68, AT32 variant):
- Module: SIM68M.
- Signals routed: 3V3, GND, TX, RX.

## RF Chain

- Synthesizer: ADF7012B with 16 MHz reference (XO; likely TCXO given observed stability).
  - Signals used: RFOUT and MuxOut into external chain.
  - CLKOUT is wired to MCU clock input but not used by firmware.
- PA: 6‑pin device, top‑mark “49N”, supplied from 5V LDO (78L05).
- Output filter: LC filter after PA; architecture not yet characterized.
- Antenna: 1/4‑λ wire soldered to a pad; feed assumed 50 Ω after matching.

Test points:
- ADF VCO_IN test point present near ADF.

## Stock boom sensors

- Temperature: NTC bead on a boom via ribbon cable, pulled up to 3.3 V with a 10 kΩ resistor; capacitor in parallel across the NTC network; node goes directly into MCU ADC.
  * [TODO: Find exact bead, eventually]
- Humidity: SHT31‑ARP (analog) routed directly into MCU through an inductor (simple input choke).

## Pinouts

### Boom connector
The pinout of the sensor boom connector is as follows:
```
______________________|           |______________________
|                                                       |
|   1           2           3           4           5   |
|   |           |           |           |               |
|   1           2           3           4           6   |
|_______________________________________________________|

(pay attention to the notch in the connector)
```
* Pin 1 - Temp IN (PE8)
* Pin 2 - 3V3
* Pin 3 - Humidity IN (PE9)
* Pin 4 - GND
* Pin 5 - UART1 TXD (PA9)
* Pin 6 - UART1 RXD (PA10)

### Programming connector
The pinout for the programming connector is as follows:
```
-------
|  1  | ----UART1 TXD (PA9)
|     |
|  2  | ----UART1 RXD (PA10)
|     |
|  3  | ----3V3
|     |
|  4  | ----GND
|     |
|  5  | ----NRST
|     |
|  6  | ----BOOT0
|     |
|  7  | ----Unused
|     |
|  8  | ----Unused
|     |
|  9  | ----GND
|     |
|  10 | ----GND
|     |
|  11 | ----GND
-------
```

Main ↔ Daughter interconnect: 20×1, 2.54 mm header, 90° angle
- Majority of pins are GND.
- Known signals:
  - Pin 1: TX
  - Pin 2: RX
  - Pin 14: VCC
  - Others: GND
- [TODO: Confirm orientation marker for pin 1 and provide a full pin map photo/diagram.]

SWD pads (near MCU):
- Present; order and spacing: [TODO photo and map].
- NRST also available on edge connector.

ADF test point:
- VCO_IN TP near ADF.

## Mechanical

- Mainboard: 67 × 59 mm, 2‑layer FR‑4; thickness likely 2.0 mm (to be confirmed).
- Daughterboard: 59 × 59 mm, 2‑layer FR‑4.
- Stack: 90° assembly via bent 2.54 mm headers (Dupont‑style) soldered between boards.
- Grounding: extensive ground via the 20×1 header, most positions tied to GND.
- Antenna: wire whip exits from mainboard pad; GNSS patch faces outward on daughterboard.