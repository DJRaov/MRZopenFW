# MRZ‑N1 Radiosonde Hardware

This documents the MRZ‑N1 radiosonde hardware only; external components and vendor modules are referenced by part family where needed.

Images:
- ![Mainboard (MRZ‑N1 v3)](rev3/img/mrz-n1-mainboard.jpg)
- ![Mainboard Annotated (MRZ‑N1 v3)](rev3/img/mrz-n1-mainboard_annotated.jpg)

## 1) Overview & Variants

- Standard: MRZ-N1 (STM32 + u‑blox NEO‑M8N GNSS)
- Variant: MRZ-N1-SIM68 (STM32 + SIMCom SIM68M GNSS)
- Variant: MRZ-N1-ART68 (AT32F413CBT7 + SIM68M)

Similarities:
- ADF7012B present on all; same general RF configuration across variants.
- MCUs run on internal RC (no HSE/LSE fitted). ADF has a 16 MHz ref (likely TCXO).
- GNSS PPS not used, UART is 3V3.

Differences:
- 

Variant summary:

- MRZ-N1
  - MCU: STM32F373C8T6
  - GNSS: u‑blox NEO‑M8N
- MRZ-N1-SIM68
  - MCU: STM32F373C8T6
  - GNSS: SIMCom SIM68M
- AMRZ-N1-ART68
  - MCU: AT32F413CBT7
  - GNSS: SIMCom SIM68M

## 2) System Block (text)

- Power: 3×AA lithium -> LM2733Y Boost converter (8v out) -> 78L05 -> PA
                                                          -> LM1117-3.3 -> 3v3 rail.
- RF: ADF7012B -> PA (top‑mark “49N”, 6‑pin) on 5 V -> LC filter -> 1/4‑λ whip antenna.
- Sensors: NTC bead and SHT31‑ARP analog humidity sensor into SDADC.

[TODO: Add a drawn block diagram once close‑ups confirm exact power routing and references.]


## 3) MCU & Pin Map

- MCUs: STM32F373C8T6 or AT32F413CBT7.
- Clocking: No HSE/LSE fitted. Internal RC used. ADF CLKOUT wired to MCU clock input but unused in practice.
- BOOT/RST: Edge connector exposes NRST and BOOT0. BOOT0/NRST on‑board pulls: [TODO: values].
- SWD: Pads near MCU; SWD left open (stock firmware locked, but SWD access available).
  - [TODO: Document pad order for SWDIO/SWCLK/GND/3V3/NRST with a photo callout.]
- UART to GNSS: 3V3 UART (no level shifters). Pins as per firmware.
- MCU differences STM32 vs AT32: minor placement shifts only (status LED, ADF ref XO). No strap differences observed.

[TODO: Extract exact pin mapping from firmware (UARTs, ADF control pins, ADC channels).]

## 4) GNSS Daughterboards

Common mechanical:
- Daughterboard: 59×59 mm, 2‑layer. Patch antenna on the outside‑facing side; module on inside‑facing side.
- Interconnect to main via 20×1 header at 90°; most pins are ground; power and UART present.

u‑blox (mrz‑n1):
- Module: NEO‑M8N.
- Signals routed: 3V3, GND, TX, RX. PPS not connected.

SIMCom (mrz‑n1‑sim68, AT32 variant):
- Module: SIM68M.
- Signals routed: 3V3, GND, TX, RX. PPS not connected.

[TODO: Add close‑ups (both sides) of each daughterboard; confirm mounting hole positions if present.]

## 5) RF Chain

- Synthesizer: ADF7012B with 16 MHz reference (XO; likely TCXO given observed stability).
  - Signals used: RFOUT and MuxOut into external chain.
  - CLKOUT is wired to MCU clock input but not used by firmware.
- PA: 6‑pin device, top‑mark “49N”, supplied from dedicated 5 V_LDO (78L05).
  - PA level set to 30 (per sniffed config). Exact PA model TBD.
- Output filter: LC filter after PA; architecture not yet characterized.
- Antenna: 1/4‑λ wire soldered to a pad; feed assumed 50 Ω after matching.
- Frequency error observation: +700 Hz (may be SDR reference offset rather than TX).

[TODO: Capture macro photos of the PA and filter; list L/C values where markings are visible. Identify PA via top‑mark if possible. Confirm if any SAW present.]

Test points:
- ADF VCO_IN test point present near ADF.
- Additional unlabeled TPs: [TODO: map if/when identified].

## 6) Sensors & ADC/SDADC

- Temperature: NTC bead on a boom via ribbon cable, pulled up to 3.3 V with a 10 kΩ resistor; capacitor in parallel across the NTC network; node goes directly into MCU ADC.
  - [TODO: Document exact capacitor value and any series resistors if present.]
- Humidity: SHT31‑ARP (analog) routed directly into MCU through an inductor (simple input choke).
  - No op‑amp/filtering observed beyond the inductor.
- Calibration: SDADC/ADC calibration method unknown (stock firmware locked). [TODO: characterize in firmware or empirical calibration method later.]
- Battery sense: VBAT via 2 kΩ series into ADC (no divider observed). See Power Tree note.

[TODO: Map these nets to exact MCU ADC/SDADC channels and include a small schematic fragment.]

## 7) Connectors & Test Points

Edge connector (6‑pin), viewed as in photos, pin 1 on the left:
- Pin 1: TX
- Pin 2: RX
- Pin 3: MCU 3V3
- Pin 4: GND
- Pin 5: NRST
- Pin 6: BOOT0

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

## 8) Mechanical

- Mainboard: 67 × 59 mm, 2‑layer FR‑4; thickness likely 2.0 mm (to be confirmed).
- Daughterboard: 59 × 59 mm, 2‑layer FR‑4.
- Stack: 90° assembly via bent 2.54 mm headers (Dupont‑style) soldered between boards.
- Grounding: extensive ground via the 20×1 header, most positions tied to GND.
- Antenna: wire whip exits from mainboard pad; GNSS patch faces outward on daughterboard.

[TODO: Confirm PCB thickness with calipers; note standoff/stack height; add mounting hole coordinates if any.]

## 9) Bring‑Up & Test Notes

Electrical checks (suggested):
- Verify presence of ~8 V node at SOT‑223 output and 5 V at 78L05 during TX (PA enabled).
- Confirm 3.3 V rail stability with GNSS active.
- SWD connectivity: read MCU IDCODE; confirm RDP state.
- GNSS UART: confirm RX/TX activity (3V3) at power‑up (module‑specific default baud; typically 9600 or 115200 bps). [TODO]
- ADF lock: observe MuxOut behavior during tuning; verify VCO_IN TP range.
- RF: basic sanity via SDR; note observed frequency offset relative to known reference.

Firmware tie‑ins:
- ADF PA level currently set to 30 (per observed config).
- MCU clock source is internal RC; ADF CLKOUT not used.
- Protocols handled in firmware; not detailed here.

## 10) Known Deltas & Open Items

- AT32 variant: status LED and ADF ref XO placement moved vs STM32 build; otherwise equivalent routes.
- Power path: identify SOT‑223 “2SDF NO5B” function and surrounding parts (inductor/diode values).
- 3.3 V regulator identity and consumers.
- Full 20×1 header pin map and orientation.
- SWD pad order; BOOT0/NRST pull values.
- RF PA exact part ID and output filter values.
- ADC/SDADC channel mapping and calibration details.
- Current consumption in RX/idle/TX and GNSS lock states.