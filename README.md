# MRZopenFW

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Version](https://img.shields.io/badge/Version-0.069a-orange.svg)](https://github.com/DJRaov/MRZopenFW)
[![Platform](https://img.shields.io/badge/Platform-STM32F373-green.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32f373.html)

An open-source firmware for the MRZ-N1 radiosonde series manufactured by Radiy.

## Firmware features
- (almost) **Full Cross-Compatibility**: Works with (almost) all revisions of the MRZ-N1 (except revision v4, code rebase coming soon[tm])
- **Wide TX protocol support**
  * [Horus Binary v2 (4FSK)](https://github.com/projecthorus/horusdemodlib/wiki)
    * Very low SNR requirements, allows for 9 bytes of custom data
    * Multi-QRG operation supported (soon)
    * NOTE: uses 250hz shift due to HW constraints
  * APRS (soon)
    * Standard AFSK@1200bd modulation, timer-switched
    * add text here
  * RTTY (soon)
    * UKHAS-compliant formatting, 7N2 (configurable)
    * Customizable baud rates
    * Tone spacing quantizable down to 125hz (250hz default)
  * Morse code/CW (soon)
    * Customizable WPM
    * UKHAS-compliant formatting
- **Pre-flight check** via onboard UART and LED

## Supported sonde revisions
- MRZ-N1 (v.2)
  * NOTE: Phased out since mid-2023, last serials seem to be around 305000. Untested, SIM68-specific cmds may cause unexpected behavior
- MRZ-N1-SIM68 (v.3)
  * NOTE: Last supported serial is ~406000 (manufactured around mid-2024)
- MRZ-N1-ART68 (v.4) (soon)

## Dependencies
- STM32duino core
- [MicroNMEA library](https://github.com/stevemarple/MicroNMEA)

**Included Libraries** (in `src/libs`):
- [Project Horus FEC encoder](https://github.com/projecthorus/horusdemodlib) (src/libs/horusv2)
- [MicroNMEA](https://github.com/stevemarple/MicroNMEA) (src/libs/micronmea)

## Configuration

### Protocol Selection
Protocol selection is done via compile-time defines:
```cpp
#define modHorus     // Default - currently implemented
#define modAPRS      // Planned
#define modRTTY      // Planned
```

### Debug Options
Enable debug output by uncommenting:
```cpp
#define debug           // Verbose startup and status messages
#define debugHorus      // Horus v2 protocol debug information
#define debugSensors    // Sensor reading debug output
#define debugGNSS       // GNSS-specific debug output
```

### Sonde ID and transmit frequency
Sonde ID and transmit frequency are defined in the main .ino file right after all the includes:
```cpp
uint16_t payloadID = 0;      //default, change before launch!
uint32_t txFreq = 437600000; //in Hz
```

## Compilation
**BIG NOTE: You need to add the STM32F373 definition in boards.txt yourself before attempting to compile!**

just press the upload sketch button lol

it is as shrimple as that

## Hardware Modifications
Some more advanced features require hardware modifications. Refer to the `mods/` directory for:
- Additional sensor integration
- VCO inductor trimming
- Reducing power draw
(all coming soon)

## Roadmap
- u-blox GNSS support (Mid/Late September 2025)
- Full rebase to STM32 HAL (October/November 2025)
- Improved stock sensor boom accuracy (Early 2026)
- MRZ-N1-ART68 support (post-rebase, Early 2026)
- Additional sensor support (post-rebase, Early 2026)
- APRS, RTTY and CW support (post-rebase, Early 2026)
- Multi-QRG support (post-rebase, Early 2026)

## License
This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.

## Credits
**Author**: Raov (2025)

**Acknowledgments**:
- Project Horus team (David Rowe) for the FEC library and protocol specification
- Steve Marple for the MicroNMEA library
- STM32duino community for platform support