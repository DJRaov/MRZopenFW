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
  * Morse code/CW
    * Customizable WPM
    * UKHAS-compliant formatting
- **Pre-flight check** via onboard UART

## Supported sonde revisions
- MRZ-N1 (SIM68)

NOTE: Last supported serial is ~406000 (manufactured around mid-2024) - newer sondes have an incompatible Artery MCU
- MRZ-N1 (U-Blox)

NOTE: Phased out since mid-2023, last serials seem to be around 305000. Untested, SIM68-specific cmds may cause unexpected behavior

## Dependencies
- STM32duino core
- [MicroNMEA library](https://github.com/stevemarple/MicroNMEA)

**Included Libraries** (in `src/` directory):
- Project Horus FEC library (GPL 3.0) - Forward error correction for Horus v2

## Configuration

### Protocol Selection (Future)
Protocol selection will be done via compile-time defines:
```cpp
// #define modHorus     // Default - currently implemented
// #define modAPRS      // Planned
// #define modRTTY      // Planned
```

### Debug Options
Enable debug output by uncommenting:
```cpp
#define debug           // Verbose startup and status messages
#define debugHorus      // Horus v2 protocol debug information
#define debugSensors    // Sensor reading debug output
#define debugGNSS       // GNSS-specific debug output
```

## Hardware Modifications

Some more advanced features require hardware modifications. Refer to the `mods/` directory for:
- Additional sensor integration guides (soon)
- Adding GFSK support
- Wiring up PPS
- Wiring up non-stock sensors

## Contributing

Contributions are welcome! Please feel free to:
- Report bugs and issues
- Suggest new features
- Submit pull requests
- Improve documentation

## License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.

## Credits

**Author**: Raov (2025)

**Acknowledgments**:
- Project Horus team (David Rowe) for the FEC library and protocol specification
- Steve Marple for the MicroNMEA library
- STM32duino community for platform support

## Disclaimer

This firmware is provided "as is", without warranty.
