# MRZopenFW

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Version](https://img.shields.io/badge/Version-0.069a-orange.svg)](https://github.com/DJRaov/MRZopenFW)
[![Platform](https://img.shields.io/badge/Platform-STM32F373-green.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32f373.html)

An open-source firmware for the MRZ-N1 radiosonde series manufactured by Radiy.

## Features

- **GNSS Cross-Compatibility**: Works with both U-Blox and SIM68 revision sondes
- **Standard Telemetry Protocols**: Horus v2 (default), APRS and UKHAS support
- **Protocol-Driven Modulation**: Automatic modulation selection based on chosen protocol
- **Debug Interface**: UART-based debugging and monitoring capabilities

## Supported sondes
- MRZ-N1 (SIM68)

NOTE: Last supported serial is ~406000 (manufactured around mid-2024) - newer sondes have an incompatible Artery MCU
- MRZ-N1 (U-Blox)

NOTE: Phased out since mid-2023, last serials seem to be around 305000. Untested, SIM68-specific cmds may cause unexpected behavior

## Dependencies
- STM32duino core

**Included Libraries** (in `src/` directory):
- MicroNMEA library (LGPL 2.1) - NMEA parsing
- Project Horus FEC library (GPL 3.0) - Forward error correction for Horus v2

## Configuration

### Protocol Selection (Future)
Protocol selection will be done via compile-time defines:
```cpp
// #define PROTOCOL_HORUS_V2  // Default - currently implemented
// #define PROTOCOL_APRS      // Planned
// #define PROTOCOL_UKHAS     // Planned
```

### Key Parameters
- **Transmission Frequency**: Default 404.5MHz (stock works between 400-420MHz)
- **Horus v2 Payload ID**: Set your assigned payload ID
  ```cpp
  uint16_t payloadID = 0; // 0=Test, use your assigned ID for flights
  ```

### Debug Options
Enable debug output by uncommenting:
```cpp
#define debugHorus      // Horus v2 protocol debug information
#define debugSensors    // Sensor reading debug output
#define debugGNSS       // GNSS-specific debug output
```

## Telemetry Protocols

### Horus v2 (Default)
- **Modulation**: 4FSK with forward error correction
- **Baud Rate**: 100 symbols/second
- **Configuration**: Set your payload ID in the firmware

### APRS (Planned)
- Standard APRS packet format
- Compatible with existing APRS infrastructure

### UKHAS (Planned)
- ASCII telemetry format
- Compatible with UKHAS tracking systems

## Hardware Modifications

Some advanced features may require hardware modifications. Refer to the `mods/` directory for:
- Additional sensor integration guides (soon)

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
