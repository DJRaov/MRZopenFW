# MRZopenFW

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Platform](https://img.shields.io/badge/Platform-STM32F373-green.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32f373.html)

An open-source firmware for the MRZ-N1 radiosonde series manufactured by Radiy.

## Features

- **GNSS cross-interoperbility**: Works with both U-Blox and SIM68 revision sondes
- **Multiple Modulation Modes**: FSK, GFSK, ASK, and OOK support
- **Multiple protocol support**: Supports APRS (soon) and Horus v2 (soon), alongside a custom protocol
- **Debug Interface**: UART-based debugging and monitoring capabilities

## Supported sondes
- MRZ-N1 (SIM68)

NOTE: Last supported serial is ~406000 (manufactured around mid-2024) - newer sondes have an incompatible Artery MCU
- MRZ-N1 (U-Blox)

NOTE: Phased out since mid-2023, last serials seem to be around 305000. Untested, SIM68-specific cmds may cause unexpected behavior

## Dependencies
- STM32duino core
- [MicroNMEA](https://github.com/stevemarple/MicroNMEA)

## Configuration

### Key Parameters
- **Transmission Frequency**: Default 404.5MHz (stock works between 400-420MHz)
- **Modulation**: FSK/GFSK/ASK/OOK selectable (NOTE: GFSK requires a HW mod - refer to `mods/readme`)

### Debug Options
Enable debug output by uncommenting:
```cpp
#define debug        // General debug information
#define debugGNSS    // GNSS-specific debug output
```

## Hardware Modifications

Some advanced features may require hardware modifications. Refer to the `mods/` directory for:
- GFSK modulation hardware modifications (soon)
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

**Acknowledgments**:
- MicroNMEA library contributors

## Disclaimer

This firmware is provided "as is", without warranty.
