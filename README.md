<!--
SPDX-FileCopyrightText: 2025 Ronny Eia <3652665+eiaro@users.noreply.github.com>

SPDX-License-Identifier: MIT
-->

# MAX31856 Thermocouple Interface for ESP-IDF

This component provides an ESP-IDF driver for the [MAX31856](https://www.analog.com/en/products/max31856.html) thermocouple-to-digital converter.

## Features

- Supports multiple thermocouple types (K, J, N, R, S, T, E, B)
- SPI communication interface
- Cold-junction compensation
- Configurable averaging (1, 2, 4, 8, 16 samples)
- 50/60Hz noise filtering options
- Fault detection and reporting

## Installation

### As ESP-IDF Component

1. Clone this repository into your project's components directory:
   ```sh
   cd your_project
   mkdir -p components
   git clone https://github.com/eiaro/esp-idf-max31856.git components/max31856
   ```

2. Include the component in your project's CMakeLists.txt:
   ```cmake
   idf_component_register(
       # Your project details...
       REQUIRES max31856
   )
   ```

## Usage Example

See the examples for C and C++ usage.

## Configuration Options

The component can be configured through menuconfig:

```bash
idf.py menuconfig
```

Navigate to "MAX31856 Configuration" to access:
- Debug logging enable/disable


## License

MIT License

## References

- [MAX31856 Datasheet](https://datasheets.maximintegrated.com/en/ds/MAX31856.pdf)
- [Analog Devices Product Page](https://www.analog.com/en/products/max31856.html)