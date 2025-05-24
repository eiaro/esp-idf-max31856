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
   git clone https://github.com/eiaro/max31856.git components/max31856
   ```

2. Include the component in your project's CMakeLists.txt:
   ```cmake
   idf_component_register(
       # Your project details...
       REQUIRES max31856
   )
   ```

## Hardware Connection

| MAX31856 Pin | ESP32 Pin (Default) |
|--------------|---------------------|
| SCK          | GPIO18              |
| MISO         | GPIO19              |
| MOSI         | GPIO23              |
| CS           | GPIO5               |
| VCC          | 3.3V                |
| GND          | GND                 |

## Usage Example

```c
#include "max31856.h"
#include "driver/spi_master.h"

void app_main(void)
{
    // Configure SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = GPIO_NUM_19,
        .mosi_io_num = GPIO_NUM_23,
        .sclk_io_num = GPIO_NUM_18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    
    // Configure SPI device
    spi_device_interface_config_t devcfg = {
        .address_bits = 8,
        .mode = 1,
        .clock_speed_hz = 1000000,
        .spics_io_num = GPIO_NUM_5,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 1,
    };
    
    // Initialize SPI bus and device
    spi_device_handle_t spi;
    spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI_HOST, &devcfg, &spi);
    
    // Configure MAX31856 device
    max31856_dev_t max31856_dev = {
        .spi_dev = spi,
        .thermocouple_type = MAX31856_TC_TYPE_K,
        .averaging = MAX31856_AVG_4,
        .filter_50hz = true,
        .use_cold_junction = true
    };
    
    // Initialize MAX31856
    max31856_init(&max31856_dev);
    
    // Read temperatures
    float tc_temp, cj_temp;
    max31856_read_thermocouple(&max31856_dev, &tc_temp);
    max31856_read_cold_junction(&max31856_dev, &cj_temp);
    
    printf("Thermocouple: %.2f°C, Cold Junction: %.2f°C\n", tc_temp, cj_temp);
}
```

## Configuration Options

The component can be configured through menuconfig:

```bash
idf.py menuconfig
```

Navigate to "MAX31856 Configuration" to access:
- Debug logging enable/disable

## API Documentation

### Functions

- `esp_err_t max31856_init(max31856_dev_t *data)` - Initialize MAX31856 with provided configuration
- `esp_err_t max31856_read_thermocouple(max31856_dev_t *data, float *temperature)` - Read thermocouple temperature
- `esp_err_t max31856_read_cold_junction(max31856_dev_t *data, float *temperature)` - Read cold junction temperature
- `esp_err_t max31856_read_fault_status(max31856_dev_t *data, uint8_t *status)` - Read fault status register

### Types

```c
// Available thermocouple types
typedef enum {
    MAX31856_TC_TYPE_B = 0b0000,
    MAX31856_TC_TYPE_E = 0b0001,
    MAX31856_TC_TYPE_J = 0b0010,
    MAX31856_TC_TYPE_K = 0b0011,
    MAX31856_TC_TYPE_N = 0b0100,
    MAX31856_TC_TYPE_R = 0b0101,
    MAX31856_TC_TYPE_S = 0b0110,
    MAX31856_TC_TYPE_T = 0b0111,
} max31856_thermocouple_type_t;

// Averaging modes
typedef enum {
    MAX31856_AVG_1 = 0b000,
    MAX31856_AVG_2 = 0b001,
    MAX31856_AVG_4 = 0b010,
    MAX31856_AVG_8 = 0b011,
    MAX31856_AVG_16 = 0b100,
} max31856_average_t;

// Device configuration struct
typedef struct {
    spi_device_handle_t spi_dev;
    max31856_thermocouple_type_t thermocouple_type;
    max31856_average_t averaging;
    bool filter_50hz;
    bool use_cold_junction;
} max31856_dev_t;
```

## License

MIT License

## References

- [MAX31856 Datasheet](https://datasheets.maximintegrated.com/en/ds/MAX31856.pdf)
- [Analog Devices Product Page](https://www.analog.com/en/products/max31856.html)