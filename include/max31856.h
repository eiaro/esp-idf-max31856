/*
 * SPDX-FileCopyrightText: 2025 Ronny Eia <3652665+eiaro@users.noreply.github.com>
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include "esp_err.h"
#include "driver/spi_master.h"
#include <stdint.h>
#include <stdbool.h>

// thermocouple types
#define MAX31856_TC_TYPE_B 0b0000
#define MAX31856_TC_TYPE_E 0b0001
#define MAX31856_TC_TYPE_J 0b0010
#define MAX31856_TC_TYPE_K 0b0011
#define MAX31856_TC_TYPE_N 0b0100
#define MAX31856_TC_TYPE_R 0b0101
#define MAX31856_TC_TYPE_S 0b0110
#define MAX31856_TC_TYPE_T 0b0111

// Averaging modes
#define MAX31856_AVERAGE_1 0b000
#define MAX31856_AVERAGE_2 0b001
#define MAX31856_AVERAGE_4 0b010
#define MAX31856_AVERAGE_8 0b011
#define MAX31856_AVERAGE_16 0b100

// Open-Circuit detection modes
#define MAX31856_OC_DETECT_OFF 0b00
#define MAX31856_OC_DETECT_1 0b01
#define MAX31856_OC_DETECT_2 0b10
#define MAX31856_OC_DETECT_3 0b11

typedef struct max31856_dev {
    spi_device_handle_t spi_dev;
    uint8_t thermocouple_type;
    bool filter_50hz;
    uint8_t averaging;
    bool use_cold_junction; // If true, the cold junction compensation is enabled
} max31856_dev_t; 


esp_err_t max31856_init(max31856_dev_t *data);
esp_err_t max31856_read_thermocouple(max31856_dev_t *data, float *temperature);
esp_err_t max31856_read_cold_junction(max31856_dev_t *data, float *temperature);
esp_err_t max31856_read_fault_status(max31856_dev_t *data, uint8_t *status);