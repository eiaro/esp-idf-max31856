/*
 * SPDX-FileCopyrightText: 2025 Ronny Eia <3652665+eiaro@users.noreply.github.com>
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// thermocouple types
typedef enum {
    MAX31856_TC_TYPE_B = 0b0000, // Type B thermocouple
    MAX31856_TC_TYPE_E = 0b0001, // Type E thermocouple
    MAX31856_TC_TYPE_J = 0b0010, // Type J thermocouple
    MAX31856_TC_TYPE_K = 0b0011, // Type K thermocouple
    MAX31856_TC_TYPE_N = 0b0100, // Type N thermocouple
    MAX31856_TC_TYPE_R = 0b0101, // Type R thermocouple
    MAX31856_TC_TYPE_S = 0b0110, // Type S thermocouple
    MAX31856_TC_TYPE_T = 0b0111, // Type T thermocouple
} max31856_thermocouple_type_t;

// Averaging modes
typedef enum {
    MAX31856_AVG_1 = 0b000,  // No averaging
    MAX31856_AVG_2 = 0b001,  // Average 2 samples
    MAX31856_AVG_4 = 0b010,  // Average 4 samples
    MAX31856_AVG_8 = 0b011,  // Average 8 samples
    MAX31856_AVG_16 = 0b100, // Average 16 samples
} max31856_average_t;

// Open-Circuit detection modes
typedef enum {
    MAX31856_OC_DETECT_OFF = 0b00, // Open-Circuit detection disabled
    MAX31856_OC_DETECT_1   = 0b01, // Open-Circuit detection mode 1
    MAX31856_OC_DETECT_2   = 0b10, // Open-Circuit detection mode 2
    MAX31856_OC_DETECT_3   = 0b11  // Open-Circuit detection mode 3
} max31856_oc_detection_t;

/**
 * @brief Structure representing a MAX31856 device instance.
 *
 * This structure holds all relevant information and configuration
 * required to interface with the MAX31856 thermocouple-to-digital converter.
 * Populate and manage this structure when initializing and communicating
 * with the device.
 */
typedef struct max31856_dev
{
    spi_device_handle_t spi_dev;
    max31856_thermocouple_type_t thermocouple_type;
    bool filter_50hz;
    max31856_average_t averaging;
    max31856_oc_detection_t oc_detection;
    bool use_cold_junction; // If true, the cold junction compensation is enabled
} max31856_dev_t;

/**
 * @brief Initialize the MAX31856 device.
 *
 * This function initializes the MAX31856 device structure and configures the necessary hardware interfaces.
 *
 * @param[in,out] data Pointer to the MAX31856 device structure to initialize.
 * @return
 *      - ESP_OK on success
 *      - Appropriate error code from esp_err_t on failure
 */
esp_err_t max31856_init(max31856_dev_t *data);

/**
 * @brief Reads the thermocouple temperature from the MAX31856 device.
 *
 * This function communicates with the MAX31856 thermocouple-to-digital converter
 * to retrieve the current thermocouple temperature measurement.
 *
 * @param[in]  data         Pointer to the MAX31856 device structure.
 * @param[out] temperature  Pointer to a float variable where the measured temperature (in °C) will be stored.
 *
 * @return
 *     - ESP_OK on success
 *     - Appropriate esp_err_t error code otherwise
 */
esp_err_t max31856_read_thermocouple(max31856_dev_t *data, float *temperature);

/**
 * @brief Reads the cold junction temperature from the MAX31856 device.
 *
 * This function retrieves the current cold junction (reference junction) temperature
 * from the specified MAX31856 device and stores the result in the provided temperature pointer.
 *
 * @param[in]  data         Pointer to the MAX31856 device structure.
 * @param[out] temperature  Pointer to a float variable where the temperature (in degrees Celsius) will be stored.
 *
 * @return
 *     - ESP_OK on success
 *     - Appropriate esp_err_t error code otherwise
 */
esp_err_t max31856_read_cold_junction(max31856_dev_t *data, float *temperature);

/**
 * @brief Reads the fault status register from the MAX31856 device.
 *
 * This function retrieves the current fault status from the MAX31856 thermocouple-to-digital converter
 * and stores it in the provided status variable.
 *
 * @param[in]  data    Pointer to the MAX31856 device structure.
 * @param[out] status  Pointer to a variable where the fault status byte will be stored.
 *
 * @return
 *     - ESP_OK on success
 *     - Appropriate error code otherwise
 */
esp_err_t max31856_read_fault_status(max31856_dev_t *data, uint8_t *status);

#ifdef __cplusplus
}
#endif