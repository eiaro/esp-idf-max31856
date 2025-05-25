/*
 * SPDX-FileCopyrightText: 2025 Ronny Eia <3652665+eiaro@users.noreply.github.com>
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once
#include "max31856.h"
#include "esp_err.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Creates a contiguous bitmask spanning from bit position 'l' to 'h' (inclusive)
 *
 * This macro generates a value where bits in positions l through h are set to 1,
 * and all other bits are set to 0. It's commonly used for masking specific bit fields
 * within registers.
 *
 * @param h The high bit position (most significant bit of the mask)
 * @param l The low bit position (least significant bit of the mask)
 * @return A value with bits set from position l to position h
 *
 * Example:
 *   GENMASK(3, 0) produces 0b1111 (0xF)
 *   GENMASK(7, 4) produces 0b11110000 (0xF0)
 */
#define GENMASK(h, l) (((1UL << ((h) - (l) + 1)) - 1) << (l))

#define MAX31856_RD_WR_MASK BIT(7)              // Read/Write mask for SPI transactions
#define MAX31856_CR1_TCTYPE_MASK GENMASK(3, 0)  // Thermocouple type mask
#define MAX31856_CR1_AVERAGE_MASK GENMASK(6, 4) // Averaging mode mask
#define MAX31856_CR1_AVERAGE_SHIFT 4            // Shift for averaging mode bits
#define MAX31856_CR0_OC_MASK GENMASK(5, 4)      // Open-Circuit detection mode mask
#define MAX31856_CR0_OC_SHIFT 4                 // Shift for Open-Circuit detection mode bits
#define MAX31856_CR0_AUTOCONVERT BIT(7)         // Auto Conversion Mode bit
#define MAX31856_CR0_1SHOT BIT(6)               // One-Shot Mode bit
#define MAX31856_CR0_CJ BIT(3)                  // Cold Junction Compensation bit
#define MAX31856_CR0_FAULT BIT(2)               // Fault Output bit

// registers
#define MAX31856_CR0_REG 0x00    // Configuration Register 0
#define MAX31856_CR1_REG 0x01    // Configuration Register 1
#define MAX31856_MASK_REG 0x02   // Fault Mask Register
#define MAX31856_CJHF_REG 0x03   // Cold Junction High Threshold Register
#define MAX31856_CJLF_REG 0x04   // Cold Junction Low Threshold Register
#define MAX31856_LTHFTH_REG 0x05 // Linearized Temperature High Fault Threshold MSB Register
#define MAX31856_LTHFTL_REG 0x06 // Linearized Temperature High Fault Threshold LSB Register
#define MAX31856_LTLFTH_REG 0x07 // Linearized Temperature Low Fault Threshold MSB Register
#define MAX31856_LTLFTL_REG 0x08 // Linearized Temperature Low Fault Threshold LSB Register
#define MAX31856_CJTO_REG 0x09   // Cold-Junction Temperature Offset Register
#define MAX31856_CJTH_REG 0x0A   // Cold Junction Temperature Register, MSB
#define MAX31856_CJTL_REG 0x0B   // Cold Junction Temperature Register, LSB
#define MAX31856_LTCBH_REG 0x0C  // Linearized TC Temperature, Byte 2
#define MAX31856_LTCBM_REG 0x0D  // Linearized TC Temperature, Byte 1
#define MAX31856_LTCBL_REG 0x0E  // Linearized TC Temperature, Byte 0
#define MAX31856_SR_REG 0x0F     // Fault Status Register

/**
 * @brief Write a value to a specific register address of the MAX31856 device.
 *
 * This function writes a single byte value to the specified register address
 * of the MAX31856 thermocouple-to-digital converter via SPI.
 *
 * @param[in] data   Pointer to the MAX31856 device structure.
 * @param[in] addr   Register address to write to.
 * @param[in] value  Value to write to the register.
 *
 * @return
 *     - ESP_OK on success
 *     - Appropriate esp_err_t error code otherwise
 */
esp_err_t max31856_write(max31856_dev_t *data, uint8_t addr, uint8_t value);

/**
 * @brief Read a value from a specific register address of the MAX31856 device.
 *
 * This function reads a single byte value from the specified register address
 * of the MAX31856 thermocouple-to-digital converter via SPI.
 *
 * @param[in] data   Pointer to the MAX31856 device structure.
 * @param[in] addr   Register address to read from.
 * @param[out] value Pointer to store the read value.
 *
 * @return
 *     - ESP_OK on success
 *     - Appropriate esp_err_t error code otherwise
 */
esp_err_t max31856_read(max31856_dev_t *data, uint8_t addr, uint8_t *value);

#ifdef __cplusplus
}
#endif
