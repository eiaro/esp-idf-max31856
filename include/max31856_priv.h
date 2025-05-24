/*
 * SPDX-FileCopyrightText: 2025 Ronny Eia <3652665+eiaro@users.noreply.github.com>
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once
#include "max31856.h"
#include "esp_err.h"
#include "driver/spi_master.h"

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

#define MAX31856_RD_WR_MASK BIT(7)
#define MAX31856_CR1_TCTYPE_MASK GENMASK(3, 0)
#define MAX31856_CR1_AVERAGE_MASK GENMASK(6, 4)
#define MAX31856_CR1_AVERAGE_SHIFT 4
#define MAX31856_CR0_OC_MASK GENMASK(5, 4)
#define MAX31856_CR0_OC_SHIFT 4
#define MAX31856_CR0_AUTOCONVERT BIT(7)
#define MAX31856_CR0_1SHOT BIT(6)
#define MAX31856_CR0_CJ BIT(3)
#define MAX31856_CR0_FAULT BIT(2)


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




esp_err_t max31856_write(max31856_dev_t *data, uint8_t addr, uint8_t value);
float parse_cold_junction(uint8_t cjth, uint8_t cjtl);
float parse_thermocouple(uint8_t msb, uint8_t mid, uint8_t lsb);
esp_err_t max31856_read(max31856_dev_t *data, uint8_t addr, uint8_t *value);
