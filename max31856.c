/*
 * SPDX-FileCopyrightText: 2025 Ronny Eia <3652665+eiaro@users.noreply.github.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include <string.h>
#include "max31856_priv.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "max31856";

/* Forward declarations of helper functions */
static float parse_thermocouple(uint8_t msb, uint8_t mid, uint8_t lsb);
static float parse_cold_junction(uint8_t cjth, uint8_t cjtl);

/* Low-level SPI communication functions */
esp_err_t max31856_write(max31856_dev_t *data, uint8_t addr, uint8_t value) {
    spi_transaction_t t = { 
        .addr = (addr | MAX31856_RD_WR_MASK), // Send the address to write to               
        .length = 8,       
        .rxlength = 0, // No need to read back data
        .flags = SPI_TRANS_USE_TXDATA, // Use the tx_data field for data transfer
    };

    memcpy(t.tx_data, &value, sizeof(uint8_t)); // Copy the value to be written to the transaction data

    #ifdef CONFIG_MAX31856_ENABLE_DEBUG_LOG
    ESP_LOGD(TAG, "Writing 0x%02x to register 0x%02x", value, addr);
    #endif

    return spi_device_polling_transmit(data->spi_dev, &t);    
}

esp_err_t max31856_read(max31856_dev_t *data, uint8_t addr, uint8_t *value) {
    esp_err_t ret;

    spi_transaction_t t = {        
        .addr = addr, // Send the address to read from        
        .length = 0, // 8 bits for address + 8 bits for data
        .rxlength = 8, // Read 8 bits        
        .flags = SPI_TRANS_USE_RXDATA, // Use the rx_data field for data transfer
    };

    #ifdef CONFIG_MAX31856_ENABLE_DEBUG_LOG
    ESP_LOGD(TAG, "Reading from register 0x%02x", addr);
    #endif

    ret = spi_device_polling_transmit(data->spi_dev, &t);
    if (ret != ESP_OK) {
        return ret;
    }
    memcpy(value, t.rx_data, sizeof(uint8_t)); // Copy the received data to the value pointer
 
    #ifdef CONFIG_MAX31856_ENABLE_DEBUG_LOG
    ESP_LOGD(TAG, "Read value 0x%02x from register 0x%02x", *value, addr);
    #endif

    return ESP_OK;
}

/* Main public API functions */
esp_err_t max31856_init(max31856_dev_t *data) {
    esp_err_t ret;
    uint8_t reg_cr0_value, reg_cr1_value;

    /* It is recommended to turn off the auto conversion mode before configuring the device. 
     * This is done by writing 0 to the CR0 register's AUTOCONVERT bit.       
     */
    ret = max31856_read(data, MAX31856_CR0_REG, &reg_cr0_value);
    if (ret != ESP_OK) {
        return ret;
    }

    reg_cr0_value &= ~MAX31856_CR0_AUTOCONVERT; 
    ret = max31856_write(data, MAX31856_CR0_REG, reg_cr0_value);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Read Control Register 1 for setup */
    ret = max31856_read(data, MAX31856_CR1_REG, &reg_cr1_value);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Set the thermocouple type */
    reg_cr1_value &= ~MAX31856_CR1_TCTYPE_MASK; // Clear the TCTYPE bits
    reg_cr1_value |= data->thermocouple_type; // Set the TCTYPE bits
    

    /* Set the averaging mode */
    reg_cr1_value &= ~MAX31856_CR1_AVERAGE_MASK; // Clear the AVERAGE bits
    reg_cr1_value |= (data->averaging << MAX31856_CR1_AVERAGE_SHIFT); // Set the AVERAGE bits
    
    /* Write the CR1 */
    ret = max31856_write(data, MAX31856_CR1_REG, reg_cr1_value);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Set the OC detection mode */
    reg_cr0_value &= ~MAX31856_CR0_OC_MASK; // Clear the OC bits
    reg_cr0_value |= (MAX31856_OC_DETECT_1 << MAX31856_CR0_OC_SHIFT); // Set the OC bits

    /* set Auto Conversion Mode back on */
    reg_cr0_value &= ~MAX31856_CR0_1SHOT; // Clear the 1SHOT bit
    reg_cr0_value |= MAX31856_CR0_AUTOCONVERT; // Set the AUTOCONVERT bit

    if (data->filter_50hz) {
        reg_cr0_value |= BIT(3); // Set the 50Hz filter bit
    } else {
        reg_cr0_value &= ~BIT(3); // Clear the 50Hz filter bit
    }

    /* Set the cold junction compensation mode */
    if (data->use_cold_junction) {
        reg_cr0_value &= ~MAX31856_CR0_CJ; // Clear the CJC bit to enable cold junction compensation
    } else {
        reg_cr0_value |= MAX31856_CR0_CJ; // Set the CJC bit to disable cold junction compensation
    }

    /* Always turn off FAULT output - this could be added later */
    reg_cr0_value &= ~MAX31856_CR0_FAULT; // Clear the FAULT bit

    ret = max31856_write(data, MAX31856_CR0_REG, reg_cr0_value);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return ret;
}

esp_err_t max31856_read_thermocouple(max31856_dev_t *data, float *temperature) {
    esp_err_t ret;
    uint8_t ltcbl, ltcbm, ltcbh;

    // Read the temperature registers
    ret = max31856_read(data, MAX31856_LTCBL_REG, &ltcbl);
    if (ret != ESP_OK) return ret;

    ret = max31856_read(data, MAX31856_LTCBM_REG, &ltcbm);
    if (ret != ESP_OK) return ret;

    ret = max31856_read(data, MAX31856_LTCBH_REG, &ltcbh);
    if (ret != ESP_OK) return ret;
    
    // Convert to temperature in Celsius
    *temperature = parse_thermocouple(ltcbh, ltcbm, ltcbl);

    #ifdef CONFIG_MAX31856_ENABLE_SELF_TEST
    // Self-test for known values
    if (parse_thermocouple(0b11110000, 0b01100000, 0b00000000) != -250.0) {
        ESP_LOGE(TAG, "Thermocouple parsing function failed");
        return ESP_ERR_INVALID_STATE; // Parsing function failed
    };
    if (parse_thermocouple(0b00000110, 0b01001111, 0b00000000) != 100.9375) {
        ESP_LOGE(TAG, "Thermocouple parsing function failed");
        return ESP_ERR_INVALID_STATE; // Parsing function failed
    };
    #endif


    return ESP_OK;
}

esp_err_t max31856_read_cold_junction(max31856_dev_t *data, float *temperature) {
    esp_err_t ret;
    uint8_t cjtl, cjth, cjto;

    // Read the cold junction temperature offset register
    ret = max31856_read(data, MAX31856_CJTO_REG, &cjto);
    if (ret != ESP_OK) return ret;

    // Read the cold junction temperature registers
    ret = max31856_read(data, MAX31856_CJTL_REG, &cjtl);
    if (ret != ESP_OK) return ret;

    ret = max31856_read(data, MAX31856_CJTH_REG, &cjth);
    if (ret != ESP_OK) return ret;

    *temperature = parse_cold_junction(cjth, cjtl);
    
    #ifdef CONFIG_MAX31856_ENABLE_SELF_TEST
    // Self-test for known values
    if (parse_cold_junction(0b01111111, 0b11111100) != 127.984375) {
        ESP_LOGE(TAG, "Cold junction parsing function failed");
        return ESP_ERR_INVALID_STATE;
    };
    if (parse_cold_junction(0b11001001, 0b00000000) != -55.0) {
        ESP_LOGE(TAG, "Cold junction parsing function failed");
        return ESP_ERR_INVALID_STATE;
    };
    #endif

    return ESP_OK;
}

esp_err_t max31856_read_fault_status(max31856_dev_t *data, uint8_t *status) {
    esp_err_t ret;

    // Read the fault status register
    ret = max31856_read(data, MAX31856_SR_REG, status);
    if (ret != ESP_OK) {
        return ret;
    }

    #ifdef CONFIG_MAX31856_ENABLE_DEBUG_LOG
    ESP_LOGD(TAG, "Fault Status Register: 0x%02x", *status);
    #endif

    return ESP_OK;
}

/* Helper functions for temperature conversion */
static float parse_cold_junction(uint8_t cjth, uint8_t cjtl) {
    int8_t integer_part = (int8_t)cjth;        // signed heltallsdel
    uint8_t frac_raw = cjtl >> 2;              // få ut de 6 gyldige bitene

    float fractional_part = frac_raw * 0.015625f; // 1 LSB = 1/64 = 0.015625

    // For negative temperaturer: trekk fra fraksjonen
    return (integer_part >= 0) ? 
        (integer_part + fractional_part) : 
        (integer_part - fractional_part);
}

float parse_thermocouple(uint8_t msb, uint8_t mid, uint8_t lsb) {
    // Sett sammen 19-bit verdi (shift opp i 32-bit int)
    int32_t raw = ((int32_t)msb << 11) | ((int32_t)mid << 3) | (lsb >> 5);

    // Sign-extend 19-bit -> 32-bit dersom negativ
    if (raw & (1 << 18)) {
        raw |= ~((1 << 19) - 1);  // sett alle bit over bit 18 til 1
    }

    return raw * 0.0078125f;  // 1/128 = 0.0078125 °C per LSB
}