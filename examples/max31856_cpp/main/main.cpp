// SPDX-FileCopyrightText: 2025 Ronny Eia <3652665+eiaro@users.noreply.github.com>
//
// SPDX-License-Identifier: MIT

#include <stdio.h>
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "max31856.hpp"

static const char* TAG = "MAX31856_TEST";

// SPI Configuration
#define ESP_HOST VSPI_HOST
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
#define SPI_CLOCK_HZ 1000000  // 1 MHz

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "MAX31856 Test Program Starting...");

    // Initialize SPI bus
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = PIN_NUM_MOSI;
    buscfg.miso_io_num = PIN_NUM_MISO;
    buscfg.sclk_io_num = PIN_NUM_CLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 2048;
    ESP_ERROR_CHECK(spi_bus_initialize(ESP_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Create MAX31856 instance using new C++ constructor (RAII, simplified)
    max31856::Max31856 thermocouple(ESP_HOST, (gpio_num_t)PIN_NUM_CS, SPI_CLOCK_HZ);

    // Initialize the sensor
    esp_err_t ret = thermocouple.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MAX31856!");
        return;
    }

    ESP_LOGI(TAG, "MAX31856 initialized successfully!");

    // Main loop
    float tc = 0.0f, cj = 0.0f;
    uint8_t fault = 0;
    while (1) {
        esp_err_t err_tc = thermocouple.readThermocouple(tc);
        esp_err_t err_cj = thermocouple.readColdJunction(cj);
        if (err_tc == ESP_OK && err_cj == ESP_OK) {
            ESP_LOGI(TAG, "Thermocouple Temperature: %.2f°C", tc);
            ESP_LOGI(TAG, "Cold Junction Temperature: %.2f°C", cj);
        } else {
            ESP_LOGE(TAG, "Failed to read temperature(s): TC=%d, CJ=%d", err_tc, err_cj);
        }
        esp_err_t err_fault = thermocouple.readFaultStatus(fault);
        if (err_fault == ESP_OK && fault) {
            ESP_LOGW(TAG, "Fault detected: 0x%02x", fault);
        } else if (err_fault != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read fault status: %d", err_fault);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second
    }
}