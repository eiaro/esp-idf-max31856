/*
 * SPDX-FileCopyrightText: 2025 Ronny Eia <3652665+eiaro@users.noreply.github.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "max31856.h"

#define ESP_HOST VSPI_HOST
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_CLK GPIO_NUM_18
#define PIN_NUM_CS GPIO_NUM_5

static const char *TAG = "max31856_basic";

esp_err_t ret;

void app_main(void)
{
    spi_device_handle_t spi;
    max31856_dev_t max31856_device = {
        .thermocouple_type = MAX31856_TC_TYPE_K,
        .filter_50hz = true,
        .averaging = 0,
    };
    
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);

    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2048,
    };
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 8,
        .dummy_bits = 0,
        .clock_speed_hz = 1 * 1000, // Clock out at 1 MHz
        .mode = 1,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX,
    };
    
    ESP_LOGI(TAG, "Initialize SPI bus: miso=%d,mosi=%d,sclk=%d", 
        buscfg.miso_io_num,
        buscfg.mosi_io_num,
        buscfg.sclk_io_num
    );
    ESP_ERROR_CHECK(spi_bus_initialize(ESP_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(ESP_HOST, &devcfg, &spi));

    max31856_device.spi_dev = spi;
    max31856_device.thermocouple_type = MAX31856_TC_TYPE_K;
    max31856_device.use_cold_junction = true;
    max31856_device.filter_50hz = true;
    ESP_ERROR_CHECK(max31856_init(&max31856_device));

    float tc_temperature, cj_temperature;
    uint8_t status;
    while (1) {
        max31856_read_thermocouple(&max31856_device, &tc_temperature);
        max31856_read_cold_junction(&max31856_device, &cj_temperature);
        max31856_read_fault_status(&max31856_device, &status);

        ESP_LOGI(TAG, "Read: tc=%f, cj=%f, status=%x", 
            tc_temperature,
            cj_temperature,
            status
        );

        // Sleep for 5 second (1000ms)
        vTaskDelay(pdMS_TO_TICKS(5 * 1000));        
    }


}
