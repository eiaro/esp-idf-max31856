// SPDX-FileCopyrightText: 2025 Ronny Eia <3652665+eiaro@users.noreply.github.com>
//
// SPDX-License-Identifier: MIT
//
// C++ wrapper for the MAX31856 ESP-IDF driver
#pragma once

#include "max31856.h"
#include <utility>
#include <cstdint>
#include <stdexcept>

namespace max31856
{

    class Max31856
    {
    public:
        // Type aliases for enums
        using ThermocoupleType = max31856_thermocouple_type_t;
        using Averaging = max31856_average_t;
        using OpenCircuitDetection = max31856_oc_detection_t;

        // Deleted copy semantics, allow move
        Max31856(const Max31856 &) = delete;
        Max31856 &operator=(const Max31856 &) = delete;
        Max31856(Max31856 &&other) noexcept : dev_(other.dev_), spi_handle_(other.spi_handle_), owns_spi_(other.owns_spi_)
        {
            other.spi_handle_ = nullptr;
            other.owns_spi_ = false;
        }
        Max31856 &operator=(Max31856 &&other) noexcept
        {
            if (this != &other)
            {
                cleanup();
                dev_ = other.dev_;
                spi_handle_ = other.spi_handle_;
                owns_spi_ = other.owns_spi_;
                other.spi_handle_ = nullptr;
                other.owns_spi_ = false;
            }
            return *this;
        }

        // Construct and add SPI device (RAII, simplified)
        /**
         * @brief Constructs a Max31856 thermocouple interface object.
         *
         * Initializes the MAX31856 device on the specified SPI host and chip select pin,
         * with configurable parameters for thermocouple type, filtering, averaging, open circuit detection,
         * and cold junction compensation.
         *
         * @param host            SPI host device to use (e.g., HSPI_HOST or VSPI_HOST).
         * @param cs_pin          GPIO pin number used for chip select (CS).
         * @param clock_speed_hz  SPI clock speed in Hz (default: 1,000,000 Hz).
         * @param type            Thermocouple type (default: MAX31856_TC_TYPE_K).
         * @param filter_50hz     Enable 50Hz noise filter (default: false).
         * @param avg             Averaging mode for temperature readings (default: MAX31856_AVG_1).
         * @param oc              Open circuit detection mode (default: MAX31856_OC_DETECT_OFF).
         * @param use_cold_junction Enable cold junction compensation (default: true).
         *
         * @note If the device cannot be added to the SPI bus, the object will not own the SPI handle,
         *       and further operations may fail. Consider checking for initialization errors after construction.
         */
        Max31856(spi_host_device_t host,
                 gpio_num_t cs_pin,
                 int clock_speed_hz = 1000000, // 1 MHz default
                 ThermocoupleType type = MAX31856_TC_TYPE_K,
                 bool filter_50hz = false,
                 Averaging avg = MAX31856_AVG_1,
                 OpenCircuitDetection oc = MAX31856_OC_DETECT_OFF,
                 bool use_cold_junction = true)
        {
            spi_device_interface_config_t devcfg = {};
            devcfg.command_bits = 0;
            devcfg.address_bits = 8;
            devcfg.dummy_bits = 0;
            devcfg.mode = 1; // MAX31856 uses SPI mode 1
            devcfg.clock_speed_hz = clock_speed_hz;
            devcfg.spics_io_num = cs_pin;
            devcfg.queue_size = 1;
            devcfg.flags = SPI_DEVICE_HALFDUPLEX;
            esp_err_t err = spi_bus_add_device(host, &devcfg, &spi_handle_);
            if (err != ESP_OK)
            {
                // Could not add device, leave handle null and set owns_spi_ false
                spi_handle_ = nullptr;
                owns_spi_ = false;
                // Optionally, set an error flag or log here
            }
            else
            {
                dev_.spi_dev = spi_handle_;
                dev_.thermocouple_type = type;
                dev_.filter_50hz = filter_50hz;
                dev_.averaging = avg;
                dev_.oc_detection = oc;
                dev_.use_cold_junction = use_cold_junction;
                owns_spi_ = true;
            }
        }

        // Construct from existing SPI device handle (does not own)
        Max31856(spi_device_handle_t spi,
                 ThermocoupleType type = MAX31856_TC_TYPE_K,
                 bool filter_50hz = false,
                 Averaging avg = MAX31856_AVG_1,
                 OpenCircuitDetection oc = MAX31856_OC_DETECT_OFF,
                 bool use_cold_junction = true)
        {
            dev_.spi_dev = spi;
            dev_.thermocouple_type = type;
            dev_.filter_50hz = filter_50hz;
            dev_.averaging = avg;
            dev_.oc_detection = oc;
            dev_.use_cold_junction = use_cold_junction;
            spi_handle_ = spi;
            owns_spi_ = false;
        }

        ~Max31856() { cleanup(); }

        // Initialize the device
        /**
         * @brief Initializes the MAX31856 device.
         *
         * This function sets up the MAX31856 device by calling the underlying
         * max31856_init function with the device context.
         *
         * @return
         *     - ESP_OK on success
         *     - Appropriate esp_err_t error code otherwise
         */
        esp_err_t init()
        {
            return max31856_init(&dev_);
        }

        /**
         * @brief Reads the current thermocouple temperature.
         *
         * This function retrieves the temperature measured by the thermocouple sensor
         * and stores it in the provided reference variable.
         *
         * @param[out] temp Reference to a float where the measured temperature will be stored.
         * @return esp_err_t Returns ESP_OK on success, or an error code on failure.
         */
        esp_err_t readThermocouple(float &temp)
        {
            return max31856_read_thermocouple(&dev_, &temp);
        }

        /**
         * @brief Reads the cold junction temperature from the MAX31856 device.
         *
         * This function retrieves the current cold junction (reference junction) temperature
         * and stores it in the provided float reference parameter.
         *
         * @param[out] temp Reference to a float variable where the cold junction temperature will be stored.
         * @return esp_err_t Returns ESP_OK on success, or an appropriate error code on failure.
         */
        esp_err_t readColdJunction(float &temp)
        {
            return max31856_read_cold_junction(&dev_, &temp);
        }

        /**
         * @brief Reads the fault status register from the MAX31856 device.
         *
         * This function retrieves the current fault status register value and stores it
         * in the provided reference variable.
         *
         * @param[out] status Reference to a uint8_t where the fault status will be stored.
         * @return esp_err_t Returns ESP_OK on success, or an error code on failure.
         */
        esp_err_t readFaultStatus(uint8_t &status)
        {
            return max31856_read_fault_status(&dev_, &status);
        }

        // Access to underlying C struct (if needed)
        max31856_dev_t *raw() { return &dev_; }
        const max31856_dev_t *raw() const { return &dev_; }
        spi_device_handle_t spi_handle() const { return spi_handle_; }

    private:
        max31856_dev_t dev_{};
        spi_device_handle_t spi_handle_ = nullptr;
        bool owns_spi_ = false;
        void cleanup()
        {
            if (owns_spi_ && spi_handle_)
            {
                spi_bus_remove_device(spi_handle_);
                spi_handle_ = nullptr;
            }
        }
    };

} // namespace max31856
