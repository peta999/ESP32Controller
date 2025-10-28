/*
 * SPDX-FileCopyrightText: 2010-2025 Espressif Systems
 * SPDX-License-Identifier: CC0-1.0
 */

#include <iostream>
#include <iomanip>
#include <cinttypes>
#include "sdkconfig.h"
#include "esp_mac.h"
#include <stdio.h>
#include "../sensors/shtc3Sensor/SHTC3Sensor.h"

// Constants for sensor configuration
constexpr int I2C_SCL_PIN = 27;
constexpr int I2C_SDA_PIN = 26;
constexpr int MEASUREMENT_INTERVAL_MS = 5000;

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_system.h"
    #include "driver/i2c.h"
    #include "../components/shtc1/sensirion_i2c.h"
}

// Callback function to handle measurements
void measurementHandler(int32_t temperature, int32_t humidity) {
    printf("measured temperature: %0.2f degreeCelsius, "
           "measured humidity: %0.2f percentRH\n",
           temperature / 1000.0f, humidity / 1000.0f);
}

extern "C" void app_main(void)
{
    // Create sensor instance with default configuration (address 0x70, normal power mode)
    SHTC3Sensor sensor = SHTC3Sensor::Builder(I2C_SCL_PIN, I2C_SDA_PIN).build();

    /* Initialize the i2c bus for the current platform */
    sensor.initializeBus();

/* Probe loop with backoff delay to yield CPU, limited retries */
int retry_count = 0;
const int max_retries = 10;
int delay_ms = 50;
const int max_delay_ms = 1000;

while (!sensor.probe()) {
    printf("SHT sensor probing failed (retry %d/%d)\n", retry_count + 1, max_retries);
    retry_count++;
    if (retry_count >= max_retries) {
        printf("SHT sensor initialization failed after %d attempts\n", max_retries);
        return;
    }
vTaskDelay(pdMS_TO_TICKS(delay_ms));
    if (delay_ms < max_delay_ms) {
        delay_ms *= 2;
    }
}
printf("SHT sensor probing successful\n");

// Set measurement interval and callback, then start continuous measurements
sensor.setMeasurementInterval(MEASUREMENT_INTERVAL_MS); // 5 second interval
sensor.setMeasurementCallback(measurementHandler);
if (!sensor.startContinuousMeasurement()) {
    printf("Failed to start continuous measurement task (possibly insufficient memory)\n");
    return;
}

// Main task can exit or delay indefinitely
vTaskDelay(portMAX_DELAY);
    return;
}
