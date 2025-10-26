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
    SHTC3Sensor sensor;

    /* Initialize the i2c bus for the current platform */
    sensor.initializeBus();

/* Busy loop for initialization, because the main loop does not work without
 * a sensor.
 */
while (!sensor.probe()) {
    printf("SHT sensor probing failed\n");
}
printf("SHT sensor probing successful\n");

// Set measurement interval and callback, then start continuous measurements
sensor.setMeasurementInterval(5000); // 1 second interval
sensor.setMeasurementCallback(measurementHandler);
sensor.startContinuousMeasurement();

// Main task can exit or delay indefinitely
vTaskDelay(portMAX_DELAY);
    return;
}
