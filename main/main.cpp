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
#include "SHTC1Sensor.h"

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_chip_info.h"
    #include "esp_flash.h"
    #include "esp_system.h"
    #include "driver/i2c.h"
    #include "../components/shtc1/sensirion_i2c.h"
}

extern "C" void app_main(void)
{
    // Create sensor instance with default configuration (address 0x70, normal power mode)
    SHTC1Sensor sensor;

    /* Initialize the i2c bus for the current platform */
    sensor.initializeBus();

    /* Busy loop for initialization, because the main loop does not work without
     * a sensor.
     */
    while (!sensor.probe()) {
        printf("SHT sensor probing failed\n");
    }
    printf("SHT sensor probing successful\n");

    while (1) {
        int32_t temperature, humidity;
        /* Measure temperature and relative humidity and store into variables
         * temperature, humidity (each output multiplied by 1000).
         */
        if (sensor.measure(temperature, humidity)) {
            printf("measured temperature: %0.2f degreeCelsius, "
                   "measured humidity: %0.2f percentRH\n",
                   temperature / 1000.0f, humidity / 1000.0f);
        } else {
            printf("error reading measurement\n");
        }

        sensirion_sleep_usec(1000000);
    }
    return;
}
