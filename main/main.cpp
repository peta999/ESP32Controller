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



extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_chip_info.h"
    #include "esp_flash.h"
    #include "esp_system.h"    
    #include "driver/i2c.h"
    #include "../components/shtc1/shtc1.h"
}


extern "C" void app_main(void)
{
    /* Initialize the i2c bus for the current platform */
    sensirion_i2c_init();

    /* Busy loop for initialization, because the main loop does not work without
     * a sensor.
     */
    while (shtc1_probe() != STATUS_OK) {
        printf("SHT sensor probing failed\n");
    }
    printf("SHT sensor probing successful\n");

    while (1) {
        int32_t temperature, humidity;
        /* Measure temperature and relative humidity and store into variables
         * temperature, humidity (each output multiplied by 1000).
         */
        int8_t ret = shtc1_measure_blocking_read(&temperature, &humidity);
        if (ret == STATUS_OK) {
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
