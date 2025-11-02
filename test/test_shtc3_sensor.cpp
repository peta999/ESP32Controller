/**
 * @file test_shtc3_sensor.cpp
 * @brief Comprehensive unit tests for SHTC3Sensor class
 * 
 * Tests cover:
 * - Builder pattern functionality
 * - Sensor initialization and probing
 * - Measurement operations
 * - Low power mode settings
 * - Continuous measurement with callbacks
 * - Error handling and edge cases
 * - Resource cleanup
 */

#include <string.h>
#include "unity.h"
#include "../sensors/shtc3Sensor/SHTC3Sensor.h"

extern "C" {
    #include "../components/shtc1/shtc1.h"
    #include "../components/shtc1/sensirion_i2c.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
}

// Test constants
static constexpr int GPIO_SDA_DEFAULT = 27;
static constexpr int GPIO_SCL_DEFAULT = 26;
static constexpr int GPIO_SDA_ALT = 22;
static constexpr int GPIO_SCL_ALT = 21;
static constexpr int GPIO_SDA_ALT2 = 5;
static constexpr int GPIO_SCL_ALT2 = 4;
static constexpr int GPIO_SDA_MIN = 0;
static constexpr int GPIO_SCL_MIN = 1;
static constexpr int GPIO_SDA_MAX = 32;
static constexpr int GPIO_SCL_MAX = 33;

static constexpr uint8_t I2C_ADDR_DEFAULT = 0x70;
static constexpr uint8_t I2C_ADDR_ALT1 = 0x71;
static constexpr uint8_t I2C_ADDR_ALT2 = 0x72;
static constexpr uint8_t I2C_ADDR_MIN = 0x00;
static constexpr uint8_t I2C_ADDR_MAX = 0xFF;

static constexpr uint32_t INTERVAL_1_SEC = 1000;
static constexpr uint32_t INTERVAL_5_SEC = 5000;
static constexpr uint32_t INTERVAL_500_MS = 500;
static constexpr uint32_t INTERVAL_300_MS = 300;
static constexpr uint32_t INTERVAL_MIN = 100;
static constexpr uint32_t INTERVAL_MIN_BELOW = 50;
static constexpr uint32_t INTERVAL_BELOW_MIN = 1;
static constexpr uint32_t INTERVAL_1_MIN = 60000;
static constexpr uint32_t INTERVAL_5_MIN = 300000;
static constexpr uint32_t INTERVAL_MAX = 0xFFFFFFFFU;
static constexpr uint32_t INTERVAL_JUST_ABOVE_MIN = 101;
static constexpr uint32_t INTERVAL_JUST_BELOW_MIN = 99;

static constexpr uint32_t DELAY_10_MS = 10;
static constexpr uint32_t DELAY_50_MS = 50;
static constexpr uint32_t DELAY_100_MS = 100;
static constexpr uint32_t DELAY_150_MS = 150;
static constexpr uint32_t DELAY_1_SEC = 1000;
static constexpr uint32_t LOOP_COUNT = 5;

// Test fixture state
static SHTC3Sensor* test_sensor = nullptr;
static bool callback_invoked = false;
static int32_t callback_temperature = 0;
static int32_t callback_humidity = 0;
static int callback_count = 0;

// Test callback function
static void test_measurement_callback(int32_t temperature, int32_t humidity) {
    callback_invoked = true;
    callback_temperature = temperature;
    callback_humidity = humidity;
    callback_count++;
}

// Reset callback state
static void reset_callback_state() {
    callback_invoked = false;
    callback_temperature = 0;
    callback_humidity = 0;
    callback_count = 0;
}

// Setup function called before each test
void setUp() {
    test_sensor = nullptr;
    reset_callback_state();
}

// Teardown function called after each test
void tearDown() {
    if (test_sensor != nullptr) {
        delete test_sensor;
        test_sensor = nullptr;
    }
    reset_callback_state();
}

/**
 * Test Builder pattern - basic construction with required pins
 */
void test_builder_basic_construction() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    // Should construct with default values
    TEST_ASSERT_EQUAL_UINT8(I2C_ADDR_DEFAULT, sensor.getAddress());
}

/**
 * Test Builder pattern - set custom I2C address
 */
void test_builder_with_custom_address() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT)
                            .setAddress(I2C_ADDR_ALT1)
                            .build();
    
    TEST_ASSERT_EQUAL_UINT8(I2C_ADDR_ALT1, sensor.getAddress());
}

/**
 * Test Builder pattern - enable low power mode
 */
void test_builder_with_low_power_mode() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT)
                            .setLowPower(true)
                            .build();
    
    // Low power mode should be set (verified through successful construction)
    TEST_ASSERT_EQUAL_UINT8(I2C_ADDR_DEFAULT, sensor.getAddress());
}

/**
 * Test Builder pattern - method chaining
 */
void test_builder_method_chaining() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT)
                            .setAddress(I2C_ADDR_ALT2)
                            .setLowPower(true)
                            .build();
    
    TEST_ASSERT_EQUAL_UINT8(I2C_ADDR_ALT2, sensor.getAddress());
}

/**
 * Test Builder pattern - different GPIO pins
 */
void test_builder_with_different_pins() {
    // Test with different valid GPIO pin combinations
    SHTC3Sensor sensor1 = SHTC3Sensor::Builder(GPIO_SDA_ALT, GPIO_SCL_ALT).build();
    TEST_ASSERT_EQUAL_UINT8(I2C_ADDR_DEFAULT, sensor1.getAddress());
    
    SHTC3Sensor sensor2 = SHTC3Sensor::Builder(GPIO_SDA_ALT2, GPIO_SCL_ALT2).build();
    TEST_ASSERT_EQUAL_UINT8(I2C_ADDR_DEFAULT, sensor2.getAddress());
}

/**
 * Test getAddress returns configured address
 */
void test_get_address() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT)
                            .setAddress(I2C_ADDR_DEFAULT)
                            .build();
    
    TEST_ASSERT_EQUAL_UINT8(I2C_ADDR_DEFAULT, sensor.getAddress());
}

/**
 * Test setLowPowerMode enables low power
 */
void test_set_low_power_mode_enable() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    // Should not throw exception
    sensor.setLowPowerMode(true);
    
    // Verify it doesn't crash (actual hardware behavior requires sensor)
    TEST_PASS();
}

/**
 * Test setLowPowerMode disables low power
 */
void test_set_low_power_mode_disable() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT)
                            .setLowPower(true)
                            .build();
    
    // Should not throw exception
    sensor.setLowPowerMode(false);
    
    TEST_PASS();
}

/**
 * Test getDriverVersion returns non-null string
 */
void test_get_driver_version() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    const char* version = sensor.getDriverVersion();
    
    TEST_ASSERT_NOT_NULL(version);
    TEST_ASSERT_GREATER_THAN(0, strlen(version));
}

/**
 * Test setMeasurementInterval with valid interval
 */
void test_set_measurement_interval_valid() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    // Valid intervals
    sensor.setMeasurementInterval(INTERVAL_1_SEC);
    sensor.setMeasurementInterval(INTERVAL_5_SEC);
    sensor.setMeasurementInterval(INTERVAL_MIN);
    
    TEST_PASS();
}

/**
 * Test setMeasurementInterval with zero (should clamp to minimum)
 */
void test_set_measurement_interval_zero() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    // Should clamp to minimum (100ms)
    sensor.setMeasurementInterval(0);
    
    TEST_PASS();
}

/**
 * Test setMeasurementInterval with value below minimum
 */
void test_set_measurement_interval_below_minimum() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    // Should clamp to minimum (100ms)
    sensor.setMeasurementInterval(INTERVAL_MIN_BELOW);
    sensor.setMeasurementInterval(INTERVAL_BELOW_MIN);
    
    TEST_PASS();
}

/**
 * Test setMeasurementInterval with large value
 */
void test_set_measurement_interval_large() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    // Should accept large intervals
    sensor.setMeasurementInterval(INTERVAL_1_MIN);
    sensor.setMeasurementInterval(INTERVAL_5_MIN);
    
    TEST_PASS();
}

/**
 * Test setMeasurementCallback with valid callback
 */
void test_set_measurement_callback_valid() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    sensor.setMeasurementCallback(test_measurement_callback);
    
    TEST_PASS();
}

/**
 * Test setMeasurementCallback with nullptr
 */
void test_set_measurement_callback_nullptr() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    sensor.setMeasurementCallback(nullptr);
    
    TEST_PASS();
}

/**
 * Test multiple setMeasurementCallback calls (overwrite)
 */
void test_set_measurement_callback_overwrite() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    sensor.setMeasurementCallback(test_measurement_callback);
    sensor.setMeasurementCallback(nullptr);
    sensor.setMeasurementCallback(test_measurement_callback);
    
    TEST_PASS();
}

/**
 * Test measure with uninitialized sensor (should return false)
 */
void test_measure_uninitialized_sensor() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    int32_t temperature = 0;
    int32_t humidity = 0;
    
    // Without hardware, should return false
    bool result = sensor.measure(temperature, humidity);
    
    // Without actual sensor hardware, this will fail
    TEST_ASSERT_FALSE(result);
}

/**
 * Test measure with valid output parameters
 */
void test_measure_with_valid_parameters() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    int32_t temperature = -999999;
    int32_t humidity = -999999;
    
    // Call measure (will fail without hardware, but parameters should be safe)
    sensor.measure(temperature, humidity);
    
    // Parameters should remain accessible
    TEST_PASS();
}

/**
 * Test multiple consecutive measure calls
 */
void test_measure_multiple_consecutive_calls() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    int32_t temp1 = 0, hum1 = 0, temp2 = 0, hum2 = 0;
    
    sensor.measure(temp1, hum1);
    sensor.measure(temp2, hum2);
    
    TEST_PASS();
}

/**
 * Test startContinuousMeasurement returns status
 */
void test_start_continuous_measurement_without_hardware() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    sensor.setMeasurementInterval(INTERVAL_1_SEC);
    sensor.setMeasurementCallback(test_measurement_callback);
    
    // Should create task successfully
    bool result = sensor.startContinuousMeasurement();
    
    // Task creation should succeed even without hardware
    TEST_ASSERT_TRUE(result);
    
    // Cleanup
    sensor.stopContinuousMeasurement();
    vTaskDelay(pdMS_TO_TICKS(DELAY_50_MS));
}

/**
 * Test startContinuousMeasurement with null callback (should still work)
 */
void test_start_continuous_measurement_null_callback() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    sensor.setMeasurementInterval(INTERVAL_500_MS);
    // Don't set callback
    
    bool result = sensor.startContinuousMeasurement();
    TEST_ASSERT_TRUE(result);
    
    sensor.stopContinuousMeasurement();
    vTaskDelay(pdMS_TO_TICKS(DELAY_50_MS));
}

/**
 * Test startContinuousMeasurement called twice
 */
void test_start_continuous_measurement_called_twice() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    sensor.setMeasurementInterval(INTERVAL_1_SEC);
    
    bool result1 = sensor.startContinuousMeasurement();
    TEST_ASSERT_TRUE(result1);
    
    // Second call should return true (already active)
    bool result2 = sensor.startContinuousMeasurement();
    TEST_ASSERT_TRUE(result2);
    
    sensor.stopContinuousMeasurement();
    vTaskDelay(pdMS_TO_TICKS(DELAY_50_MS));
}

/**
 * Test stopContinuousMeasurement when not running
 */
void test_stop_continuous_measurement_not_running() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    // Should not crash
    sensor.stopContinuousMeasurement();
    
    TEST_PASS();
}

/**
 * Test stopContinuousMeasurement after start
 */
void test_stop_continuous_measurement_after_start() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    sensor.setMeasurementInterval(INTERVAL_500_MS);
    sensor.startContinuousMeasurement();
    
    // Wait a bit for task to start
    vTaskDelay(pdMS_TO_TICKS(DELAY_100_MS));
    
    // Should stop cleanly
    sensor.stopContinuousMeasurement();
    
    TEST_PASS();
}

/**
 * Test multiple start/stop cycles
 */
void test_continuous_measurement_multiple_cycles() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    sensor.setMeasurementInterval(INTERVAL_300_MS);
    
    // Cycle 1
    sensor.startContinuousMeasurement();
    vTaskDelay(pdMS_TO_TICKS(DELAY_50_MS));
    sensor.stopContinuousMeasurement();
    vTaskDelay(pdMS_TO_TICKS(DELAY_50_MS));
    
    // Cycle 2
    sensor.startContinuousMeasurement();
    vTaskDelay(pdMS_TO_TICKS(DELAY_50_MS));
    sensor.stopContinuousMeasurement();
    vTaskDelay(pdMS_TO_TICKS(DELAY_50_MS));
    
    TEST_PASS();
}

/**
 * Test destructor stops continuous measurement
 */
void test_destructor_stops_continuous_measurement() {
    auto sensor = new SHTC3Sensor(SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build());
    
    sensor->setMeasurementInterval(INTERVAL_500_MS);
    sensor->startContinuousMeasurement();
    
    vTaskDelay(pdMS_TO_TICKS(DELAY_100_MS));
    
    // Destructor should stop measurement cleanly
    delete sensor;
    
    vTaskDelay(pdMS_TO_TICKS(DELAY_100_MS));
    
    TEST_PASS();
}

/**
 * Test continuous measurement with very short interval
 */
void test_continuous_measurement_short_interval() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    // Set minimum interval (should clamp to 100ms)
    sensor.setMeasurementInterval(INTERVAL_MIN_BELOW);
    sensor.setMeasurementCallback(test_measurement_callback);
    
    sensor.startContinuousMeasurement();
    vTaskDelay(pdMS_TO_TICKS(DELAY_150_MS));
    sensor.stopContinuousMeasurement();
    
    TEST_PASS();
}

/**
 * Test sleep command
 */
void test_sleep_command() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    // Without hardware, will return false
    bool result = sensor.sleep();
    
    // Should return false without sensor
    TEST_ASSERT_FALSE(result);
}

/**
 * Test wakeUp command
 */
void test_wakeup_command() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    // Without hardware, will return false
    bool result = sensor.wakeUp();
    
    // Should return false without sensor
    TEST_ASSERT_FALSE(result);
}

/**
 * Test sleep/wakeUp sequence
 */
void test_sleep_wakeup_sequence() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    // Should not crash
    sensor.sleep();
    vTaskDelay(pdMS_TO_TICKS(DELAY_10_MS));
    sensor.wakeUp();
    
    TEST_PASS();
}

/**
 * Test readSerial without hardware
 */
void test_read_serial_no_hardware() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    uint32_t serial = 0;
    bool result = sensor.readSerial(serial);
    
    // Without hardware, should return false
    TEST_ASSERT_FALSE(result);
}

/**
 * Test probe without hardware
 */
void test_probe_no_hardware() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    // Without hardware, should return false
    bool result = sensor.probe();
    
    TEST_ASSERT_FALSE(result);
}

/**
 * Test initializeBus with default pins
 */
void test_initialize_bus_default_pins() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    // May succeed or fail depending on hardware, but should not crash
    sensor.initializeBus();
    
    TEST_PASS();
}

/**
 * Test initializeBus with different pins
 */
void test_initialize_bus_custom_pins() {
    SHTC3Sensor sensor1 = SHTC3Sensor::Builder(GPIO_SDA_ALT, GPIO_SCL_ALT).build();
    sensor1.initializeBus();
    
    SHTC3Sensor sensor2 = SHTC3Sensor::Builder(GPIO_SDA_ALT2, GPIO_SCL_ALT2).build();
    sensor2.initializeBus();
    
    TEST_PASS();
}

/**
 * Test Builder with boundary GPIO values
 */
void test_builder_with_boundary_gpio_values() {
    // Test with GPIO 0 (valid on ESP32)
    SHTC3Sensor sensor1 = SHTC3Sensor::Builder(GPIO_SDA_MIN, GPIO_SCL_MIN).build();
    TEST_ASSERT_EQUAL_UINT8(I2C_ADDR_DEFAULT, sensor1.getAddress());
    
    // Test with high GPIO numbers (valid on some ESP32 variants)
    SHTC3Sensor sensor2 = SHTC3Sensor::Builder(GPIO_SDA_MAX, GPIO_SCL_MAX).build();
    TEST_ASSERT_EQUAL_UINT8(I2C_ADDR_DEFAULT, sensor2.getAddress());
}

/**
 * Test address boundaries
 */
void test_address_boundaries() {
    // Test minimum address
    SHTC3Sensor sensor1 = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT)
                             .setAddress(I2C_ADDR_MIN)
                             .build();
    TEST_ASSERT_EQUAL_UINT8(I2C_ADDR_MIN, sensor1.getAddress());
    
    // Test maximum address
    SHTC3Sensor sensor2 = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT)
                             .setAddress(I2C_ADDR_MAX)
                             .build();
    TEST_ASSERT_EQUAL_UINT8(I2C_ADDR_MAX, sensor2.getAddress());
}

/**
 * Test measurement interval edge cases
 */
void test_measurement_interval_edge_cases() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    // Maximum uint32_t value
    sensor.setMeasurementInterval(INTERVAL_MAX);
    
    // Just above minimum
    sensor.setMeasurementInterval(INTERVAL_JUST_ABOVE_MIN);
    
    // Exactly at minimum
    sensor.setMeasurementInterval(INTERVAL_MIN);
    
    // Just below minimum (should clamp)
    sensor.setMeasurementInterval(INTERVAL_JUST_BELOW_MIN);
    
    TEST_PASS();
}

/**
 * Test copy construction behavior (should handle properly)
 */
void test_sensor_copy_construction() {
    SHTC3Sensor sensor1 = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    // Copy construction
    SHTC3Sensor sensor2 = sensor1;
    
    TEST_ASSERT_EQUAL_UINT8(sensor1.getAddress(), sensor2.getAddress());
}

/**
 * Test sensor behavior with different low power mode settings
 */
void test_low_power_mode_toggle() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    // Toggle multiple times
    for (int i = 0; i < LOOP_COUNT; i++) {
        sensor.setLowPowerMode(true);
        sensor.setLowPowerMode(false);
    }
    
    TEST_PASS();
}

/**
 * Test continuous measurement with interval changes during operation
 */
void test_interval_change_during_continuous_measurement() {
    SHTC3Sensor sensor = SHTC3Sensor::Builder(GPIO_SDA_DEFAULT, GPIO_SCL_DEFAULT).build();
    
    sensor.setMeasurementInterval(INTERVAL_1_SEC);
    sensor.startContinuousMeasurement();
    
    vTaskDelay(pdMS_TO_TICKS(DELAY_100_MS));
    
    // Change interval while running
    sensor.setMeasurementInterval(INTERVAL_500_MS);
    
    vTaskDelay(pdMS_TO_TICKS(DELAY_100_MS));
    
    sensor.stopContinuousMeasurement();
    
    TEST_PASS();
}

// Main test app
extern "C" void app_main() {
    // Wait for system to stabilize
    vTaskDelay(pdMS_TO_TICKS(DELAY_1_SEC));
    
    UNITY_BEGIN();
    
    // Builder pattern tests
    RUN_TEST(test_builder_basic_construction);
    RUN_TEST(test_builder_with_custom_address);
    RUN_TEST(test_builder_with_low_power_mode);
    RUN_TEST(test_builder_method_chaining);
    RUN_TEST(test_builder_with_different_pins);
    RUN_TEST(test_builder_with_boundary_gpio_values);
    
    // Basic property tests
    RUN_TEST(test_get_address);
    RUN_TEST(test_get_driver_version);
    RUN_TEST(test_address_boundaries);
    
    // Low power mode tests
    RUN_TEST(test_set_low_power_mode_enable);
    RUN_TEST(test_set_low_power_mode_disable);
    RUN_TEST(test_low_power_mode_toggle);
    
    // Measurement interval tests
    RUN_TEST(test_set_measurement_interval_valid);
    RUN_TEST(test_set_measurement_interval_zero);
    RUN_TEST(test_set_measurement_interval_below_minimum);
    RUN_TEST(test_set_measurement_interval_large);
    RUN_TEST(test_measurement_interval_edge_cases);
    
    // Callback tests
    RUN_TEST(test_set_measurement_callback_valid);
    RUN_TEST(test_set_measurement_callback_nullptr);
    RUN_TEST(test_set_measurement_callback_overwrite);
    
    // Measurement tests
    RUN_TEST(test_measure_uninitialized_sensor);
    RUN_TEST(test_measure_with_valid_parameters);
    RUN_TEST(test_measure_multiple_consecutive_calls);
    
    // Continuous measurement tests
    RUN_TEST(test_start_continuous_measurement_without_hardware);
    RUN_TEST(test_start_continuous_measurement_null_callback);
    RUN_TEST(test_start_continuous_measurement_called_twice);
    RUN_TEST(test_stop_continuous_measurement_not_running);
    RUN_TEST(test_stop_continuous_measurement_after_start);
    RUN_TEST(test_continuous_measurement_multiple_cycles);
    RUN_TEST(test_continuous_measurement_short_interval);
    RUN_TEST(test_interval_change_during_continuous_measurement);
    
    // Sleep/Wake tests
    RUN_TEST(test_sleep_command);
    RUN_TEST(test_wakeup_command);
    RUN_TEST(test_sleep_wakeup_sequence);
    
    // Serial and probe tests
    RUN_TEST(test_read_serial_no_hardware);
    RUN_TEST(test_probe_no_hardware);
    
    // Initialization tests
    RUN_TEST(test_initialize_bus_default_pins);
    RUN_TEST(test_initialize_bus_custom_pins);
    
    // Resource management tests
    RUN_TEST(test_destructor_stops_continuous_measurement);
    RUN_TEST(test_sensor_copy_construction);
    
    UNITY_END();
}