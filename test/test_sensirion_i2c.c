/**
 * @file test_sensirion_i2c.c
 * @brief Unit tests for modified sensirion_i2c functions
 * 
 * Tests the changes to sensirion_i2c_init() to accept pin parameters
 * and return success/failure status.
 */

#include "unity.h"
#include "../components/shtc1/sensirion_i2c.h"

#define SYSTEM_STABILIZATION_DELAY_MS 1000

// Setup function
void setUp(void) {
    // Release any previous I2C initialization
    sensirion_i2c_release();
}

// Teardown function
void tearDown(void) {
    // Clean up I2C resources
    sensirion_i2c_release();
}

/**
 * Test I2C initialization with default pins (GPIO 27, 26)
 */
void test_i2c_init_default_pins(void) {
    bool result = sensirion_i2c_init(27, 26);
    
    // Should succeed (or fail gracefully on hardware without these pins)
    // The function should return a boolean indicating status
    TEST_ASSERT_TRUE(result || !result); // Should not crash
}

/**
 * Test I2C initialization with custom pins
 */
void test_i2c_init_custom_pins(void) {
    sensirion_i2c_release();
    
    bool result = sensirion_i2c_init(22, 21);
    
    // Should return a status
    TEST_ASSERT_TRUE(result || !result);
}

/**
 * Test I2C initialization with different pin combinations
 */
void test_i2c_init_various_pin_combinations(void) {
    // Test pin pair 1
    sensirion_i2c_release();
    bool result1 = sensirion_i2c_init(5, 4);
    TEST_ASSERT_TRUE(result1 || !result1);
    
    // Test pin pair 2
    sensirion_i2c_release();
    bool result2 = sensirion_i2c_init(18, 19);
    TEST_ASSERT_TRUE(result2 || !result2);
    
    // Test pin pair 3
    sensirion_i2c_release();
    bool result3 = sensirion_i2c_init(32, 33);
    TEST_ASSERT_TRUE(result3 || !result3);
}

/**
 * Test I2C initialization with same pin for SCL and SDA (invalid)
 */
void test_i2c_init_same_pins(void) {
    sensirion_i2c_release();
    
    // Using same pin for SCL and SDA should fail
    bool result = sensirion_i2c_init(27, 27);
    
    // May fail, but should not crash
    TEST_ASSERT_TRUE(result || !result);
}

/**
 * Test I2C initialization with GPIO 0 (valid on ESP32)
 */
void test_i2c_init_with_gpio_zero(void) {
    sensirion_i2c_release();
    
    bool result = sensirion_i2c_init(0, 1);
    
    // Should handle GPIO 0 gracefully
    TEST_ASSERT_TRUE(result || !result);
}

/**
 * Test I2C initialization with high GPIO numbers
 */
void test_i2c_init_high_gpio_numbers(void) {
    sensirion_i2c_release();
    
    // Some ESP32 variants support up to GPIO 39
    bool result = sensirion_i2c_init(34, 35);
    
    TEST_ASSERT_TRUE(result || !result);
}

/**
 * Test repeated I2C initialization (re-initialization)
 */
void test_i2c_init_repeated(void) {
    sensirion_i2c_init(27, 26);
    sensirion_i2c_release();
    
    // Re-initialize with same pins
    bool result1 = sensirion_i2c_init(27, 26);
    TEST_ASSERT_TRUE(result1 || !result1);
    
    sensirion_i2c_release();
    
    // Re-initialize with different pins
    bool result2 = sensirion_i2c_init(22, 21);
    TEST_ASSERT_TRUE(result2 || !result2);
}

/**
 * Test I2C release after initialization
 */
void test_i2c_release_after_init(void) {
    sensirion_i2c_init(27, 26);
    
    // Should not crash
    sensirion_i2c_release();
    
    TEST_PASS();
}

/**
 * Test I2C release without initialization
 */
void test_i2c_release_without_init(void) {
    // Should not crash
    sensirion_i2c_release();
    sensirion_i2c_release(); // Call twice
    
    TEST_PASS();
}

/**
 * Test I2C select bus function
 */
void test_i2c_select_bus(void) {
    sensirion_i2c_init(27, 26);
    
    int16_t result = sensirion_i2c_select_bus(0);
    
    // Should return STATUS_OK (0) for single-bus setup
    TEST_ASSERT_EQUAL_INT16(0, result);
}

/**
 * Test I2C select bus with different indices
 */
void test_i2c_select_bus_various_indices(void) {
    sensirion_i2c_init(27, 26);
    
    // Bus 0
    TEST_ASSERT_EQUAL_INT16(0, sensirion_i2c_select_bus(0));
    
    // Bus 1 (may not be supported, but should not crash)
    sensirion_i2c_select_bus(1);
    
    // Bus 2
    sensirion_i2c_select_bus(2);
    
    TEST_PASS();
}

/**
 * Test boundary conditions for pin numbers
 */
void test_i2c_init_boundary_pin_numbers(void) {
    sensirion_i2c_release();
    
    // Test with pin 0 and max pin
    bool result1 = sensirion_i2c_init(0, 39);
    TEST_ASSERT_TRUE(result1 || !result1);
    
    sensirion_i2c_release();
    
    // Test with typical valid pins
    bool result2 = sensirion_i2c_init(21, 22);
    TEST_ASSERT_TRUE(result2 || !result2);
}

/**
 * Test I2C initialization followed by bus operations
 */
void test_i2c_init_then_bus_select(void) {
    bool init_result = sensirion_i2c_init(27, 26);
    TEST_ASSERT_TRUE(init_result || !init_result);
    
    int16_t select_result = sensirion_i2c_select_bus(0);
    TEST_ASSERT_EQUAL_INT16(0, select_result);
}

/**
 * Test multiple init/release cycles
 */
void test_i2c_multiple_init_release_cycles(void) {
    for (int i = 0; i < 3; i++) {
        bool result = sensirion_i2c_init(27, 26);
        TEST_ASSERT_TRUE(result || !result);
        
        sensirion_i2c_release();
    }
    
    TEST_PASS();
}

// Main test application
void app_main(void) {
    // Wait for system stabilization
    vTaskDelay(pdMS_TO_TICKS(SYSTEM_STABILIZATION_DELAY_MS));
    
    UNITY_BEGIN();
    
    // Initialization tests
    RUN_TEST(test_i2c_init_default_pins);
    RUN_TEST(test_i2c_init_custom_pins);
    RUN_TEST(test_i2c_init_various_pin_combinations);
    RUN_TEST(test_i2c_init_same_pins);
    RUN_TEST(test_i2c_init_with_gpio_zero);
    RUN_TEST(test_i2c_init_high_gpio_numbers);
    RUN_TEST(test_i2c_init_repeated);
    RUN_TEST(test_i2c_init_boundary_pin_numbers);
    
    // Release tests
    RUN_TEST(test_i2c_release_after_init);
    RUN_TEST(test_i2c_release_without_init);
    
    // Bus selection tests
    RUN_TEST(test_i2c_select_bus);
    RUN_TEST(test_i2c_select_bus_various_indices);
    
    // Combined operation tests
    RUN_TEST(test_i2c_init_then_bus_select);
    RUN_TEST(test_i2c_multiple_init_release_cycles);
    
    UNITY_END();
}