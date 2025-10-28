# SHTC3 Sensor Test Suite

This directory contains comprehensive unit tests for the SHTC3Sensor C++ wrapper class and the modified sensirion_i2c C functions.

## Test Coverage

### test_shtc3_sensor.cpp

Tests for the SHTC3Sensor C++ class including:
- **Builder Pattern**: Construction, method chaining, parameter validation
- **Sensor Operations**: Initialization, probing, measurements
- **Configuration**: Low-power mode, measurement intervals, I2C address
- **Continuous Measurements**: Start/stop, callbacks, task management
- **Sleep/Wake**: SHTC3-specific power management
- **Edge Cases**: Boundary values, error conditions, resource cleanup
- **Thread Safety**: Atomic operations, task synchronization

### test_sensirion_i2c.c

Tests for modified I2C initialization functions:
- **Pin Configuration**: Various GPIO combinations, boundary values
- **Initialization**: Success/failure handling, repeated initialization
- **Resource Management**: Proper cleanup, multiple init/release cycles
- **Bus Selection**: Single and multi-bus scenarios

## Building and Running Tests

### Option 1: Build as separate test application

```bash
cd /path/to/project/test
idf.py build
idf.py flash monitor
```

### Option 2: Run with Unity test runner (if configured)

```bash
idf.py build
idf.py flash
idf.py monitor
```

### Option 3: Integration with main application

Temporarily modify main/CMakeLists.txt to include test component:

```cmake
idf_component_register(SRCS "main.cpp"
                       PRIV_REQUIRES freertos test
                       REQUIRES shtc3Sensor
                       INCLUDE_DIRS "")
```

## Test Philosophy

These tests follow several key principles:

1. **No Hardware Required**: Most tests can run without actual sensor hardware, testing the software logic and API contracts
2. **Comprehensive Coverage**: Tests cover happy paths, edge cases, boundary conditions, and error scenarios
3. **Resource Safety**: Tests verify proper resource cleanup and lifecycle management
4. **Thread Safety**: Tests for FreeRTOS task interactions and atomic operations
5. **API Validation**: Tests ensure public interfaces behave correctly under all conditions

## Notes

- Tests are designed to run on ESP32 with FreeRTOS
- Some tests will report failures without actual sensor hardware (this is expected)
- Continuous measurement tests verify task creation and lifecycle without requiring sensor responses
- Pin configuration tests validate parameter handling without requiring specific hardware configurations

## Expected Results Without Hardware

When run without connected sensor hardware:
- Builder and configuration tests: **PASS**
- I2C initialization tests: **PASS** (driver initialization succeeds)
- Sensor probing tests: **FAIL** (no sensor responds - expected)
- Measurement tests: **FAIL** (no sensor data - expected)
- Continuous measurement task tests: **PASS** (task lifecycle works)
- Sleep/wake tests: **FAIL** (no sensor responds - expected)

## Adding New Tests

To add new tests:

1. Add test function following Unity conventions:

   ```c
   void test_my_new_feature(void) {
       // Arrange
       // Act
       // Assert
       TEST_ASSERT_...
   }
   ```

2. Register test in app_main():

   ```c
   RUN_TEST(test_my_new_feature);
   ```

3. Follow naming convention: `test_<component>_<scenario>`
4. Include descriptive comments explaining what is being tested
5. Use setUp/tearDown for test isolation