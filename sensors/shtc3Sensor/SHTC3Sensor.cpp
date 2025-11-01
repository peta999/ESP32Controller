#include "SHTC3Sensor.h"
#include <iostream>
#include <iomanip>
#include <cinttypes>

static const uint32_t kMinMeasurementIntervalMs = 100u;

extern "C" {
    #include "../../components/shtc1/shtc1.h"
    #include "../../components/shtc1/sensirion_i2c.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
}

/**
 * @brief Constructs an SHTC3Sensor with the given I2C address, power mode, and I2C pins.
 *
 * Stores the provided I2C address, low-power preference, and SCL/SDA pin numbers.
 * The stored address is retained for API compatibility but is not used for I2C operations;
 * the underlying C driver uses a fixed device address.
 *
 * @param address I2C address to store for API compatibility.
 * @param low_power Enable the sensor's low-power mode if true.
 * @param scl_pin GPIO pin number for I2C SCL.
 * @param sda_pin GPIO pin number for I2C SDA.
 */
SHTC3Sensor::SHTC3Sensor(uint8_t address, bool low_power, uint8_t scl_pin, uint8_t sda_pin)
    : address_(address), low_power_mode_(low_power), initialized_(false),
      measurement_interval_ms_(1000), measurement_callback_(nullptr), continuous_active_(false),
      measure_task_handle_(nullptr), scl_pin_(scl_pin), sda_pin_(sda_pin) {
    // Note: The underlying C library uses a hardcoded address (0x70),
    // so we store the configured address here for API compatibility but it has no effect
    // on sensor operations. All I2C operations will use the fixed C library address.
}

/**
     * @brief Construct a Builder with the specified I2C SCL and SDA pins.
     *
     * @param scl_pin I2C clock (SCL) GPIO pin number to use when building the sensor.
     * @param sda_pin I2C data (SDA) GPIO pin number to use when building the sensor.
     */
SHTC3Sensor::Builder::Builder(uint8_t scl_pin, uint8_t sda_pin)
    : scl_pin_(scl_pin), sda_pin_(sda_pin) {}

/**
 * @brief Set the I2C address to use for the sensor being built.
 *
 * Stores the provided I2C address in the builder for the resulting SHTC3Sensor instance.
 *
 * @param address I2C 7-bit address to assign to the sensor.
 * @return SHTC3Sensor::Builder& Reference to the builder to allow method chaining.
 */
SHTC3Sensor::Builder& SHTC3Sensor::Builder::setAddress(uint8_t address) {
    address_ = address;
    return *this;
}

/**
 * @brief Configure the builder to enable or disable the sensor's low-power mode.
 *
 * @param low_power If `true`, the constructed sensor will be configured for low-power operation; if `false`, normal power mode will be used.
 * @return SHTC3Sensor::Builder& Reference to this Builder to allow method chaining.
 */
SHTC3Sensor::Builder& SHTC3Sensor::Builder::setLowPower(bool low_power) {
    low_power_ = low_power;
    return *this;
}

/**
 * @brief Constructs an SHTC3Sensor configured with the builder's settings.
 *
 * @return SHTC3Sensor Configured sensor instance using the builder's address, low-power flag,
 *         SCL pin, and SDA pin.
 */
SHTC3Sensor SHTC3Sensor::Builder::build() const {
    return SHTC3Sensor(address_, low_power_, scl_pin_, sda_pin_);
}

/**
 * @brief Initialize the sensor I2C bus using the configured SCL and SDA pins.
 *
 * Initializes the underlying I2C interface with the sensor's stored SCL and SDA pin numbers.
 *
 * @return true if the I2C initialization succeeded, false otherwise.
 */
bool SHTC3Sensor::initializeBus() {
    return sensirion_i2c_init(scl_pin_, sda_pin_);
}

/**
 * @brief Attempts to probe the SHTC3 sensor on the I2C bus and apply the configured low-power mode on success.
 *
 * Sets the internal initialized flag to reflect probe outcome.
 *
 * @return `true` if the sensor was detected and initialized (low-power mode applied), `false` otherwise.
 */
bool SHTC3Sensor::probe() {
    if (shtc1_probe() == STATUS_OK) {
        setLowPowerMode(low_power_mode_);
        initialized_ = true;
        return true;
    }
    initialized_ = false;
    return false;
}

/**
 * @brief Perform a blocking temperature and humidity measurement and store the results.
 *
 * If the sensor is not initialized, the function will attempt to probe and initialize it
 * before performing the measurement. On success, the provided output parameters are
 * populated with the measured values.
 *
 * @param temperature Reference to an int32_t to receive the measured temperature.
 * @param humidity Reference to an int32_t to receive the measured humidity.
 * @return true if the measurement completed successfully and outputs were populated, false otherwise.
 */
bool SHTC3Sensor::measure(int32_t& temperature, int32_t& humidity) {
    if (!initialized_ && !probe()) {
        return false;
    }

    int8_t ret = shtc1_measure_blocking_read(&temperature, &humidity);
    return (ret == STATUS_OK);
}

/**
 * @brief Enable or disable the sensor's low-power operating mode.
 *
 * Updates the sensor's low-power setting and applies it to the underlying driver.
 *
 * @param enable `true` to enable low-power mode, `false` to disable it.
 */
void SHTC3Sensor::setLowPowerMode(bool enable) {
    low_power_mode_ = enable;
    shtc1_enable_low_power_mode(enable ? 1 : 0);
}

/**
 * @brief Gets the configured I2C address for this sensor instance.
 *
 * @return uint8_t The stored I2C address configured for this SHTC3Sensor.
 */
uint8_t SHTC3Sensor::getAddress() const {
    return address_;
}

/**
 * @brief Reads the sensor's 32-bit serial number into the provided reference.
 *
 * @param serial Reference to a uint32_t that will be set to the sensor's serial number on success.
 * @return true if the serial number was read successfully, false otherwise.
 */
bool SHTC3Sensor::readSerial(uint32_t& serial) {
    return (shtc1_read_serial(&serial) == STATUS_OK);
}

/**
 * @brief Retrieve the driver version string for the underlying SHTC1/SHTC3 implementation.
 *
 * @return const char* Pointer to a null-terminated string containing the driver version. The caller must not free or modify the returned pointer.
 */
const char* SHTC3Sensor::getDriverVersion() const {
    return shtc1_get_driver_version();
}

/**
 * @brief Transition the sensor into its low-power sleep state.
 *
 * @return true if the sleep command was accepted by the sensor, false otherwise.
 */
bool SHTC3Sensor::sleep() {
    return (shtc1_sleep() == STATUS_OK);
}

/**
 * @brief Wake the SHTC3 sensor from sleep.
 *
 * Sends a wake command to the sensor and updates its internal state via the underlying driver.
 *
 * @return `true` if the wake command succeeded, `false` otherwise.
 */
bool SHTC3Sensor::wakeUp() {
    return (shtc1_wake_up() == STATUS_OK);
}

/**
 * @brief Ensures the sensor's continuous measurement is stopped before destruction.
 *
 * Stops any active continuous measurement and performs necessary cleanup related to
 * the measurement task to leave the object in a safe state prior to destruction.
 */
SHTC3Sensor::~SHTC3Sensor() {
    stopContinuousMeasurement();
}

/**
 * @brief Set the measurement interval used for periodic (continuous) measurements.
 *
 * Updates the sensor's stored measurement interval. If `interval_ms` is zero or less than
 * `kMinMeasurementIntervalMs`, the interval is clamped to `kMinMeasurementIntervalMs`.
 *
 * @param interval_ms Desired measurement interval in milliseconds.
 */
void SHTC3Sensor::setMeasurementInterval(uint32_t interval_ms) {
    if (interval_ms == 0 || interval_ms < kMinMeasurementIntervalMs) {
        std::cout << "Invalid measurement interval: " << interval_ms << "ms, clamping to minimum: " << kMinMeasurementIntervalMs << "ms" << std::endl;
        measurement_interval_ms_ = kMinMeasurementIntervalMs;
    } else {
        measurement_interval_ms_ = interval_ms;
    }
}

/**
 * @brief Sets the callback invoked with temperature and humidity after each successful measurement.
 *
 * @param callback Function pointer called with the measured `temperature` and `humidity` (both as `int32_t` values).
 *                 Pass `nullptr` to clear/disarm the callback. The callback is invoked when a measurement succeeds
 *                 (for example during continuous measurement).
 */
void SHTC3Sensor::setMeasurementCallback(void (*callback)(int32_t temperature, int32_t humidity)) {
    measurement_callback_ = callback;
}

/**
 * @brief Start periodic continuous measurements and invoke the registered callback for each sample.
 *
 * Starts a FreeRTOS task that repeatedly performs sensor measurements at the configured interval and,
 * if a measurement callback is registered, calls that callback with each temperature and humidity pair.
 * If continuous measurement is already active this function does nothing.
 *
 * @return true if continuous measurement is already active or the measurement task was started successfully, `false` if task creation failed.
 */
bool SHTC3Sensor::startContinuousMeasurement() {
    if (continuous_active_.load()) {
        printf("Continuous measurement already active\n");
        return true;
    }

    continuous_active_.store(true);

    BaseType_t result = xTaskCreate(continuousMeasureTask, "SHTC3MeasureTask", 2048, (void*)this, 5, &measure_task_handle_
    );

    if (result == pdPASS) {
        printf("Continuous measurement task started\n");
        return true;
    } else {
        continuous_active_.store(false);
        return false;
    }
}


/**
 * @brief Stops any ongoing continuous measurement and waits for its task to finish.
 *
 * Clears the active continuous-measurement flag and blocks until the background
 * measurement task has terminated and its task handle is cleared.
 */
void SHTC3Sensor::stopContinuousMeasurement() {
    continuous_active_.store(false);
    while (measure_task_handle_ != nullptr) {
        vTaskDelay(1);
    }
}

/**
 * @brief FreeRTOS task that continuously performs sensor measurements and invokes the registered callback.
 *
 * Runs a loop until the sensor's continuous_active_ flag is cleared: each iteration performs a measurement,
 * calls the measurement callback with temperature and humidity if present and the measurement succeeded,
 * then delays for the configured measurement interval. When the loop exits, clears the task handle and deletes the task.
 *
 * @param param Pointer to the SHTC3Sensor instance (passed as void* by FreeRTOS task creation).
 */
void SHTC3Sensor::continuousMeasureTask(void* param) {
    SHTC3Sensor* sensor = (SHTC3Sensor*)param;
    while (sensor->continuous_active_.load()) {
        int32_t temperature, humidity;
        if (sensor->measure(temperature, humidity) && sensor->measurement_callback_) {
            sensor->measurement_callback_(temperature, humidity);
        }
        vTaskDelay(pdMS_TO_TICKS(sensor->measurement_interval_ms_));
    }
    sensor->measure_task_handle_ = nullptr;
    vTaskDelete(nullptr);
}