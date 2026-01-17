#include "SHTC3Sensor.h"
#include <iomanip>
#include <cinttypes>
#include "esp_log.h"

static const char* TAG = "SHTC3Sensor";
static const uint32_t kMinMeasurementIntervalMs = 100u;

namespace {
constexpr uint16_t kShtc3TaskStackWords = 2048;
constexpr UBaseType_t kShtc3TaskPriority = 5;
} // namespace

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
      measure_task_handle_(nullptr), scl_pin_(scl_pin), sda_pin_(sda_pin),
      consecutive_failures_(0), reconnection_attempts_(0), last_reconnection_time_(0) {
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
 * Includes I2C bus error recovery - if measurement fails, attempts bus recovery before
 * reporting failure.
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
    if (ret == STATUS_OK) {
        return true;
    }

    // Measurement failed - attempt I2C bus recovery
    ESP_LOGW(TAG, "Measurement failed, attempting I2C bus recovery");
    if (recoverI2CBus()) {
        // Try measurement again after bus recovery
        ret = shtc1_measure_blocking_read(&temperature, &humidity);
        if (ret == STATUS_OK) {
            ESP_LOGI(TAG, "Measurement successful after bus recovery");
            return true;
        }
    }

    return false;
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
        ESP_LOGW(TAG, "Invalid measurement interval: %u ms, clamping to minimum: %u ms", interval_ms, kMinMeasurementIntervalMs);
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
    bool expected = false;
    if (!continuous_active_.compare_exchange_strong(expected, true)) {
        ESP_LOGI(TAG, "Continuous measurement already active");
        return true;
    }

    BaseType_t result = xTaskCreate(continuousMeasureTask, "SHTC3MeasureTask",
                                    kShtc3TaskStackWords, this, kShtc3TaskPriority,
                                    &measure_task_handle_);

    if (result == pdPASS) {
        ESP_LOGI(TAG, "Continuous measurement task started");
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
 * measurement task has terminated and its task handle is cleared. If the task
 * doesn't exit gracefully within a timeout, it is explicitly deleted.
 */
void SHTC3Sensor::stopContinuousMeasurement() {
    continuous_active_.store(false);

    // Wait for task to exit gracefully with timeout
    const TickType_t kMaxWaitTicks = pdMS_TO_TICKS(5000); // 5 second timeout
    TickType_t startTicks = xTaskGetTickCount();

    while (measure_task_handle_ != nullptr) {
        vTaskDelay(pdMS_TO_TICKS(10)); // Check every 10ms

        // Check for timeout
        if ((xTaskGetTickCount() - startTicks) >= kMaxWaitTicks) {
            ESP_LOGW(TAG, "Continuous measurement task did not exit gracefully, forcing deletion");

            // Suspend all tasks to prevent race conditions during task deletion
            vTaskSuspendAll();
            if (measure_task_handle_ != nullptr) {
                vTaskDelete(measure_task_handle_);
                measure_task_handle_ = nullptr;
            }
            xTaskResumeAll();

            break;
        }
    }

    // If task exited gracefully, remove it from watchdog
    if (measure_task_handle_ == nullptr) {
        // Task handle is cleared by the task itself, so we can't remove it from watchdog here
        // The task should have already been removed from watchdog before exiting
    }
}

/**
 * @brief Attempt to reconnect to the sensor after failures
 *
 * @return true if reconnection successful
 */
bool SHTC3Sensor::reconnectSensor() {
    ESP_LOGI(TAG, "Attempting to reconnect to sensor (attempt %u)", reconnection_attempts_ + 1);

    // Reset I2C bus (this is a simple approach - in production you might want more sophisticated bus recovery)
    sensirion_i2c_release();
    vTaskDelay(pdMS_TO_TICKS(100)); // Brief delay before reinit
    if (!sensirion_i2c_init(scl_pin_, sda_pin_)) {
        ESP_LOGE(TAG, "Failed to reinitialize I2C bus during reconnection");
        return false;
    }

    // Try to probe the sensor
    if (probe()) {
        ESP_LOGI(TAG, "Sensor reconnection successful");
        resetFailureCounters();
        return true;
    }

    ESP_LOGW(TAG, "Sensor reconnection failed");
    reconnection_attempts_++;
    last_reconnection_time_ = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return false;
}

/**
 * @brief Check if sensor needs reconnection based on failure count
 *
 * @return true if reconnection should be attempted
 */
bool SHTC3Sensor::shouldAttemptReconnection() {
    if (consecutive_failures_ < MAX_CONSECUTIVE_FAILURES) {
        return false;
    }

    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if ((current_time - last_reconnection_time_) < MIN_RECONNECTION_INTERVAL_MS) {
        return false; // Too soon since last attempt
    }

    return true;
}

/**
 * @brief Reset failure counters after successful operation
 */
void SHTC3Sensor::resetFailureCounters() {
    consecutive_failures_ = 0;
    reconnection_attempts_ = 0;
    last_reconnection_time_ = 0;
}

/**
 * Record a measurement failure and update counters
 */
void SHTC3Sensor::recordMeasurementFailure() {
    consecutive_failures_++;
    ESP_LOGW(TAG, "Measurement failure recorded (consecutive: %u)", consecutive_failures_);
}

/**
 * Attempt to recover from I2C bus errors by resetting the bus
 * @return true if bus recovery successful
 */
bool SHTC3Sensor::recoverI2CBus() {
    ESP_LOGW(TAG, "Attempting I2C bus recovery");

    // Release current I2C driver
    sensirion_i2c_release();

    // Brief delay before reinit
    vTaskDelay(pdMS_TO_TICKS(50));

    // Reinitialize I2C bus
    if (!sensirion_i2c_init(scl_pin_, sda_pin_)) {
        ESP_LOGE(TAG, "Failed to reinitialize I2C bus during recovery");
        return false;
    }

    ESP_LOGI(TAG, "I2C bus recovery successful");
    return true;
}

/**
 * @brief FreeRTOS task that continuously performs sensor measurements and invokes the registered callback.
 *
 * Runs a loop until the sensor's continuous_active_ flag is cleared: each iteration performs a measurement,
 * calls the measurement callback with temperature and humidity if present and the measurement succeeded,
 * then delays for the configured measurement interval. When the loop exits, clears the task handle and deletes the task.
 *
 * Includes error handling and recovery mechanisms to prevent watchdog timeouts when sensor disconnects.
 *
 * @param param Pointer to the SHTC3Sensor instance (passed as void* by FreeRTOS task creation).
 */
void SHTC3Sensor::continuousMeasureTask(void* param) {
    SHTC3Sensor* sensor = (SHTC3Sensor*)param;

    while (sensor->continuous_active_.load()) {
        // Check if we need to attempt reconnection
        if (sensor->shouldAttemptReconnection()) {
            if (!sensor->reconnectSensor()) {
                // Reconnection failed, use exponential backoff (capped to prevent watchdog timeout)
                uint32_t backoff_delay = 500 * (1 << sensor->reconnection_attempts_); // Start with 500ms base
                if (backoff_delay > 3000) { // Cap at 3 seconds to stay under 5-second watchdog timeout
                    backoff_delay = 3000;
                }
                ESP_LOGW(TAG, "Reconnection failed, backing off for %u ms", backoff_delay);
                vTaskDelay(pdMS_TO_TICKS(backoff_delay));
                continue;
            }
        }

        int32_t temperature, humidity;
        bool measurement_success = sensor->measure(temperature, humidity);

        if (measurement_success) {
            sensor->resetFailureCounters();
            if (sensor->measurement_callback_) {
                sensor->measurement_callback_(temperature, humidity);
            }
        } else {
            sensor->recordMeasurementFailure();
        }

        vTaskDelay(pdMS_TO_TICKS(sensor->measurement_interval_ms_));
    }

    sensor->measure_task_handle_ = nullptr;
    vTaskDelete(nullptr);
}
