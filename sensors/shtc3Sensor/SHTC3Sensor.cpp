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

SHTC3Sensor::SHTC3Sensor(uint8_t address, bool low_power, uint8_t scl_pin, uint8_t sda_pin)
    : address_(address), low_power_mode_(low_power), initialized_(false),
      measurement_interval_ms_(1000), measurement_callback_(nullptr), continuous_active_(false),
      measure_task_handle_(nullptr), scl_pin_(scl_pin), sda_pin_(sda_pin) {
    // Note: The underlying C library uses a hardcoded address (0x70),
    // so we store the configured address here for API compatibility but it has no effect
    // on sensor operations. All I2C operations will use the fixed C library address.
}

// Builder class implementations
SHTC3Sensor::Builder::Builder(uint8_t scl_pin, uint8_t sda_pin)
    : scl_pin_(scl_pin), sda_pin_(sda_pin) {}

SHTC3Sensor::Builder& SHTC3Sensor::Builder::setAddress(uint8_t address) {
    address_ = address;
    return *this;
}

SHTC3Sensor::Builder& SHTC3Sensor::Builder::setLowPower(bool low_power) {
    low_power_ = low_power;
    return *this;
}

SHTC3Sensor SHTC3Sensor::Builder::build() const {
    return SHTC3Sensor(address_, low_power_, scl_pin_, sda_pin_);
}

bool SHTC3Sensor::initializeBus() {
    sensirion_i2c_init(scl_pin_, sda_pin_);
    return true; // I2C init doesn't return error codes
}

bool SHTC3Sensor::probe() {
    if (shtc1_probe() == STATUS_OK) {
        setLowPowerMode(low_power_mode_); // Apply current configuration
        initialized_ = true;
        return true;
    }
    initialized_ = false;
    return false;
}

bool SHTC3Sensor::measure(int32_t& temperature, int32_t& humidity) {
    if (!initialized_ && !probe()) {
        return false;
    }

    int8_t ret = shtc1_measure_blocking_read(&temperature, &humidity);
    return (ret == STATUS_OK);
}

void SHTC3Sensor::setLowPowerMode(bool enable) {
    low_power_mode_ = enable;
    shtc1_enable_low_power_mode(enable ? 1 : 0);
}

uint8_t SHTC3Sensor::getAddress() const {
    return address_;
}

bool SHTC3Sensor::readSerial(uint32_t& serial) {
    return (shtc1_read_serial(&serial) == STATUS_OK);
}

const char* SHTC3Sensor::getDriverVersion() const {
    return shtc1_get_driver_version();
}

bool SHTC3Sensor::sleep() {
    return (shtc1_sleep() == STATUS_OK);
}

bool SHTC3Sensor::wakeUp() {
    return (shtc1_wake_up() == STATUS_OK);
}

SHTC3Sensor::~SHTC3Sensor() {
    stopContinuousMeasurement();
}

void SHTC3Sensor::setMeasurementInterval(uint32_t interval_ms) {
    if (interval_ms == 0 || interval_ms < kMinMeasurementIntervalMs) {
        std::cout << "Invalid measurement interval: " << interval_ms << "ms, clamping to minimum: " << kMinMeasurementIntervalMs << "ms" << std::endl;
        measurement_interval_ms_ = kMinMeasurementIntervalMs;
    } else {
        measurement_interval_ms_ = interval_ms;
    }
}

void SHTC3Sensor::setMeasurementCallback(void (*callback)(int32_t temperature, int32_t humidity)) {
    measurement_callback_ = callback;
}

void SHTC3Sensor::startContinuousMeasurement() {
    if (continuous_active_.load()) return;
    continuous_active_.store(true);
    xTaskCreate(continuousMeasureTask, "SHTC3MeasureTask", 2048, (void*)this, 5, (TaskHandle_t*)&measure_task_handle_);
}

void SHTC3Sensor::stopContinuousMeasurement() {
    continuous_active_.store(false);
    while (measure_task_handle_ != nullptr) {
        vTaskDelay(1);
    }
}

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
