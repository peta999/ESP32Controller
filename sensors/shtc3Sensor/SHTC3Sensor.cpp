#include "SHTC3Sensor.h"
#include <iostream>
#include <iomanip>
#include <cinttypes>

extern "C" {
    #include "../../components/shtc1/shtc1.h"
    #include "../../components/shtc1/sensirion_i2c.h"
}

SHTC3Sensor::SHTC3Sensor(uint8_t address, bool low_power)
    : address_(address), low_power_mode_(low_power), initialized_(false) {
    // Note: The underlying C library uses a hardcoded address,
    // so we store it here for future extensibility but currently
    // all instances will use the global SHTC1_ADDRESS
}

bool SHTC3Sensor::initializeBus() {
    sensirion_i2c_init();
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

const char* SHTC3Sensor::getDriverVersion() {
    return shtc1_get_driver_version();
}

bool SHTC3Sensor::sleep() {
    return (shtc1_sleep() == STATUS_OK);
}

bool SHTC3Sensor::wakeUp() {
    return (shtc1_wake_up() == STATUS_OK);
}
