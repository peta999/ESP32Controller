#ifndef SHTC3Sensor_H_
#define SHTC3Sensor_H_

#include <cstdint>

class SHTC3Sensor {
public:
    /**
     * Builder class for constructing SHTC3Sensor instances
     */
    class Builder {
    public:
        /**
         * Constructor for Builder requiring SCL and SDA pins
         * @param scl_pin GPIO pin for SCL (required)
         * @param sda_pin GPIO pin for SDA (required)
         */
        Builder(uint8_t scl_pin, uint8_t sda_pin);

        /**
         * Set the I2C address of the sensor (currently not implemented)
         * @param address I2C address (currently ignored - all sensors use fixed address 0x70)
         * @return Reference to this builder for chaining
         * @note The underlying C driver uses a hardcoded address (0x70) and does not support
         *       runtime address configuration. This parameter is stored for future compatibility
         *       but has no effect on sensor operation at this time.
         */
        Builder& setAddress(uint8_t address);

        /**
         * Set the low power mode
         * @param low_power Enable low power mode (default false)
         * @return Reference to this builder for chaining
         */
        Builder& setLowPower(bool low_power);

        /**
         * Build and return the SHTC3Sensor instance
         * @return Constructed SHTC3Sensor object
         */
        SHTC3Sensor build() const;

    private:
        uint8_t address_ = 0x70;
        bool low_power_ = false;
        uint8_t scl_pin_;
        uint8_t sda_pin_;
    };

    /**
     * Initialize the I2C bus (global initialization, should be called once)
     * @return true if initialization successful
     */
    bool initializeBus();

    /**
     * Probe if sensor is available and responding
     * @return true if sensor detected
     */
    bool probe();

    /**
     * Perform a blocking temperature and humidity measurement
     * @param temperature Temperature result (multiplied by 1000)
     * @param humidity Humidity result (multiplied by 1000)
     * @return true if measurement successful
     */
    bool measure(int32_t& temperature, int32_t& humidity);

    /**
     * Enable or disable low power mode
     * @param enable true to enable low power mode
     */
    void setLowPowerMode(bool enable);

    /**
     * Get the configured I2C address (note: address setting is currently ignored)
     * @return sensor address (as configured, though not used by sensor operations)
     * @note Returns the address that was set during construction, but all I2C operations
     *       use the fixed address 0x70 due to C library limitations.
     */
    uint8_t getAddress() const;

    /**
     * Read sensor serial number
     * @param serial Serial number output
     * @return true if read successful
     */
    bool readSerial(uint32_t& serial);

    /**
     * Get driver version string
     * @return version string
     */
    const char* getDriverVersion() const;

   /**
    * Send sensor to sleep (SHTC3 only)
    * @return true if command successful
    */
   bool sleep();

   /**
    * Wake sensor from sleep (SHTC3 only)
    * @return true if command successful
    */
   bool wakeUp();

   /**
    * Set the measurement interval for continuous measurements (in milliseconds)
    * @param interval_ms Interval between measurements
    */
   void setMeasurementInterval(uint32_t interval_ms);

   /**
    * Set the callback function for measurement results
    * @param callback Function pointer for handling temperature and humidity data
    */
   void setMeasurementCallback(void (*callback)(int32_t temperature, int32_t humidity));

   /**
    * Start continuous periodic measurements based on the set interval
    */
   void startContinuousMeasurement();

   /**
    * Stop continuous measurements
    */
   void stopContinuousMeasurement();

private:
private:
    /**
     * Constructor for SHTC3 sensor instance
     * @param address I2C address of the sensor (default 0x70)
     * @param low_power Enable low power mode (default false)
     * @param scl_pin GPIO pin for SCL (default 27)
     * @param sda_pin GPIO pin for SDA (default 26)
     */
    SHTC3Sensor(uint8_t address = 0x70, bool low_power = false, uint8_t scl_pin = 27, uint8_t sda_pin = 26);

    uint8_t address_;       // I2C address
    bool low_power_mode_;   // Low power mode setting
    bool initialized_;      // Sensor probed and ready
    uint32_t measurement_interval_ms_;  // Measurement interval in ms
    void (*measurement_callback_)(int32_t, int32_t);  // Callback for measurements
    bool continuous_active_;  // Flag for continuous measurement state
    uint8_t scl_pin_;       // SCL GPIO pin
    uint8_t sda_pin_;       // SDA GPIO pin

    static void continuousMeasureTask(void* param);
};

#endif // SHTC3Sensor_H_
