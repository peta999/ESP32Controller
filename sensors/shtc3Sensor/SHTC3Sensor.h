#ifndef SHTC3Sensor_H_
#define SHTC3Sensor_H_

#include <cstdint>

class SHTC3Sensor {
public:
    /**
     * Constructor for SHTC1 sensor instance
     * @param address I2C address of the sensor (default 0x70)
     * @param low_power Enable low power mode (default false)
     */
    SHTC3Sensor(uint8_t address = 0x70, bool low_power = false);

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
     * Get the configured I2C address
     * @return sensor address
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
    const char* getDriverVersion();

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

private:
    uint8_t address_;       // I2C address
    bool low_power_mode_;   // Low power mode setting
    bool initialized_;      // Sensor probed and ready
};

#endif // SHTC3Sensor_H_
