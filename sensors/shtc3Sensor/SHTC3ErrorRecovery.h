#ifndef SHTC3_ERROR_RECOVERY_H_
#define SHTC3_ERROR_RECOVERY_H_

#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

class SHTC3ErrorRecovery {
public:
    /**
     * Constructor for error recovery manager
     */
    SHTC3ErrorRecovery();

    /**
     * Record a measurement failure
     */
    void recordFailure();

    /**
     * Check if reconnection should be attempted based on failure count
     * @return true if reconnection should be attempted
     */
    bool shouldAttemptReconnection() const;

    /**
     * Reset all failure counters after successful operation
     */
    void resetCounters();

    /**
     * Get current consecutive failure count
     * @return current consecutive failure count
     */
    uint32_t getConsecutiveFailures() const;

    /**
     * Get current reconnection attempt count
     * @return current reconnection attempt count
     */
    uint32_t getReconnectionAttempts() const;

    /**
     * Calculate backoff delay for reconnection attempts
     * @return backoff delay in milliseconds
     */
    uint32_t calculateBackoffDelay() const;

    /**
     * Check if minimum time has passed since last reconnection attempt
     * @return true if minimum time has passed
     */
    bool canAttemptReconnection() const;

    /**
     * Record a reconnection attempt
     */
    void recordReconnectionAttempt();

    /**
     * Set the minimum time between reconnection attempts
     * @param min_interval_ms minimum interval in milliseconds
     */
    void setMinReconnectionInterval(uint32_t min_interval_ms);

private:
    uint32_t consecutive_failures_;
    uint32_t reconnection_attempts_;
    uint32_t last_reconnection_time_;
    uint32_t min_reconnection_interval_ms_;

    static constexpr uint32_t MAX_CONSECUTIVE_FAILURES = 2;
    static constexpr uint32_t DEFAULT_MIN_RECONNECTION_INTERVAL_MS = 10000;
    static constexpr uint32_t MAX_BACKOFF_DELAY_MS = 2000;
};

#endif // SHTC3_ERROR_RECOVERY_H_
