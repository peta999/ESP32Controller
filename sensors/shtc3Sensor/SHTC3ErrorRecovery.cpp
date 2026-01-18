#include "SHTC3ErrorRecovery.h"
#include "esp_log.h"

static const char* TAG = "SHTC3ErrorRecovery";

SHTC3ErrorRecovery::SHTC3ErrorRecovery()
    : consecutive_failures_(0),
      reconnection_attempts_(0),
      last_reconnection_time_(0),
      min_reconnection_interval_ms_(DEFAULT_MIN_RECONNECTION_INTERVAL_MS) {
}

void SHTC3ErrorRecovery::recordFailure() {
    consecutive_failures_++;
    ESP_LOGW(TAG, "Measurement failure recorded (consecutive: %u)", consecutive_failures_);
}

bool SHTC3ErrorRecovery::shouldAttemptReconnection() const {
    if (consecutive_failures_ < MAX_CONSECUTIVE_FAILURES) {
        return false;
    }
    return canAttemptReconnection();
}

void SHTC3ErrorRecovery::resetCounters() {
    consecutive_failures_ = 0;
    reconnection_attempts_ = 0;
    last_reconnection_time_ = 0;
    ESP_LOGI(TAG, "Error counters reset");
}

uint32_t SHTC3ErrorRecovery::getConsecutiveFailures() const {
    return consecutive_failures_;
}

uint32_t SHTC3ErrorRecovery::getReconnectionAttempts() const {
    return reconnection_attempts_;
}

uint32_t SHTC3ErrorRecovery::calculateBackoffDelay() const {
    uint32_t backoff_delay = 500 * (1 << reconnection_attempts_);
    if (backoff_delay > MAX_BACKOFF_DELAY_MS) {
        backoff_delay = MAX_BACKOFF_DELAY_MS;
    }
    ESP_LOGW(TAG, "Calculated backoff delay: %u ms (capped to prevent watchdog timeout)", backoff_delay);
    return backoff_delay;
}

bool SHTC3ErrorRecovery::canAttemptReconnection() const {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if ((current_time - last_reconnection_time_) < min_reconnection_interval_ms_) {
        ESP_LOGI(TAG, "Too soon since last reconnection attempt");
        return false;
    }
    return true;
}

void SHTC3ErrorRecovery::recordReconnectionAttempt() {
    reconnection_attempts_++;
    last_reconnection_time_ = xTaskGetTickCount() * portTICK_PERIOD_MS;
    ESP_LOGI(TAG, "Reconnection attempt recorded (attempt %u)", reconnection_attempts_);
}

void SHTC3ErrorRecovery::setMinReconnectionInterval(uint32_t min_interval_ms) {
    min_reconnection_interval_ms_ = min_interval_ms;
    ESP_LOGI(TAG, "Minimum reconnection interval set to %u ms", min_reconnection_interval_ms_);
}
