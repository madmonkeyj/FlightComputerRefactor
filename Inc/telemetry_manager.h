/**
  ******************************************************************************
  * @file    telemetry_manager.h
  * @brief   Telemetry Manager - Data transmission coordination (Phase 3)
  * @note    Handles packet creation and LoRa transmission management
  ******************************************************************************
  */

#ifndef TELEMETRY_MANAGER_H_
#define TELEMETRY_MANAGER_H_

#include "main.h"
#include "sensor_system.h"      // For SensorData_t
#include "navigation_manager.h" // For NavigationSolution_t
#include "gps_module.h"         // For GPS_Data_t
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Telemetry packet types
 */
typedef enum {
    TELEMETRY_TYPE_ROCKET = 0,      // High-priority rocket telemetry
    TELEMETRY_TYPE_DEBUG = 1,       // Debug data transmission
    TELEMETRY_TYPE_HEALTH = 2       // System health and diagnostics
} TelemetryType_t;

/**
 * @brief Telemetry transmission priority
 */
typedef enum {
    TELEMETRY_PRIORITY_LOW = 0,     // Best effort transmission
    TELEMETRY_PRIORITY_NORMAL = 1,  // Standard priority
    TELEMETRY_PRIORITY_HIGH = 2,    // Critical data
    TELEMETRY_PRIORITY_EMERGENCY = 3 // Emergency data - force transmission
} TelemetryPriority_t;

/**
 * @brief Telemetry transmission request
 */
typedef struct {
    TelemetryType_t type;           // Type of telemetry data
    TelemetryPriority_t priority;   // Transmission priority
    uint8_t data[256];              // Pointer to data buffer
    uint16_t data_length;           // Length of data in bytes
    uint32_t max_age_ms;            // Maximum age before data is stale
    uint32_t timestamp;             // When data was created
    bool requires_ack;              // Whether acknowledgment is needed
} TelemetryRequest_t;

/**
 * @brief Telemetry system health and statistics
 */
typedef struct {
    bool lora_healthy;              // LoRa module operational
    bool transmission_active;       // Currently transmitting

    uint32_t total_requests;        // Total transmission requests
    uint32_t successful_transmissions; // Successful transmissions
    uint32_t failed_transmissions;  // Failed transmissions
    uint32_t timeout_transmissions; // Timed-out transmissions

    uint32_t last_transmission_time; // Time of last transmission
    uint32_t average_transmission_time; // Average transmission duration

    float success_rate_percent;     // Transmission success rate
    float current_data_rate;        // Current transmission rate (Hz)

    uint32_t queue_depth;           // Current transmission queue depth
    uint32_t max_queue_depth;       // Maximum queue depth reached

    bool rate_limited;              // Currently rate limiting
    uint32_t rate_limit_drops;      // Packets dropped due to rate limiting

} TelemetryDiagnostics_t;

/**
 * @brief Telemetry transmission modes
 */
typedef enum {
    TELEMETRY_MODE_NORMAL = 0,      // Normal operation
    TELEMETRY_MODE_LOW_POWER = 1,   // Reduced transmission rate
    TELEMETRY_MODE_EMERGENCY = 2,   // Emergency mode - critical data only
    TELEMETRY_MODE_SILENT = 3       // No transmissions (testing mode)
} TelemetryMode_t;

/**
 * @brief Initialize the Telemetry Manager
 * @retval true if initialization successful
 * @retval false if critical failure
 */
bool TelemetryManager_Init(void);

/**
 * @brief Update telemetry system - call regularly from main loop
 * @note Handles transmission queue processing and LoRa management
 */
void TelemetryManager_Update(void);

/**
 * @brief Send rocket telemetry data (high priority)
 * @param nav_solution Pointer to current navigation solution
 * @param sensor_data Pointer to current sensor data
 * @param gps_data Pointer to current GPS data
 * @retval true if transmission queued successfully
 * @retval false if transmission failed or queue full
 */
bool TelemetryManager_SendRocketTelemetry(const NavigationSolution_t* nav_solution,
                                         const SensorData_t* sensor_data,
                                         const GPS_Data_t* gps_data);

/**
 * @brief Send debug data (normal priority)
 * @param debug_string Pointer to debug string
 * @param string_length Length of debug string
 * @retval true if transmission queued successfully
 * @retval false if transmission failed or queue full
 */
bool TelemetryManager_SendDebugData(const char* debug_string, uint16_t string_length);

/**
 * @brief Send system health data (normal priority)
 * @param nav_solution Pointer to navigation solution
 * @param sensor_data Pointer to sensor data
 * @retval true if transmission queued successfully
 * @retval false if transmission failed
 */
bool TelemetryManager_SendHealthData(const NavigationSolution_t* nav_solution,
                                    const SensorData_t* sensor_data);

/**
 * @brief Send custom data packet
 * @param request Pointer to telemetry request structure
 * @retval true if transmission queued successfully
 * @retval false if transmission failed or invalid request
 */
bool TelemetryManager_SendCustomData(const TelemetryRequest_t* request);

/**
 * @brief Check if telemetry system is ready for transmission
 * @retval true if ready to accept new transmissions
 * @retval false if busy or not ready
 */
bool TelemetryManager_IsReady(void);

/**
 * @brief Check if LoRa transmission is currently active
 * @retval true if transmission in progress
 * @retval false if idle
 */
bool TelemetryManager_IsTransmitting(void);

/**
 * @brief Set telemetry transmission mode
 * @param mode New telemetry mode
 */
void TelemetryManager_SetMode(TelemetryMode_t mode);

/**
 * @brief Get current telemetry mode
 * @retval Current telemetry mode
 */
TelemetryMode_t TelemetryManager_GetMode(void);

/**
 * @brief Set transmission rate limit (packets per second)
 * @param max_rate_hz Maximum transmission rate in Hz
 */
void TelemetryManager_SetRateLimit(float max_rate_hz);

/**
 * @brief Get telemetry system diagnostics
 * @param diagnostics Pointer to diagnostics structure to fill
 */
void TelemetryManager_GetDiagnostics(TelemetryDiagnostics_t* diagnostics);

/**
 * @brief Get formatted diagnostics string for debug output
 * @param buffer Buffer to write diagnostics
 * @param buffer_size Size of buffer
 */
void TelemetryManager_GetDiagnosticsString(char* buffer, size_t buffer_size);

/**
 * @brief Check if telemetry system is healthy
 * @retval true if system operating normally
 * @retval false if degraded performance detected
 */
bool TelemetryManager_IsHealthy(void);

/**
 * @brief Reset telemetry statistics and error counters
 */
void TelemetryManager_ResetStatistics(void);

/**
 * @brief Emergency transmission - bypasses queue and rate limiting
 * @param data Pointer to emergency data
 * @param data_length Length of emergency data
 * @retval true if emergency transmission initiated
 * @retval false if transmission failed
 * @note Use sparingly - can disrupt normal telemetry flow
 */
bool TelemetryManager_EmergencyTransmission(const uint8_t* data, uint16_t data_length);

/**
 * @brief Clear all pending transmissions from queue
 */
void TelemetryManager_ClearQueue(void);

/**
 * @brief Get current queue status
 * @param queue_depth Pointer to store current queue depth
 * @param max_depth Pointer to store maximum queue capacity
 * @retval true if queue information retrieved
 * @retval false if invalid parameters
 */
bool TelemetryManager_GetQueueStatus(uint32_t* queue_depth, uint32_t* max_depth);

/**
 * @brief Enable or disable automatic rocket telemetry
 * @param enable true to enable automatic transmission, false to disable
 * @param interval_ms Transmission interval in milliseconds (if enabled)
 */
void TelemetryManager_SetAutoTelemetry(bool enable, uint32_t interval_ms);

/**
 * @brief LoRa transmission completion callback - call from UART interrupt
 * @param success true if transmission completed successfully
 * @note This function should be called from the LoRa UART completion interrupt
 */
void TelemetryManager_TransmissionComplete(bool success);

#endif /* TELEMETRY_MANAGER_H_ */
