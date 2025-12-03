/**
  ******************************************************************************
  * @file    flight_manager.h
  * @brief   Flight Manager header - Phase 4 refactoring
  * @note    Top-level system coordination and timing management
  ******************************************************************************
  */

#ifndef FLIGHT_MANAGER_H
#define FLIGHT_MANAGER_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* Module includes */
#include "sensor_system.h"
#include "navigation_manager.h"
#include "rocket_telemetry.h"
#include "gps_module.h"
#include "ble_module.h"

/**
 * @brief Flight system status structure
 */
typedef struct {
    bool sensors_healthy;
    bool navigation_healthy;
    bool telemetry_healthy;
    bool gps_healthy;
    bool ble_healthy;
    uint32_t system_uptime;
    uint32_t error_count;
    uint32_t sensor_update_count;
    uint32_t telemetry_update_count;
    uint32_t gps_update_count;
} FlightSystemStatus_t;

/**
 * @brief Timing configuration constants
 */
#define FLIGHT_SENSOR_INTERVAL_MS    2      /* 500Hz sensor updates */
#define FLIGHT_TELEMETRY_INTERVAL_MS 200    /* 5Hz telemetry */
#define FLIGHT_GPS_INTERVAL_MS       5000   /* 0.2Hz GPS updates */
#define FLIGHT_DEBUG_INTERVAL_MS     10     /* 100Hz debug output */

/**
 * @brief Flight Manager API
 */

/**
 * @brief Initialize flight management system
 * @note Call after hardware initialization but before main loop
 * @return true if initialization successful, false otherwise
 */
bool FlightManager_Init(void);

/**
 * @brief Update flight management system - call from main loop
 * @note Handles all timing and module coordination
 */
void FlightManager_Update(void);

/**
 * @brief Get current system status
 * @return FlightSystemStatus_t Current system health and statistics
 */
FlightSystemStatus_t FlightManager_GetStatus(void);

/**
 * @brief Check if system is ready for operation
 * @return true if all critical systems operational
 */
bool FlightManager_IsReady(void);

/**
 * @brief Get system uptime in milliseconds
 * @return uint32_t System uptime since initialization
 */
uint32_t FlightManager_GetUptime(void);

/**
 * @brief Force telemetry transmission (for testing)
 * @return true if transmission initiated successfully
 */
bool FlightManager_ForceTelemetry(void);

/**
 * @brief Get formatted status string for debugging
 * @param buffer Buffer to write status string
 * @param buffer_size Size of buffer
 */
void FlightManager_GetStatusString(char* buffer, size_t buffer_size);

#endif /* FLIGHT_MANAGER_H */
