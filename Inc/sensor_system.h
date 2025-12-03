/**
  ******************************************************************************
  * @file    sensor_system.h
  * @brief   Raw sensor I/O system - Phase 1 refactoring
  * @note    Handles only raw sensor reading, no AHRS or navigation processing
  ******************************************************************************
  */

#ifndef SENSOR_SYSTEM_H_
#define SENSOR_SYSTEM_H_

#include "main.h"
#include "BMI088a.h"
#include "BMI088g.h"
#include "BMP390.h"
#include "MLX90393.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Clean sensor data structure - post-calibration, validated data
 */
typedef struct {
    // Raw sensor data (calibrated and converted to standard units)
    float accel[3];      // m/s² [X, Y, Z] - Body frame
    float gyro[3];       // rad/s [X, Y, Z] - Body frame
    float mag[3];        // µT [X, Y, Z] - Body frame
    float pressure;      // Pa - Absolute pressure
    float temperature;   // °C - Ambient temperature

    // Metadata
    uint32_t timestamp;  // HAL_GetTick() when data was read
    bool valid;          // All critical sensors read successfully

    // Individual sensor status (for diagnostics)
    bool accel_valid;
    bool gyro_valid;
    bool mag_valid;
    bool baro_valid;
} SensorData_t;

/**
 * @brief Sensor system health and diagnostics
 */
typedef struct {
    uint32_t total_reads;
    uint32_t error_count;
    uint32_t last_successful_read;
    bool healthy;
    float error_rate_percent;
} SensorSystemDiagnostics_t;

/**
 * @brief Initialize all sensors
 * @retval true if initialization successful (may continue with some sensor failures)
 * @retval false if critical failure prevents operation
 */
bool SensorSystem_Init(void);

/**
 * @brief Read data from all sensors - FAST execution for 500Hz operation
 * @param data Pointer to SensorData_t structure to fill
 * @retval true if read successful (all critical sensors)
 * @retval false if read failed or data invalid
 * @note This function must execute quickly for 500Hz operation
 */
bool SensorSystem_Read(SensorData_t* data);

/**
 * @brief Check if sensor system is healthy
 * @retval true if system is operating normally
 * @retval false if degraded performance or failures detected
 */
bool SensorSystem_IsHealthy(void);

/**
 * @brief Get detailed diagnostics information
 * @param diag Pointer to diagnostics structure to fill
 */
void SensorSystem_GetDiagnostics(SensorSystemDiagnostics_t* diag);

/**
 * @brief Get formatted diagnostics string for debug output
 * @param buffer Buffer to write diagnostics string
 * @param buffer_size Size of the buffer
 */
void SensorSystem_GetDiagnosticsString(char* buffer, size_t buffer_size);

/**
 * @brief Reset error counters and statistics
 */
void SensorSystem_ResetDiagnostics(void);

#endif /* SENSOR_SYSTEM_H_ */
