/**
 ******************************************************************************
 * @file    sensor_adapter.h
 * @brief   Adapter to bridge sensor_manager with navigation_manager
 * @note    Converts between hardware-specific sensor_manager format and
 *          generic SensorData_t format expected by navigation modules
 ******************************************************************************
 */

#ifndef SENSOR_ADAPTER_H
#define SENSOR_ADAPTER_H

#include "main.h"
#include "sensor_manager.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Unified sensor data structure for navigation modules
 * @note This matches the format expected by navigation_manager
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
} NavSensorData_t;

/**
 * @brief Compatibility typedef for legacy code (TelemetryManager, etc.)
 * @note NavSensorData_t and SensorData_t are identical structures
 */
typedef NavSensorData_t SensorData_t;

/**
 * @brief Convert sensor_manager scaled data to navigation format
 * @param scaled_data Input from SensorManager_GetScaledData()
 * @param raw_data Input from SensorManager_ReadRaw() for validity flags
 * @param nav_data Output in format for navigation_manager
 * @return true if conversion successful, false if invalid data
 *
 * @note Unit conversions performed:
 *       - Accel: g → m/s² (×9.80665)
 *       - Gyro: deg/s → rad/s (×π/180)
 *       - Mag: gauss → µT (×100)
 *       - Timestamp: µs → ms (÷1000)
 */
bool SensorAdapter_Convert(const SensorManager_ScaledData_t* scaled_data,
                           const SensorManager_RawData_t* raw_data,
                           NavSensorData_t* nav_data);

/**
 * @brief Read sensors and convert to navigation format in one call
 * @param nav_data Output in format for navigation_manager
 * @return true if read and conversion successful
 *
 * @note This is a convenience function that combines:
 *       SensorManager_ReadRaw() + SensorManager_GetScaledData() + Convert()
 */
bool SensorAdapter_Read(NavSensorData_t* nav_data);

/**
 * @brief Get sensor adapter statistics
 * @param total_conversions Pointer to store total conversion count
 * @param failed_conversions Pointer to store failed conversion count
 */
void SensorAdapter_GetStats(uint32_t* total_conversions, uint32_t* failed_conversions);

/**
 * @brief Reset adapter statistics
 */
void SensorAdapter_ResetStats(void);

#endif /* SENSOR_ADAPTER_H */
