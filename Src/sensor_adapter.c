/**
 ******************************************************************************
 * @file    sensor_adapter.c
 * @brief   Sensor adapter implementation
 * @note    Bridges sensor_manager (actual hardware) with navigation_manager
 ******************************************************************************
 */

#include "sensor_adapter.h"
#include <math.h>
#include <string.h>

/* Physical constants for unit conversion */
#define GRAVITY_MS2         9.80665f       // Standard gravity in m/s²
#define DEG_TO_RAD          (M_PI / 180.0f) // Degrees to radians
#define GAUSS_TO_MICROTESLA 100.0f         // Gauss to µT conversion

/* Statistics */
static uint32_t total_conversions = 0;
static uint32_t failed_conversions = 0;

/**
 * @brief Convert sensor_manager scaled data to navigation format
 */
bool SensorAdapter_Convert(const SensorManager_ScaledData_t* scaled_data,
                           const SensorManager_RawData_t* raw_data,
                           NavSensorData_t* nav_data) {
    // Validate inputs
    if (!scaled_data || !raw_data || !nav_data) {
        failed_conversions++;
        return false;
    }

    // Clear output structure
    memset(nav_data, 0, sizeof(NavSensorData_t));

    /* Convert accelerometer: g → m/s² */
    nav_data->accel[0] = scaled_data->accel_x_g * GRAVITY_MS2;
    nav_data->accel[1] = scaled_data->accel_y_g * GRAVITY_MS2;
    nav_data->accel[2] = scaled_data->accel_z_g * GRAVITY_MS2;
    nav_data->accel_valid = raw_data->imu_valid;

    /* Convert gyroscope: deg/s → rad/s */
    nav_data->gyro[0] = scaled_data->gyro_x_dps * DEG_TO_RAD;
    nav_data->gyro[1] = scaled_data->gyro_y_dps * DEG_TO_RAD;
    nav_data->gyro[2] = scaled_data->gyro_z_dps * DEG_TO_RAD;
    nav_data->gyro_valid = raw_data->imu_valid;

    /* Convert magnetometer: gauss → µT */
    nav_data->mag[0] = scaled_data->mag_x_gauss * GAUSS_TO_MICROTESLA;
    nav_data->mag[1] = scaled_data->mag_y_gauss * GAUSS_TO_MICROTESLA;
    nav_data->mag[2] = scaled_data->mag_z_gauss * GAUSS_TO_MICROTESLA;
    nav_data->mag_valid = raw_data->mag_valid;

    /* Pressure and temperature (already in correct units) */
    nav_data->pressure = scaled_data->pressure_pa;      // Pa
    nav_data->temperature = scaled_data->temperature_c;  // °C
    nav_data->baro_valid = raw_data->baro_valid;

    /* Timestamp: microseconds → milliseconds */
    nav_data->timestamp = scaled_data->timestamp_us / 1000;

    /* Overall validity: at least accel and gyro must be valid */
    nav_data->valid = nav_data->accel_valid && nav_data->gyro_valid;

    total_conversions++;
    return true;
}

/**
 * @brief Read sensors and convert to navigation format in one call
 */
bool SensorAdapter_Read(NavSensorData_t* nav_data) {
    SensorManager_RawData_t raw_data;
    SensorManager_ScaledData_t scaled_data;

    if (!nav_data) {
        failed_conversions++;
        return false;
    }

    // ✅ FIX: Only read sensors ONCE, then convert
    HAL_StatusTypeDef status = SensorManager_ReadRaw(&raw_data);
    if (status != HAL_OK) {
        failed_conversions++;
        return false;
    }

    // Convert raw to scaled WITHOUT re-reading sensors
    SensorManager_ConvertToScaled(&raw_data, &scaled_data);

    // Convert to navigation format
    return SensorAdapter_Convert(&scaled_data, &raw_data, nav_data);
}

/**
 * @brief Get sensor adapter statistics
 */
void SensorAdapter_GetStats(uint32_t* total_conv, uint32_t* failed_conv) {
    if (total_conv) {
        *total_conv = total_conversions;
    }
    if (failed_conv) {
        *failed_conv = failed_conversions;
    }
}

/**
 * @brief Reset adapter statistics
 */
void SensorAdapter_ResetStats(void) {
    total_conversions = 0;
    failed_conversions = 0;
}
