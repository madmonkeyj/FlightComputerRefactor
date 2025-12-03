/**
  ******************************************************************************
  * @file    sensor_system.c
  * @brief   Raw sensor I/O system implementation - Phase 1 refactoring
  * @note    Extracted from sensor_manager.c - handles ONLY raw sensor I/O
  ******************************************************************************
  */

#include "sensor_system.h"
#include "i2c_bitbang.h"
#include "debug_utils.h"
#include <stdio.h>
#include <string.h>

/* Private sensor instances - moved from global scope for encapsulation */
static BMI088a accel_sensor;
static BMI088g gyro_sensor;
static BMP390 baro_sensor;
static MLX90393_t mag_sensor;
static struct BMP390_calib_data baro_calib_data;

/* Health tracking and diagnostics */
static uint32_t total_read_count = 0;
static uint32_t total_error_count = 0;
static uint32_t last_successful_read_time = 0;
static bool system_initialized = false;

/* Individual sensor error tracking */
static uint32_t accel_errors = 0;
static uint32_t gyro_errors = 0;
static uint32_t mag_errors = 0;
static uint32_t baro_errors = 0;

/**
 * @brief Initialize all sensors
 */
bool SensorSystem_Init(void) {
    DebugPrint("SensorSystem: Starting initialization...\r\n");

    /* Initialize I2C communication */
    DebugPrint("SensorSystem: Initializing I2C...\r\n");
    if (I2C_BitBang_Init() != HAL_OK) {
        DebugPrint("SensorSystem: ERROR - I2C initialization failed\r\n");
        return false;
    }

    bool init_success = true;
    uint8_t error_count;
    char debug_msg[150];

    /* Initialize BMI088 Accelerometer */
    DebugPrint("SensorSystem: Initializing accelerometer...\r\n");
    error_count = BMI088a_Initialise(&accel_sensor);
    if (error_count == 255) {
        DebugPrint("SensorSystem: ERROR - Accelerometer not detected\r\n");
        init_success = false;
    } else if (error_count > 0) {
        sprintf(debug_msg, "SensorSystem: WARNING - Accelerometer init had %d errors\r\n", error_count);
        DebugPrint(debug_msg);
    } else {
        DebugPrint("SensorSystem: Accelerometer initialized successfully\r\n");
    }

    /* Initialize BMI088 Gyroscope */
    DebugPrint("SensorSystem: Initializing gyroscope...\r\n");
    error_count = BMI088g_Initialise(&gyro_sensor);
    if (error_count == 255) {
        DebugPrint("SensorSystem: ERROR - Gyroscope not detected\r\n");
        init_success = false;
    } else if (error_count > 0) {
        sprintf(debug_msg, "SensorSystem: WARNING - Gyroscope init had %d errors\r\n", error_count);
        DebugPrint(debug_msg);
    } else {
        DebugPrint("SensorSystem: Gyroscope initialized successfully\r\n");
    }

    /* Initialize MLX90393 Magnetometer */
    DebugPrint("SensorSystem: Initializing magnetometer...\r\n");
    if (MLX90393_InitWithConfig(&mag_sensor, &MLX90393_CONFIG_EARTH_FIELD) != HAL_OK) {
        DebugPrint("SensorSystem: ERROR - Magnetometer initialization failed\r\n");
        init_success = false;
    } else {
        DebugPrint("SensorSystem: Magnetometer initialized successfully\r\n");
    }

    /* Initialize BMP390 Barometer */
    DebugPrint("SensorSystem: Initializing barometer...\r\n");
    error_count = BMP390_Initialise(&baro_sensor);
    if (error_count == 255) {
        DebugPrint("SensorSystem: ERROR - Barometer not detected\r\n");
        init_success = false;
    } else if (error_count > 0) {
        sprintf(debug_msg, "SensorSystem: WARNING - Barometer init had %d errors\r\n", error_count);
        DebugPrint(debug_msg);
    } else {
        DebugPrint("SensorSystem: Barometer initialized successfully\r\n");

        /* Read barometer calibration data */
        if (BMP390_ReadCalibrationData(&baro_sensor, &baro_calib_data) != HAL_OK) {
            DebugPrint("SensorSystem: WARNING - Failed to read barometer calibration data\r\n");
        } else {
            DebugPrint("SensorSystem: Barometer calibration data loaded\r\n");
        }
    }

    /* Reset diagnostics */
    total_read_count = 0;
    total_error_count = 0;
    accel_errors = 0;
    gyro_errors = 0;
    mag_errors = 0;
    baro_errors = 0;
    last_successful_read_time = HAL_GetTick();

    system_initialized = true;

    if (init_success) {
        DebugPrint("SensorSystem: All sensors initialized successfully\r\n");
    } else {
        DebugPrint("SensorSystem: Initialization completed with some sensor failures\r\n");
    }

    return true; /* Continue operation even with some sensor failures */
}

/**
 * @brief Read data from all sensors - optimized for 500Hz operation
 */
bool SensorSystem_Read(SensorData_t* data) {
    if (!data || !system_initialized) {
        return false;
    }

    /* Clear data structure and set timestamp */
    memset(data, 0, sizeof(SensorData_t));
    data->timestamp = HAL_GetTick();
    total_read_count++;

    bool all_sensors_ok = true;

    /* Read BMI088 Accelerometer */
    if (BMI088_ReadAcc(&accel_sensor) == HAL_OK) {
        data->accel[0] = accel_sensor.acc_convert[0];  /* m/s² */
        data->accel[1] = accel_sensor.acc_convert[1];
        data->accel[2] = accel_sensor.acc_convert[2];
        data->accel_valid = true;
    } else {
        data->accel_valid = false;
        accel_errors++;
        total_error_count++;
        all_sensors_ok = false;
    }

    /* Read BMI088 Gyroscope */
    if (BMI088_ReadGyro(&gyro_sensor) == HAL_OK) {
        data->gyro[0] = gyro_sensor.gyro_convert[0];   /* rad/s */
        data->gyro[1] = gyro_sensor.gyro_convert[1];
        data->gyro[2] = gyro_sensor.gyro_convert[2];
        data->gyro_valid = true;
    } else {
        data->gyro_valid = false;
        gyro_errors++;
        total_error_count++;
        all_sensors_ok = false;
    }

    /* Read MLX90393 Magnetometer */
    if (MLX90393_ReadMeasurement(&mag_sensor) == HAL_OK) {
        data->mag[0] = mag_sensor.mag[0];              /* µT */
        data->mag[1] = mag_sensor.mag[1];
        data->mag[2] = mag_sensor.mag[2];
        data->mag_valid = true;
    } else {
        data->mag_valid = false;
        mag_errors++;
        total_error_count++;
        all_sensors_ok = false;
    }

    /* Read BMP390 Barometer */
    if (BMP390_ReadPressure(&baro_sensor) == HAL_OK) {
        data->pressure = baro_sensor.pres_P;           /* Pa */
        data->temperature = baro_sensor.temp_C;        /* °C */
        data->baro_valid = true;
    } else {
        data->baro_valid = false;
        baro_errors++;
        total_error_count++;
        all_sensors_ok = false;
    }

    /* Update overall validity */
    data->valid = all_sensors_ok;

    /* Update last successful read time if all sensors OK */
    if (all_sensors_ok) {
        last_successful_read_time = data->timestamp;
    }

    return data->valid;
}

/**
 * @brief Check if sensor system is healthy
 */
bool SensorSystem_IsHealthy(void) {
    if (!system_initialized) {
        return false;
    }

    uint32_t current_time = HAL_GetTick();

    /* Check if we've had a successful read recently (within 50ms) */
    if (current_time - last_successful_read_time > 50) {
        return false;
    }

    /* Check overall error rate (should be less than 5% after initial startup) */
    if (total_read_count > 100) {
        uint32_t error_rate_percent = (total_error_count * 100) / total_read_count;
        if (error_rate_percent > 5) {
            return false;
        }
    }

    return true;
}

/**
 * @brief Get detailed diagnostics information
 */
void SensorSystem_GetDiagnostics(SensorSystemDiagnostics_t* diag) {
    if (!diag) {
        return;
    }

    diag->total_reads = total_read_count;
    diag->error_count = total_error_count;
    diag->last_successful_read = last_successful_read_time;
    diag->healthy = SensorSystem_IsHealthy();

    if (total_read_count > 0) {
        diag->error_rate_percent = ((float)total_error_count * 100.0f) / (float)total_read_count;
    } else {
        diag->error_rate_percent = 0.0f;
    }
}

/**
 * @brief Get formatted diagnostics string for debug output
 */
void SensorSystem_GetDiagnosticsString(char* buffer, size_t buffer_size) {
    if (!buffer || buffer_size == 0) {
        return;
    }

    SensorSystemDiagnostics_t diag;
    SensorSystem_GetDiagnostics(&diag);

    uint32_t time_since_last_read = HAL_GetTick() - diag.last_successful_read;

    snprintf(buffer, buffer_size,
        "SensorSystem Diagnostics:\r\n"
        "  Status: %s\r\n"
        "  Total reads: %lu\r\n"
        "  Total errors: %lu (%.1f%%)\r\n"
        "  Individual errors: Accel=%lu, Gyro=%lu, Mag=%lu, Baro=%lu\r\n"
        "  Last successful read: %lu ms ago\r\n",
        diag.healthy ? "HEALTHY" : "DEGRADED",
        diag.total_reads,
        diag.error_count, diag.error_rate_percent,
        accel_errors, gyro_errors, mag_errors, baro_errors,
        time_since_last_read
    );
}

/**
 * @brief Reset error counters and statistics
 */
void SensorSystem_ResetDiagnostics(void) {
    total_read_count = 0;
    total_error_count = 0;
    accel_errors = 0;
    gyro_errors = 0;
    mag_errors = 0;
    baro_errors = 0;
    last_successful_read_time = HAL_GetTick();

    DebugPrint("SensorSystem: Diagnostics reset\r\n");
}
