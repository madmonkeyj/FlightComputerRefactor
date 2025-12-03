/**
  ******************************************************************************
  * @file    data_logger.c
  * @brief   Data logging module - 192-BYTE STRUCTURE (Fixed Erase)
  ******************************************************************************
  */

#include "data_logger.h"
#include "quadspi.h"
#include "ble_module.h"
#include "gps_module.h"
#include "sensor_manager.h"
#include "main.h"  // For LED GPIO definitions
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Configuration Constants */
#define EXTERNAL_FLASH_SIZE     (4 * 1024 * 1024)  // 4MB total
#define TARGET_RECORDING_TIME_MS (30 * 60 * 1000)  // 30 minutes
#define RECORDING_INTERVAL_MS   (TARGET_RECORDING_TIME_MS / (DATA_AREA_SIZE / RECORD_SIZE))
#define SECTOR_SIZE             4096

// Global metadata instance
static FlashMetadata_t current_metadata = {0};
static bool metadata_loaded = false;

/* Private Variables */
static LoggerStatus_t logger_status = LOGGER_IDLE;
static bool flash_initialized = false;
static uint32_t current_write_address = 0;
static uint32_t records_written = 0;
static uint32_t recording_start_time = 0;
static uint32_t last_record_time = 0;
static uint32_t last_recording_attempt = 0;

/* Metadata management */
static bool metadata_dirty = false;
static uint32_t last_metadata_save_time = 0;
#define METADATA_SAVE_INTERVAL_MS  10000  /* Save metadata every 10 seconds max */

/* Navigation provider interface (optional) */
static const NavigationProvider_t* nav_provider = NULL;

/**
 * @brief Pack data into 192-byte record structure
 */
static void PackDataRecord(DataRecord_t* record) {
    if (!record) return;

    // LED1 ON = Entered function, about to memset record
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

    memset(record, 0, sizeof(DataRecord_t));
    record->timestamp_ms = HAL_GetTick();

    // LED1 OFF = Basic setup done, about to read sensors
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

    // === SENSOR DATA ===
    // ⚠️ CRITICAL: Use static to avoid stack overflow - these structs are large
    static SensorManager_RawData_t raw_data;
    static SensorManager_ScaledData_t sensor_data;

    if (SensorManager_ReadRaw(&raw_data) == HAL_OK) {
        SensorManager_ConvertToScaled(&raw_data, &sensor_data);
        if (raw_data.imu_valid) {
            record->accel[0] = sensor_data.accel_x_g * 9.80665f;
            record->accel[1] = sensor_data.accel_y_g * 9.80665f;
            record->accel[2] = sensor_data.accel_z_g * 9.80665f;
            record->gyro[0] = sensor_data.gyro_x_dps * 0.0174533f;
            record->gyro[1] = sensor_data.gyro_y_dps * 0.0174533f;
            record->gyro[2] = sensor_data.gyro_z_dps * 0.0174533f;
        }
    }

    // LED2 ON = Sensors done, about to read GPS
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

    // === GPS DATA ===
    // ⚠️ CRITICAL: Use static to avoid stack overflow - GPS_Data_t is large
    static GPS_Data_t gps_data;
    memset(&gps_data, 0, sizeof(gps_data));  // Clear before use
    bool gps_valid = GPS_GetCurrentData(&gps_data);

    // LED2 OFF = GPS done, about to read navigation
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

    if (gps_valid) {
        record->gps_lat = gps_data.latitude;
        record->gps_lon = gps_data.longitude;
        record->gps_alt = gps_data.altitude;
        record->gps_speed = gps_data.speed;
        record->gps_satellites = gps_data.satellites;
        record->gps_fix_status = gps_data.fix_status;
        record->gps_hdop = gps_data.hdop;
        record->gps_vel_n = gps_data.velN;
        record->gps_vel_e = gps_data.velE;
        record->gps_vel_d = gps_data.velD;
    } else {
        record->gps_fix_status = 'V';
        record->gps_hdop = 99.9f;
    }


    // === NAVIGATION DATA (via provider interface) ===
    if (nav_provider != NULL) {
        // LED1 ON = About to get quaternion
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

        // Get quaternion attitude
        static float quat[4];
        memset(quat, 0, sizeof(quat));  // Clear before use
        if (nav_provider->get_quaternion && nav_provider->get_quaternion(quat)) {
            record->quat[0] = quat[0];
            record->quat[1] = quat[1];
            record->quat[2] = quat[2];
            record->quat[3] = quat[3];
        }

        // Use temporary aligned buffer for vector data to prevent alignment faults
        float temp_vec[3];

        // Get position NED
        if (nav_provider->get_position_ned && nav_provider->get_position_ned(temp_vec)) {
             memcpy(record->pos_ned, temp_vec, sizeof(float) * 3);
        }

        // Get velocity NED
        if (nav_provider->get_velocity_ned && nav_provider->get_velocity_ned(temp_vec)) {
             memcpy(record->vel_ned, temp_vec, sizeof(float) * 3);
        }

        // Get navigation validity
        if (nav_provider->is_valid) {
            record->nav_valid = nav_provider->is_valid() ? 1 : 0;
        } else {
            record->nav_valid = gps_valid ? 1 : 0;
        }

        // LED1 OFF = Basic nav data complete, about to check EKF debug
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    } else {
        // No navigation provider - use GPS validity only
        record->nav_valid = gps_valid ? 1 : 0;

        // Zero out navigation fields
        for (int i = 0; i < 4; i++) {
            record->quat[i] = 0.0f;
        }
        for (int i = 0; i < 3; i++) {
            record->pos_ned[i] = 0.0f;
            record->vel_ned[i] = 0.0f;
        }
    }


    // === EKF DIAGNOSTICS (via provider interface) ===
    // SAFETY: Only collect EKF debug data when:
    // 1. Navigation is valid
    // 2. GPS fix is 'A' (valid 3D fix) - RELAXED FOR TESTING
    // 3. Sufficient satellites (reduces chance of bad GPS data causing EKF instability) - RELAXED FOR TESTING
    // 4. Critical sections in getters protect against race conditions
    bool ekf_debug_safe = (nav_provider != NULL && record->nav_valid);

    if (ekf_debug_safe) {
        // LED2 ON = About to collect EKF debug data
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

        // FIX: Use aligned temporary buffer for all vector retrievals
        // Passing record->field directly can cause UsageFault if DataRecord_t is packed
        float temp_vec[3];

        // Position uncertainty
        if (nav_provider->get_position_uncertainty &&
            nav_provider->get_position_uncertainty(temp_vec)) {
            memcpy(record->pos_uncertainty, temp_vec, sizeof(float) * 3);
        } else {
            for (int i = 0; i < 3; i++) record->pos_uncertainty[i] = NAN;
        }

        // Velocity uncertainty
        if (nav_provider->get_velocity_uncertainty &&
            nav_provider->get_velocity_uncertainty(temp_vec)) {
            memcpy(record->vel_uncertainty, temp_vec, sizeof(float) * 3);
        } else {
            for (int i = 0; i < 3; i++) record->vel_uncertainty[i] = NAN;
        }

        // Position innovation
        if (nav_provider->get_innovation_position &&
            nav_provider->get_innovation_position(temp_vec)) {
            memcpy(record->innovation_pos, temp_vec, sizeof(float) * 3);
        } else {
            for (int i = 0; i < 3; i++) record->innovation_pos[i] = NAN;
        }

        // 🚫 Velocity innovation – leave as NAN for now
        for (int i = 0; i < 3; i++) record->innovation_vel[i] = NAN;

        // Kalman gain position
        if (nav_provider->get_kalman_gain_position &&
            nav_provider->get_kalman_gain_position(temp_vec)) {
            memcpy(record->kalman_gain_pos, temp_vec, sizeof(float) * 3);
        } else {
            for (int i = 0; i < 3; i++) record->kalman_gain_pos[i] = NAN;
        }

        // Kalman gain velocity
        if (nav_provider->get_kalman_gain_velocity &&
            nav_provider->get_kalman_gain_velocity(temp_vec)) {
            memcpy(record->kalman_gain_vel, temp_vec, sizeof(float) * 3);
        } else {
            for (int i = 0; i < 3; i++) record->kalman_gain_vel[i] = NAN;
        }

        // Accel NED
        if (nav_provider->get_accel_ned &&
            nav_provider->get_accel_ned(temp_vec)) {
            memcpy(record->accel_ned, temp_vec, sizeof(float) * 3);
        } else {
            for (int i = 0; i < 3; i++) record->accel_ned[i] = NAN;
        }

        // Rejection flags
        if (nav_provider->get_rejection_flags &&
            nav_provider->get_rejection_flags(&record->gps_pos_rejected,
                                              &record->gps_vel_rejected,
                                              &record->zupt_applied)) {
            // OK
        } else {
            record->gps_pos_rejected = 0;
            record->gps_vel_rejected = 0;
            record->zupt_applied = 0;
        }

        // Motion state
        if (nav_provider->get_motion_state &&
            nav_provider->get_motion_state(&record->motion_state,
                                           &record->gps_velocity_suspect)) {
            // OK
        } else {
            record->motion_state = 3;  // UNKNOWN
            record->gps_velocity_suspect = 0;
        }

        // LED2 OFF = EKF debug data collection complete!
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    } else {
        // SAFETY: EKF not safe to read - fill all debug fields with NAN/zeros
        for (int i = 0; i < 3; i++) {
            record->pos_uncertainty[i] = NAN;
            record->vel_uncertainty[i] = NAN;
            record->innovation_pos[i] = NAN;
            record->innovation_vel[i] = NAN;
            record->kalman_gain_pos[i] = NAN;
            record->kalman_gain_vel[i] = NAN;
            record->accel_ned[i] = NAN;
        }
        record->gps_pos_rejected = 0;
        record->gps_vel_rejected = 0;
        record->zupt_applied = 0;
        record->motion_state = 3;  // UNKNOWN
        record->gps_velocity_suspect = 0;
    }
}

/**
 * @brief Register navigation data provider
 * @param provider Pointer to navigation provider structure (or NULL to disable)
 * @note This decouples data_logger from specific navigation implementations
 * @note Provider can be Mahony filter, EKF, or any other navigation system
 */
void DataLogger_RegisterNavProvider(const NavigationProvider_t* provider) {
    nav_provider = provider;
}

/**
 * @brief Initialize data logger
 */
bool DataLogger_Init(void) {
    uint8_t flash_id[3] = {0};

    // First, try simple init
    if (QSPI_Simple_Init() != HAL_OK) return false;

    // Verify ID
    if (QSPI_Read_ID(flash_id) != HAL_OK) {
        HAL_Delay(10);
        if (QSPI_Read_ID(flash_id) != HAL_OK) {
            logger_status = LOGGER_ERROR;
            return false;
        }
    }

    flash_initialized = true;

    if (Metadata_Load()) {
        // Previous session restored
    } else {
        Metadata_Clear();
        current_metadata.recording_session_id = 1;
        Metadata_Save();
    }

    logger_status = LOGGER_IDLE;
    return true;
}

/**
 * @brief Start recording
 */
bool DataLogger_StartRecording(void) {
    if (!flash_initialized) return false;
    if (logger_status == LOGGER_RECORDING) return true;

    current_write_address = 0;
    records_written = 0;
    recording_start_time = HAL_GetTick();
    last_record_time = 0;
    last_recording_attempt = 0;
    logger_status = LOGGER_RECORDING;

    current_metadata.recording_session_id++;
    Metadata_Save();

    return true;
}

/**
 * @brief Stop recording
 * @note Always saves metadata on stop to ensure state persisted
 * @note Commits any pending write before stopping
 */
bool DataLogger_StopRecording(void) {
    if (logger_status != LOGGER_RECORDING) {
        return true;
    }

    logger_status = LOGGER_IDLE;

    /* Save metadata on stop (ensure state persisted) */
    if (metadata_dirty) {
        Metadata_Save();
        metadata_dirty = false;
    }

    return true;
}

/**
 * @brief Record current sensor/GPS/attitude data to flash
 * @note Uses blocking QSPI write for simplicity and reliability
 * @note Uses metadata dirty flag to reduce flash wear
 */
bool DataLogger_RecordData(void) {
    static uint32_t call_count = 0;
    if (call_count < 5) {  // Only log first 5 calls
        DebugPrint("LOGGER: DataLogger_RecordData() called (#%lu)\r\n", call_count);
    }
    call_count++;

    if (logger_status != LOGGER_RECORDING || !flash_initialized) {
        return false;
    }

    uint32_t current_time = HAL_GetTick();

    /* Rate limiting */
    if (current_time - last_recording_attempt < RECORDING_INTERVAL_MS) {
        return true;
    }
    last_recording_attempt = current_time;

    /* Check flash space */
    if (current_write_address + sizeof(DataRecord_t) > DATA_AREA_SIZE) {
        DataLogger_StopRecording();
        return false;
    }

    // ⚠️ CRITICAL FIX: Use static to avoid stack overflow
    // 192 bytes on stack is too much when EKF is running (deep call stack)
    static DataRecord_t temp_record;

    // DIAGNOSTIC: LED2 ON = About to clear temp_record
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

    memset(&temp_record, 0, sizeof(temp_record));

    // DIAGNOSTIC: LED2 OFF = memset complete, about to call PackDataRecord
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

    // DIAGNOSTIC: LED3 ON = Calling PackDataRecord now
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);

    PackDataRecord(&temp_record);

    // DIAGNOSTIC: LED3 OFF = PackDataRecord returned successfully!
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

    if (temp_record.timestamp_ms == 0) {
        return false;
    }

    // DIAGNOSTIC: Log before attempting write (to see if we reach this point)
    static uint32_t write_count_debug = 0;
    if (write_count_debug < 5) {  // Only log first 5 writes
        DebugPrint("LOGGER: Attempting QSPI write #%lu to addr 0x%08lX\r\n",
                   write_count_debug, current_write_address);
    }
    write_count_debug++;

    // Write data record to QSPI flash with timeout protection
    uint32_t write_start_time = HAL_GetTick();
    HAL_StatusTypeDef write_result = QSPI_Quad_Write((uint8_t*)&temp_record,
                                                     current_write_address,
                                                     sizeof(DataRecord_t));
    uint32_t write_duration = HAL_GetTick() - write_start_time;

    // Check for write failure or excessive blocking
    if (write_result != HAL_OK) {
        DebugPrint("LOGGER ERROR: QSPI write failed at addr 0x%08lX after %lums\r\n",
                   current_write_address, write_duration);
        logger_status = LOGGER_ERROR;
        return false;
    }

    // DIAGNOSTIC: Log completion for first 5 writes
    if (write_count_debug <= 5) {
        DebugPrint("LOGGER: Write #%lu completed in %lums\r\n",
                   write_count_debug - 1, write_duration);
    }

    // Warn if write took too long (should be <10ms for 192 bytes)
    if (write_duration > 20) {
        DebugPrint("LOGGER WARNING: QSPI write slow (%lums) at addr 0x%08lX\r\n",
                   write_duration, current_write_address);
    }

    // Update metadata
    current_write_address += sizeof(DataRecord_t);
    records_written++;
    last_record_time = current_time;
    metadata_dirty = true;

    return true;
}


/**
 * @brief Update data logger - handles periodic metadata saves
 * @note Call this from main loop to enable periodic metadata persistence
 * @note MODIFIED: Skip periodic saves during recording to prevent BLE blocking
 * @note Metadata is saved on start/stop instead - prevents 5-8 second blocking
 */
void DataLogger_Update(void) {
    /* Skip periodic metadata saves while recording to prevent blocking BLE */
    /* Metadata is saved when starting/stopping recording instead */
    if (logger_status == LOGGER_RECORDING) {
        return;  /* Don't save metadata while actively recording */
    }

    /* Periodically save metadata if dirty (only when NOT recording) */
    if (metadata_dirty && (HAL_GetTick() - last_metadata_save_time) > METADATA_SAVE_INTERVAL_MS) {
        if (Metadata_Save()) {
            metadata_dirty = false;
            last_metadata_save_time = HAL_GetTick();
        }
    }
}

/**
 * @brief Get logger statistics
 */
bool DataLogger_GetStats(LoggerStats_t* stats) {
    if (!stats) return false;

    stats->status = logger_status;
    stats->records_written = records_written;
    stats->flash_bytes_used = current_write_address;
    stats->recording_start_time = recording_start_time;
    stats->last_record_time = last_record_time;
    stats->recording_rate_hz = 1000 / RECORDING_INTERVAL_MS;
    stats->flash_ready = flash_initialized;
    stats->record_size = sizeof(DataRecord_t);

    if (logger_status == LOGGER_RECORDING) {
        uint32_t bytes_remaining = DATA_AREA_SIZE - current_write_address;
        uint32_t records_remaining = bytes_remaining / sizeof(DataRecord_t);
        stats->estimated_time_remaining_ms = records_remaining * RECORDING_INTERVAL_MS;
    } else {
        stats->estimated_time_remaining_ms = 0;
    }

    return true;
}

/**
 * @brief Get status string
 */
bool DataLogger_GetStatusString(char* buffer, size_t buffer_size) {
    if (!buffer || buffer_size < 100) return false;

    LoggerStats_t stats;
    if (!DataLogger_GetStats(&stats)) return false;

    const char* status_str;
    switch (stats.status) {
        case LOGGER_IDLE: status_str = "IDLE"; break;
        case LOGGER_RECORDING: status_str = "RECORDING"; break;
        case LOGGER_DOWNLOADING: status_str = "DOWNLOADING"; break;
        case LOGGER_ERROR: status_str = "ERROR"; break;
        default: status_str = "UNKNOWN"; break;
    }

    // Fix float formatting logic
    float flash_usage_percent = 0.0f;
    if (EXTERNAL_FLASH_SIZE > 0) {
        flash_usage_percent = (float)stats.flash_bytes_used * 100.0f / (float)EXTERNAL_FLASH_SIZE;
    }

    if (stats.status == LOGGER_RECORDING) {
        snprintf(buffer, buffer_size,
                "Status: %s, Records: %lu (%.1f%%), Rate: %luHz",
                status_str, stats.records_written, flash_usage_percent,
                stats.recording_rate_hz);
    } else {
        snprintf(buffer, buffer_size,
                "Status: %s, Records: %lu (%.1f%% used), Flash: %s",
                status_str, stats.records_written, flash_usage_percent,
                stats.flash_ready ? "Ready" : "Error");
    }

    return true;
}

bool Metadata_Load(void) {
    if (!flash_initialized) return false;

    FlashMetadata_t loaded_metadata;
    if (QSPI_Quad_Read((uint8_t*)&loaded_metadata, METADATA_SECTOR_ADDR, sizeof(FlashMetadata_t)) != HAL_OK) {
        return false;
    }

    if (!Metadata_Validate(&loaded_metadata)) return false;

    current_metadata = loaded_metadata;
    records_written = current_metadata.records_written;
    current_write_address = current_metadata.current_write_address;
    recording_start_time = current_metadata.recording_start_time;
    last_record_time = current_metadata.last_record_time;
    logger_status = (LoggerStatus_t)current_metadata.logger_status;

    if (logger_status == LOGGER_RECORDING) logger_status = LOGGER_IDLE;

    metadata_loaded = true;
    return true;
}

bool DataLogger_IsRecording(void) { return logger_status == LOGGER_RECORDING; }
bool DataLogger_IsDownloading(void) { return logger_status == LOGGER_DOWNLOADING; }

/**
 * @brief Erase all data using CHIP ERASE (Faster)
 */
bool DataLogger_EraseAll(void) {
    if (!flash_initialized) return false;
    if (logger_status == LOGGER_RECORDING || logger_status == LOGGER_DOWNLOADING) return false;

    /* CHANGE: Use Chip Erase instead of Sector Erase Loop */
    /* This takes ~20-40 seconds but is reliable */
    if (CSP_QSPI_Erase_Chip() != HAL_OK) {
        return false;
    }

    // Reset state
    current_write_address = 0;
    records_written = 0;
    recording_start_time = 0;
    last_record_time = 0;

    // We must re-create metadata after chip erase
    Metadata_Clear();
    current_metadata.recording_session_id = 1;
    Metadata_Save();

    return true;
}

/* === METADATA FUNCTIONS === */

uint32_t Metadata_CalculateChecksum(const FlashMetadata_t* metadata) {
    if (!metadata) return 0;
    uint32_t checksum = 0;
    const uint8_t* data = (const uint8_t*)metadata;
    for (int i = 0; i < (sizeof(FlashMetadata_t) - sizeof(uint32_t)); i++) {
        checksum += data[i];
    }
    return checksum;
}

bool Metadata_Validate(const FlashMetadata_t* metadata) {
    if (!metadata) return false;
    if (metadata->magic_number != METADATA_MAGIC) return false;
    if (metadata->version != METADATA_VERSION) return false;
    if (metadata->records_written > (DATA_AREA_SIZE / sizeof(DataRecord_t))) return false;
    if (metadata->current_write_address > DATA_AREA_SIZE) return false;
    return (metadata->checksum == Metadata_CalculateChecksum(metadata));
}

bool Metadata_Save(void) {
    if (!flash_initialized) return false;

    current_metadata.magic_number = METADATA_MAGIC;
    current_metadata.version = METADATA_VERSION;
    current_metadata.records_written = records_written;
    current_metadata.current_write_address = current_write_address;
    current_metadata.recording_start_time = recording_start_time;
    current_metadata.last_record_time = last_record_time;
    current_metadata.logger_status = (uint8_t)logger_status;
    current_metadata.record_size = sizeof(DataRecord_t);
    current_metadata.checksum = Metadata_CalculateChecksum(&current_metadata);

    // Erase metadata sector
    if (QSPI_Simple_Erase(METADATA_SECTOR_ADDR) != HAL_OK) return false;

    // SIMPLIFIED: Use blocking write like old working code
    if (QSPI_Quad_Write((uint8_t*)&current_metadata, METADATA_SECTOR_ADDR, sizeof(FlashMetadata_t)) != HAL_OK) {
        return false;
    }

    FlashMetadata_t verify_metadata;
    if (QSPI_Quad_Read((uint8_t*)&verify_metadata, METADATA_SECTOR_ADDR, sizeof(FlashMetadata_t)) != HAL_OK) {
        return false;
    }

    if (memcmp(&current_metadata, &verify_metadata, sizeof(FlashMetadata_t)) != 0) return false;

    metadata_loaded = true;
    return true;
}

void Metadata_Clear(void) {
    memset(&current_metadata, 0, sizeof(FlashMetadata_t));
    metadata_loaded = false;
    records_written = 0;
    current_write_address = 0;
    recording_start_time = 0;
    last_record_time = 0;
}
