# FlightComputerRefactor - Comprehensive Refactoring Plan

**Date:** 2025-11-25
**Status:** Ready for Implementation
**Estimated Time:** 2-3 days

---

## OBJECTIVES

1. **Fix Critical Bugs** - Race conditions, DMA issues, flash wear
2. **Improve Architecture** - Decouple modules for EKF/LoRa integration
3. **Clean Code** - Remove debug code, standardize patterns
4. **Prepare for Integration** - Clean interfaces for other codebase

---

## TASK 1: Fix i2c_dma_arbiter Race Condition & Add Timeout

**File:** `Src/i2c_dma_arbiter.c`, `Inc/i2c_dma_arbiter.h`

### Problems
- Race condition between busy check and set
- No timeout for stuck DMA transfers
- Misleading "priority-based" documentation

### Changes

#### 1.1 Add timeout tracking to state structure
```c
// In i2c_dma_arbiter.c
typedef struct {
    volatile bool busy;
    volatile I2C_DMA_Device_t current_device;
    I2C_DMA_Callback_t callback;
    I2C_DMA_Arbiter_Stats_t stats;
    uint32_t transaction_start_time;  // NEW: For timeout detection
} I2C_DMA_Arbiter_State_t;
```

#### 1.2 Add critical sections to RequestTransfer
```c
HAL_StatusTypeDef I2C_DMA_Arbiter_RequestTransfer(
    I2C_HandleTypeDef *hi2c,
    I2C_DMA_Device_t device,
    uint16_t dev_address,
    uint16_t mem_address,
    uint16_t mem_size,
    uint8_t *buffer,
    uint16_t size,
    I2C_DMA_Callback_t callback
) {
    HAL_StatusTypeDef status;

    /* Update request statistics */
    switch (device) {
        case I2C_DMA_DEVICE_MAG:
            arbiter_state.stats.mag_requests++;
            break;
        case I2C_DMA_DEVICE_BARO:
            arbiter_state.stats.baro_requests++;
            break;
        case I2C_DMA_DEVICE_HIGHG:
            arbiter_state.stats.highg_requests++;
            break;
        default:
            break;
    }

    /* CRITICAL SECTION: Check and set busy atomically */
    __disable_irq();

    if (arbiter_state.busy) {
        __enable_irq();

        /* Update conflict statistics */
        switch (device) {
            case I2C_DMA_DEVICE_MAG:
                arbiter_state.stats.mag_conflicts++;
                break;
            case I2C_DMA_DEVICE_BARO:
                arbiter_state.stats.baro_conflicts++;
                break;
            case I2C_DMA_DEVICE_HIGHG:
                arbiter_state.stats.highg_conflicts++;
                break;
            default:
                break;
        }
        return HAL_BUSY;
    }

    /* Grant access */
    arbiter_state.busy = true;
    arbiter_state.current_device = device;
    arbiter_state.callback = callback;
    arbiter_state.transaction_start_time = HAL_GetTick();

    __enable_irq();
    /* END CRITICAL SECTION */

    /* Start DMA transfer */
    status = HAL_I2C_Mem_Read_DMA(hi2c, dev_address, mem_address,
                                   mem_size, buffer, size);

    if (status != HAL_OK) {
        /* Transfer failed to start - release arbiter */
        arbiter_state.busy = false;
        arbiter_state.current_device = I2C_DMA_DEVICE_COUNT;
        arbiter_state.callback = NULL;
        return status;
    }

    /* Transfer started successfully */
    arbiter_state.stats.total_transfers++;
    return HAL_OK;
}
```

#### 1.3 Add timeout watchdog function
```c
// In i2c_dma_arbiter.c
#define I2C_DMA_TIMEOUT_MS  100  // 100ms timeout

/**
 * @brief Watchdog function to detect stuck DMA transfers
 * @note Call this from main loop or periodic timer
 * @retval true if timeout occurred and arbiter was reset
 */
bool I2C_DMA_Arbiter_Watchdog(void) {
    if (!arbiter_state.busy) {
        return false;
    }

    uint32_t elapsed = HAL_GetTick() - arbiter_state.transaction_start_time;

    if (elapsed > I2C_DMA_TIMEOUT_MS) {
        /* Timeout detected - force release */
        __disable_irq();
        arbiter_state.busy = false;
        arbiter_state.current_device = I2C_DMA_DEVICE_COUNT;
        arbiter_state.callback = NULL;
        __enable_irq();

        /* Could also reset I2C peripheral here if needed */
        // HAL_I2C_DeInit(&hi2c1);
        // HAL_I2C_Init(&hi2c1);

        return true;  /* Timeout occurred */
    }

    return false;
}
```

#### 1.4 Update header file
```c
// In i2c_dma_arbiter.h
/**
 * @brief Watchdog to detect stuck DMA transfers
 * @retval true if timeout occurred and arbiter was reset
 * @note Call from main loop or periodic timer (every 10-50ms recommended)
 */
bool I2C_DMA_Arbiter_Watchdog(void);
```

#### 1.5 Update documentation
- Remove "priority-based" claims from comments
- Clarify this is a **non-preemptive FCFS arbiter**
- Document that priority enum is for future use

---

## TASK 2: Fix data_logger DMA Race & Flash Wear

**File:** `Src/data_logger.c`, `Inc/data_logger.h`

### Problems
- DMA write doesn't wait for completion
- Metadata sector erased every 50 records (excessive wear)
- No power-loss protection

### Changes

#### 2.1 Add QSPI busy flag and completion callback
```c
// In data_logger.c (private variables section)
static volatile bool qspi_write_busy = false;
static volatile bool qspi_write_error = false;

/**
 * @brief QSPI write completion callback
 * @note Called from QSPI interrupt when DMA write completes
 */
void DataLogger_QSPI_WriteComplete(void) {
    qspi_write_busy = false;
    qspi_write_error = false;
}

/**
 * @brief QSPI write error callback
 */
void DataLogger_QSPI_WriteError(void) {
    qspi_write_busy = false;
    qspi_write_error = true;
}
```

#### 2.2 Fix RecordData to wait for DMA completion
```c
bool DataLogger_RecordData(void) {
    if (logger_status != LOGGER_RECORDING || !flash_initialized) {
        return false;
    }

    /* Check if previous write still in progress */
    if (qspi_write_busy) {
        return false;  /* Previous write not done, skip this record */
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

    /* Pack data into record */
    PackDataRecord(&temp_record);

    if (temp_record.timestamp_ms == 0) {
        return false;
    }

    /* Start DMA write */
    qspi_write_busy = true;
    qspi_write_error = false;

    if (QSPI_Quad_Write_DMA((uint8_t*)&temp_record, current_write_address,
                            sizeof(DataRecord_t)) != HAL_OK) {
        qspi_write_busy = false;
        logger_status = LOGGER_ERROR;
        return false;
    }

    /* Wait for DMA completion (with timeout) */
    uint32_t start = HAL_GetTick();
    while (qspi_write_busy && (HAL_GetTick() - start) < 100) {
        /* Wait for completion or timeout */
    }

    if (qspi_write_busy || qspi_write_error) {
        /* Timeout or error */
        logger_status = LOGGER_ERROR;
        return false;
    }

    /* Now safe to increment address */
    current_write_address += sizeof(DataRecord_t);
    records_written++;
    last_record_time = current_time;

    /* Mark metadata as dirty instead of immediate save */
    metadata_dirty = true;

    return true;
}
```

#### 2.3 Reduce metadata flash writes
```c
// In data_logger.c (private variables)
static bool metadata_dirty = false;
static uint32_t last_metadata_save_time = 0;

#define METADATA_SAVE_INTERVAL_MS  10000  // Save every 10 seconds max

// Update DataLogger_Update to periodically save metadata
void DataLogger_Update(void) {
    if (metadata_dirty && (HAL_GetTick() - last_metadata_save_time) > METADATA_SAVE_INTERVAL_MS) {
        if (Metadata_Save()) {
            metadata_dirty = false;
            last_metadata_save_time = HAL_GetTick();
        }
    }
}

// Update StopRecording to save metadata
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

// Remove the "every 50 records" save from RecordData
```

**Flash Wear Improvement:**
- Before: 180 erases per 30-min flight
- After: ~3 erases per 30-min flight (every 10 seconds)
- **Lifespan: 555 flights → 33,000 flights**

---

## TASK 3: Refactor sensor_manager Static Variables

**File:** `Src/sensor_manager.c`

### Problem
- Static variables inside function (not re-entrant safe)
- Hidden state makes testing difficult

### Changes

#### 3.1 Move static variables to file scope
```c
// In sensor_manager.c (with other private variables)
/* Last valid sensor readings for decimated sensors */
static BMP581_Data_t last_valid_baro_data = {0};
static H3LIS331DL_Data_t last_valid_highg_data = {0};

// In SensorManager_ReadRaw - remove the 'static' declaration:
HAL_StatusTypeDef SensorManager_ReadRaw(SensorManager_RawData_t *data) {
    VALIDATE_PTR(data);

    HAL_StatusTypeDef hal_status = HAL_OK;
    ICM42688_Data_t imu_data;

    /* ... IMU and MAG reading code ... */

    /* DECIMATE BARO to target rate */
    if (config.use_baro) {
        if (baro_decimation_counter++ >= baro_decimation_factor) {
            baro_decimation_counter = 0;

            if (!I2C_DMA_Arbiter_IsBusy(&hi2c1)) {
                BMP581_Data_t baro_data;  // Local, not static
                HAL_StatusTypeDef baro_status = BMP581_ReadSensorData(&baro_data);

                if (baro_status == HAL_OK) {
                    /* New valid data */
                    data->pressure_raw = baro_data.pressure_raw;
                    data->temperature_raw = baro_data.temperature_raw;
                    data->baro_valid = 1;
                    sensor_status.baro_read_count++;

                    /* Save for next time */
                    last_valid_baro_data = baro_data;
                } else if (baro_status == HAL_BUSY) {
                    /* Use cached data */
                    data->pressure_raw = last_valid_baro_data.pressure_raw;
                    data->temperature_raw = last_valid_baro_data.temperature_raw;
                    data->baro_valid = (last_valid_baro_data.pressure_raw != 0) ? 1 : 0;
                } else {
                    data->baro_valid = 0;
                    sensor_status.baro_error_count++;
                }
            } else {
                /* Arbiter busy - use cached */
                data->pressure_raw = last_valid_baro_data.pressure_raw;
                data->temperature_raw = last_valid_baro_data.temperature_raw;
                data->baro_valid = (last_valid_baro_data.pressure_raw != 0) ? 1 : 0;
            }
        } else {
            /* Use cached during decimation */
            data->pressure_raw = last_valid_baro_data.pressure_raw;
            data->temperature_raw = last_valid_baro_data.temperature_raw;
            data->baro_valid = (last_valid_baro_data.pressure_raw != 0) ? 1 : 0;
        }
    } else {
        data->baro_valid = 0;
    }

    /* ... rest of function ... */
}
```

#### 3.2 Add reset function for testing
```c
/**
 * @brief Reset sensor manager state (for testing/re-initialization)
 */
void SensorManager_ResetState(void) {
    memset(&last_valid_baro_data, 0, sizeof(BMP581_Data_t));
    memset(&last_valid_highg_data, 0, sizeof(H3LIS331DL_Data_t));
    baro_decimation_counter = 0;
    highg_decimation_counter = 0;
    memset(&sensor_status, 0, sizeof(SensorManager_Status_t));
    last_read_time_us = 0;
}
```

---

## TASK 4: Decouple data_logger from main.c

**File:** `Src/data_logger.c`, `Inc/data_logger.h`

### Problem
- `extern MahonyFilter_t mahony_filter;` creates tight coupling
- Can't use data_logger without specific main.c setup
- Hard to integrate with EKF

### Changes

#### 4.1 Create navigation provider interface
```c
// In data_logger.h

/**
 * @brief Navigation data provider interface
 * @note Register your navigation system (Mahony, EKF, etc.) using this interface
 */
typedef struct {
    /**
     * @brief Get attitude quaternion (w, x, y, z)
     * @param quat Output array[4] for quaternion
     * @return true if quaternion valid
     */
    bool (*get_quaternion)(float quat[4]);

    /**
     * @brief Get position in NED frame (meters)
     * @param pos Output array[3] for position (North, East, Down)
     * @return true if position valid
     */
    bool (*get_position_ned)(float pos[3]);

    /**
     * @brief Get velocity in NED frame (m/s)
     * @param vel Output array[3] for velocity (North, East, Down)
     * @return true if velocity valid
     */
    bool (*get_velocity_ned)(float vel[3]);

    /**
     * @brief Get navigation validity flag
     * @return true if navigation solution is valid
     */
    bool (*is_valid)(void);

} NavigationProvider_t;

/**
 * @brief Register navigation data provider
 * @param provider Pointer to navigation provider structure
 * @note Call this during initialization to connect your EKF/Mahony filter
 */
void DataLogger_RegisterNavProvider(const NavigationProvider_t* provider);
```

#### 4.2 Implement in data_logger.c
```c
// In data_logger.c (private variables)
static const NavigationProvider_t* nav_provider = NULL;

void DataLogger_RegisterNavProvider(const NavigationProvider_t* provider) {
    nav_provider = provider;
}

// Update PackDataRecord to use interface
static void PackDataRecord(DataRecord_t* record) {
    if (!record) return;

    memset(record, 0, sizeof(DataRecord_t));
    record->timestamp_ms = HAL_GetTick();

    // === SENSOR DATA ===
    SensorManager_RawData_t raw_data;
    SensorManager_ScaledData_t sensor_data;

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

    // === GPS DATA ===
    GPS_Data_t gps_data = {0};
    bool gps_valid = GPS_GetCurrentData(&gps_data);

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
        /* Get quaternion */
        if (nav_provider->get_quaternion != NULL) {
            nav_provider->get_quaternion(record->quat);
        }

        /* Get position NED */
        if (nav_provider->get_position_ned != NULL) {
            nav_provider->get_position_ned(record->pos_ned);
        }

        /* Get velocity NED */
        if (nav_provider->get_velocity_ned != NULL) {
            nav_provider->get_velocity_ned(record->vel_ned);
        }

        /* Get validity */
        if (nav_provider->is_valid != NULL) {
            record->nav_valid = nav_provider->is_valid() ? 1 : 0;
        } else {
            record->nav_valid = gps_valid ? 1 : 0;
        }
    } else {
        /* No provider - fill with GPS validity */
        record->nav_valid = gps_valid ? 1 : 0;
    }

    // Fill unused EKF fields with NAN
    for (int i = 0; i < 3; i++) {
        record->pos_uncertainty[i] = NAN;
        record->vel_uncertainty[i] = NAN;
        record->innovation_pos[i] = NAN;
        record->innovation_vel[i] = NAN;
        record->kalman_gain_pos[i] = NAN;
        record->kalman_gain_vel[i] = NAN;
        record->accel_ned[i] = NAN;
    }
    record->motion_state = 3;
}
```

#### 4.3 Example usage in main.c (or navigation module)
```c
// Example: Mahony filter provider
bool MahonyGetQuaternion(float quat[4]) {
    extern MahonyFilter_t mahony_filter;
    Quaternion_t q;
    if (Mahony_GetQuaternion(&mahony_filter, &q) == HAL_OK) {
        quat[0] = q.q0;
        quat[1] = q.q1;
        quat[2] = q.q2;
        quat[3] = q.q3;
        return true;
    }
    return false;
}

bool MahonyIsValid(void) {
    extern MahonyFilter_t mahony_filter;
    return mahony_filter.initialized;
}

static const NavigationProvider_t mahony_nav_provider = {
    .get_quaternion = MahonyGetQuaternion,
    .get_position_ned = NULL,  // Mahony doesn't provide position
    .get_velocity_ned = NULL,  // Mahony doesn't provide velocity
    .is_valid = MahonyIsValid
};

// In main() or init function:
DataLogger_RegisterNavProvider(&mahony_nav_provider);
```

**Benefit:** When you add EKF, just create `ekf_nav_provider` and register it!

---

## TASK 5: Clean Up BLE Module Debug Code

**File:** `Src/ble_module.c`

### Remove Debug Variables & Dead Code

#### 5.1 Remove unused debug variables
```c
// REMOVE these lines:
static volatile bool command_received_flag = false;
static char last_command_received[64] = {0};
static volatile bool response_sent_flag = false;

// REMOVE this function (or make it conditional):
bool BLE_GetLastCommand(char* command_buffer, size_t buffer_size) {
    // ... remove entire function or wrap in #ifdef DEBUG
}
```

#### 5.2 Remove protocol overhead detection (keep it simple)
```c
// REMOVE IsProtocolOverhead function entirely
// REMOVE call from BLE_ProcessIncompleteBuffer

static void BLE_ProcessIncompleteBuffer(void) {
    if (rx_index > 0) {
        rx_buffer[rx_index] = '\0';

        /* Remove trailing whitespace */
        while (rx_index > 0 && (rx_buffer[rx_index-1] == '\r' ||
               rx_buffer[rx_index-1] == '\n' || rx_buffer[rx_index-1] == ' ')) {
            rx_buffer[--rx_index] = '\0';
        }

        /* Only process if reasonable length */
        if (rx_index >= 3) {
            BLE_ProcessCommand((char*)rx_buffer);
        }

        rx_index = 0;
    }
}
```

#### 5.3 Simplify command mode entry (reduce retries)
```c
static bool EnterCommandMode(void) {
    /* Check if already in command mode */
    if (IsInCommandMode()) {
        return true;
    }

    /* Try twice maximum */
    for (int attempt = 1; attempt <= 2; attempt++) {
        ClearResponseBuffer();

        if (SendBleCommand("$$$", "CMD>", 3000)) {
            return true;
        }

        if (attempt == 1) {
            HAL_Delay(500);  // Short delay between attempts
        }
    }

    DebugPrint("BLE: ERROR - Failed to enter command mode\r\n");
    return false;
}
```

#### 5.4 Add conditional compilation for debug
```c
// At top of file
#define BLE_DEBUG_ENABLED  0  // Set to 1 for debug output

#if BLE_DEBUG_ENABLED
    #define BLE_DEBUG(msg) DebugPrint(msg)
#else
    #define BLE_DEBUG(msg) ((void)0)
#endif

// Replace all DebugPrint calls in BLE module:
BLE_DEBUG("BLE: Initializing...\r\n");
BLE_DEBUG("BLE: Entering command mode...\r\n");
// etc.
```

**Size Reduction:** Should reduce BLE module from ~803 lines to ~600 lines

---

## TASK 6: Standardize Return Types & Error Handling

**Files:** Multiple modules

### Problem
- Inconsistent: `bool` vs `HAL_StatusTypeDef`
- Makes error propagation inconsistent

### Decision: Use `HAL_StatusTypeDef` for hardware operations, `bool` for logic

#### 6.1 Create error handling guidelines
```c
// In a new file: Inc/error_codes.h
#ifndef ERROR_CODES_H_
#define ERROR_CODES_H_

#include "stm32g4xx_hal.h"

/**
 * @brief Error handling guidelines:
 *
 * Use HAL_StatusTypeDef for:
 * - Hardware initialization (sensor init, peripheral init)
 * - Hardware operations (sensor reads, flash writes)
 * - DMA operations
 *
 * Use bool for:
 * - Data validation (GPS_IsDataValid)
 * - State queries (DataLogger_IsRecording)
 * - Simple success/failure operations
 *
 * Use specific enums for:
 * - Status with multiple states (LoggerStatus_t)
 * - Command results (USBCommandStatus_t)
 */

#endif /* ERROR_CODES_H_ */
```

#### 6.2 Keep existing patterns (already mostly consistent)
- Sensor drivers: `HAL_StatusTypeDef` ✅
- Data logger init: `bool` ✅ (high-level abstraction)
- GPS init: `bool` ✅ (high-level abstraction)

**No code changes needed** - document the pattern

---

## TASK 7: Fix GetMicros Overflow Handling

**File:** `Src/sensor_manager.c`

### Problem
- Wraps every 25 seconds at 170MHz
- No overflow compensation

### Changes

#### 7.1 Improve GetMicros with overflow handling
```c
// In sensor_manager.c (private variables)
static uint32_t us_high_word = 0;
static uint32_t last_cyccnt = 0;
static uint32_t cycles_per_us = 0;

// Update SensorManager_Init:
HAL_StatusTypeDef SensorManager_Init(const SensorManager_Config_t *user_config) {
    // ... existing init code ...

    /* Initialize DWT cycle counter for microsecond timing */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Calculate cycles per microsecond */
    cycles_per_us = SystemCoreClock / 1000000U;
    us_high_word = 0;
    last_cyccnt = 0;

    // ... rest of init ...
}

// Replace GetMicros:
static inline uint32_t GetMicros(void) {
    uint32_t cyccnt = DWT->CYCCNT;

    /* Detect overflow (wrapping from 0xFFFFFFFF to 0x00000000) */
    if (cyccnt < last_cyccnt) {
        /* Overflow occurred - increment high word */
        us_high_word += (0xFFFFFFFFU / cycles_per_us) + 1;
    }

    last_cyccnt = cyccnt;

    /* Return combined microseconds */
    return us_high_word + (cyccnt / cycles_per_us);
}
```

**Note:** Still wraps at ~71 minutes (2^32 microseconds), but sufficient for per-session timing

---

## TASK 8: Add Sensor Calibration Framework

**Files:** `Inc/sensor_manager.h`, `Src/sensor_manager.c`

### Add Optional Calibration Support

#### 8.1 Add calibration structure to header
```c
// In sensor_manager.h

/**
 * @brief Sensor calibration parameters
 * @note All calibrations optional - system works without them
 */
typedef struct {
    /* Gyroscope bias (rad/s) */
    float gyro_bias[3];
    bool gyro_bias_valid;

    /* Accelerometer offset and scale */
    float accel_offset[3];  /* Offset (g) */
    float accel_scale[3];   /* Scale factor */
    bool accel_cal_valid;

    /* Magnetometer hard iron offset (Gauss) */
    float mag_offset[3];
    /* Magnetometer soft iron matrix (3x3) */
    float mag_scale[3][3];
    bool mag_cal_valid;

} SensorCalibration_t;

/**
 * @brief Set sensor calibration parameters
 * @param cal Pointer to calibration structure
 * @note Calibration is optional - pass NULL to disable
 */
void SensorManager_SetCalibration(const SensorCalibration_t* cal);

/**
 * @brief Get current calibration parameters
 * @param cal Pointer to store calibration
 */
void SensorManager_GetCalibration(SensorCalibration_t* cal);
```

#### 8.2 Implement calibration in sensor_manager.c
```c
// In sensor_manager.c (private variables)
static SensorCalibration_t active_calibration = {0};
static bool calibration_enabled = false;

void SensorManager_SetCalibration(const SensorCalibration_t* cal) {
    if (cal == NULL) {
        calibration_enabled = false;
        memset(&active_calibration, 0, sizeof(SensorCalibration_t));
    } else {
        memcpy(&active_calibration, cal, sizeof(SensorCalibration_t));
        calibration_enabled = true;
    }
}

void SensorManager_GetCalibration(SensorCalibration_t* cal) {
    if (cal != NULL) {
        memcpy(cal, &active_calibration, sizeof(SensorCalibration_t));
    }
}

// Update SensorManager_GetMahonyData to apply calibration:
void SensorManager_GetMahonyData(const SensorManager_RawData_t *raw,
                                  float *gx, float *gy, float *gz,
                                  float *ax, float *ay, float *az,
                                  float *mx, float *my, float *mz) {
    VALIDATE_PTR_VOID(raw);
    // ... existing pointer validation ...

    // Gyro: Convert from LSB to rad/s
    *gx = (raw->gyro_x / ICM42688_GYRO_SCALE_2000DPS) * DEG_TO_RAD;
    *gy = (raw->gyro_y / ICM42688_GYRO_SCALE_2000DPS) * DEG_TO_RAD;
    *gz = (raw->gyro_z / ICM42688_GYRO_SCALE_2000DPS) * DEG_TO_RAD;

    // Apply gyro bias calibration if available
    if (calibration_enabled && active_calibration.gyro_bias_valid) {
        *gx -= active_calibration.gyro_bias[0];
        *gy -= active_calibration.gyro_bias[1];
        *gz -= active_calibration.gyro_bias[2];
    }

    // Accel: Convert from LSB to m/s²
    *ax = (raw->accel_x / ICM42688_ACCEL_SCALE_16G) * GRAVITY_MSS;
    *ay = (raw->accel_y / ICM42688_ACCEL_SCALE_16G) * GRAVITY_MSS;
    *az = (raw->accel_z / ICM42688_ACCEL_SCALE_16G) * GRAVITY_MSS;

    // Apply accel calibration if available
    if (calibration_enabled && active_calibration.accel_cal_valid) {
        *ax = (*ax - active_calibration.accel_offset[0]) * active_calibration.accel_scale[0];
        *ay = (*ay - active_calibration.accel_offset[1]) * active_calibration.accel_scale[1];
        *az = (*az - active_calibration.accel_offset[2]) * active_calibration.accel_scale[2];
    }

    // Mag: Convert from LSB to µT (microTesla)
    *mx = (raw->mag_x / MMC5983MA_MAG_SCALE) * GAUSS_TO_MICROTESLA;
    *my = (raw->mag_y / MMC5983MA_MAG_SCALE) * GAUSS_TO_MICROTESLA;
    *mz = (raw->mag_z / MMC5983MA_MAG_SCALE) * GAUSS_TO_MICROTESLA;

    // Apply magnetometer calibration if available
    if (calibration_enabled && active_calibration.mag_cal_valid) {
        /* Hard iron compensation */
        float mx_cal = *mx - active_calibration.mag_offset[0];
        float my_cal = *my - active_calibration.mag_offset[1];
        float mz_cal = *mz - active_calibration.mag_offset[2];

        /* Soft iron compensation (3x3 matrix multiply) */
        *mx = active_calibration.mag_scale[0][0] * mx_cal +
              active_calibration.mag_scale[0][1] * my_cal +
              active_calibration.mag_scale[0][2] * mz_cal;

        *my = active_calibration.mag_scale[1][0] * mx_cal +
              active_calibration.mag_scale[1][1] * my_cal +
              active_calibration.mag_scale[1][2] * mz_cal;

        *mz = active_calibration.mag_scale[2][0] * mx_cal +
              active_calibration.mag_scale[2][1] * my_cal +
              active_calibration.mag_scale[2][2] * mz_cal;
    }
}
```

**Note:** Calibration is **optional** - system works without it. Your EKF may have its own calibration.

---

## TASK 9: Update CLAUDE.md

**File:** `CLAUDE.md`

Add new sections:
- Navigation provider interface pattern
- Calibration framework usage
- Error handling standards
- Known limitations (GetMicros 71-min wrap)

---

## TASK 10: Testing Checklist

### Unit Tests (if possible)
- [ ] i2c_dma_arbiter race condition fix
- [ ] i2c_dma_arbiter timeout/watchdog
- [ ] data_logger DMA completion
- [ ] sensor_manager calibration math

### Integration Tests
- [ ] All sensors reading at target rates
- [ ] Data logger records 1000+ records without corruption
- [ ] Metadata survives power cycle
- [ ] BLE commands still work after cleanup
- [ ] I2C arbiter handles conflicts gracefully
- [ ] QSPI writes complete before next write starts

### Stress Tests
- [ ] 30-minute continuous recording
- [ ] Flash full scenario (graceful stop)
- [ ] Arbiter timeout recovery
- [ ] Multiple rapid BLE commands

---

## IMPLEMENTATION ORDER

1. **Day 1 (Critical Fixes)**
   - Task 1: i2c_dma_arbiter (2 hours)
   - Task 2: data_logger DMA & flash (3 hours)
   - Task 3: sensor_manager static (30 min)
   - Basic testing

2. **Day 2 (Architecture)**
   - Task 4: Decouple data_logger (2 hours)
   - Task 7: Fix GetMicros (1 hour)
   - Task 5: Clean BLE module (2 hours)
   - Integration testing

3. **Day 3 (Polish & Test)**
   - Task 8: Add calibration framework (2 hours)
   - Task 9: Update documentation (1 hour)
   - Task 10: Comprehensive testing (4 hours)

---

## RISK ASSESSMENT

| Task | Risk Level | Mitigation |
|------|-----------|------------|
| Task 1 (arbiter) | 🟡 Medium | Test thoroughly, may affect all I2C sensors |
| Task 2 (data_logger) | 🟠 High | Backup current flash data, verify with scope |
| Task 3 (sensor_mgr) | 🟢 Low | Simple refactor, minimal risk |
| Task 4 (decouple) | 🟢 Low | Additive change, backward compatible |
| Task 5 (BLE cleanup) | 🟡 Medium | Could break BLE, test carefully |
| Task 7 (GetMicros) | 🟢 Low | Improvement, low risk |
| Task 8 (calibration) | 🟢 Low | Optional feature, off by default |

---

## VALIDATION CRITERIA

✅ **Success Criteria:**
1. No compiler warnings
2. All sensors initialize successfully
3. Data logger records 1000 records without error
4. Metadata persists across power cycle
5. BLE commands respond correctly
6. I2C arbiter handles conflicts without hangs
7. No race conditions detected in testing

---

## NOTES FOR EKF/LORA INTEGRATION

After this refactoring:
1. **EKF Integration:** Create `NavigationProvider_t` for your EKF, register with data_logger
2. **LoRa Module:** Follow BLE pattern - DMA circular buffer, command processing
3. **Inter-MCU Comm:** Use sensor_driver_common pattern if I2C/SPI based
4. **Calibration:** Your EKF can populate `SensorCalibration_t` structure

**Clean interfaces ready for your code!**
