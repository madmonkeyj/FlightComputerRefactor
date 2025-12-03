/**
  ******************************************************************************
  * @file    error_codes.h
  * @brief   Standard error codes and error handling patterns for FlightComputer
  * @note    Defines consistent error reporting across all modules
  * @note    Use these patterns to ensure uniform error handling
  ******************************************************************************
  */

#ifndef ERROR_CODES_H_
#define ERROR_CODES_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * STANDARD RETURN VALUE CONVENTIONS
 * ============================================================================
 * All modules should follow these conventions for consistency:
 *
 * 1. HAL Functions: Return HAL_StatusTypeDef
 *    - HAL_OK (0)       - Success
 *    - HAL_ERROR (1)    - Generic error
 *    - HAL_BUSY (2)     - Resource busy (critical for DMA operations)
 *    - HAL_TIMEOUT (3)  - Operation timeout
 *
 * 2. Boolean Functions: Return bool
 *    - true  - Success or condition met
 *    - false - Failure or condition not met
 *
 * 3. Functions Returning Data: Use pointer output parameter + bool/HAL return
 *    Example: bool GetData(DataStruct_t* output);
 *
 * 4. Never use negative error codes - embedded systems may not handle signed comparison correctly
 */

/* ============================================================================
 * MODULE-SPECIFIC ERROR CODES
 * ============================================================================
 * Each major module defines its own error codes starting from 0x1000 offset
 * This prevents collisions and allows quick identification of error source
 */

/* === I2C DMA ARBITER ERROR CODES === */
typedef enum {
    I2C_ARBITER_OK           = 0x0000,    /* Success */
    I2C_ARBITER_BUSY         = 0x1001,    /* Arbiter currently busy with another device */
    I2C_ARBITER_TIMEOUT      = 0x1002,    /* DMA transfer timeout (>100ms) */
    I2C_ARBITER_DMA_ERROR    = 0x1003,    /* DMA transfer failed */
    I2C_ARBITER_NULL_PTR     = 0x1004,    /* NULL pointer passed to function */
    I2C_ARBITER_INVALID_DEV  = 0x1005     /* Invalid device ID */
} I2C_Arbiter_ErrorCode_t;

/* === DATA LOGGER ERROR CODES === */
typedef enum {
    LOGGER_OK                = 0x0000,    /* Success */
    LOGGER_NOT_INITIALIZED   = 0x2001,    /* Flash not initialized */
    LOGGER_FLASH_FULL        = 0x2002,    /* Flash memory full */
    LOGGER_FLASH_ERROR       = 0x2003,    /* QSPI flash error */
    LOGGER_DMA_BUSY          = 0x2004,    /* Previous DMA write still in progress */
    LOGGER_DMA_TIMEOUT       = 0x2005,    /* DMA write timeout (>100ms) */
    LOGGER_METADATA_INVALID  = 0x2006,    /* Metadata checksum failed */
    LOGGER_WRONG_STATE       = 0x2007,    /* Operation not allowed in current state */
    LOGGER_NULL_PTR          = 0x2008     /* NULL pointer passed to function */
} DataLogger_ErrorCode_t;

/* === SENSOR MANAGER ERROR CODES === */
typedef enum {
    SENSOR_OK                = 0x0000,    /* Success */
    SENSOR_NOT_INITIALIZED   = 0x3001,    /* Sensor not initialized */
    SENSOR_TIMEOUT           = 0x3002,    /* Sensor read timeout */
    SENSOR_DMA_BUSY          = 0x3003,    /* DMA transfer already in progress */
    SENSOR_INVALID_DATA      = 0x3004,    /* Sensor data validation failed */
    SENSOR_NULL_PTR          = 0x3005,    /* NULL pointer passed to function */
    SENSOR_CONFIG_ERROR      = 0x3006     /* Sensor configuration failed */
} SensorManager_ErrorCode_t;

/* === GPS MODULE ERROR CODES === */
typedef enum {
    GPS_OK                   = 0x0000,    /* Success */
    GPS_NO_FIX               = 0x4001,    /* GPS has no valid fix */
    GPS_STALE_DATA           = 0x4002,    /* GPS data too old (>5s) */
    GPS_CHECKSUM_FAIL        = 0x4003,    /* UBX checksum validation failed */
    GPS_PARSE_ERROR          = 0x4004,    /* UBX message parsing error */
    GPS_BUFFER_OVERFLOW      = 0x4005,    /* DMA circular buffer overflow */
    GPS_NULL_PTR             = 0x4006     /* NULL pointer passed to function */
} GPS_ErrorCode_t;

/* === BLE MODULE ERROR CODES === */
typedef enum {
    BLE_OK                   = 0x0000,    /* Success */
    BLE_NOT_INITIALIZED      = 0x5001,    /* BLE module not initialized */
    BLE_NOT_CONNECTED        = 0x5002,    /* No BLE device connected */
    BLE_TX_TIMEOUT           = 0x5003,    /* UART transmit timeout */
    BLE_CONFIG_FAILED        = 0x5004,    /* BLE configuration failed */
    BLE_CMD_MODE_FAILED      = 0x5005,    /* Failed to enter command mode */
    BLE_NULL_PTR             = 0x5006     /* NULL pointer passed to function */
} BLE_ErrorCode_t;

/* === MAHONY FILTER ERROR CODES === */
typedef enum {
    MAHONY_OK                = 0x0000,    /* Success */
    MAHONY_NOT_INITIALIZED   = 0x6001,    /* Filter not initialized */
    MAHONY_INVALID_GAINS     = 0x6002,    /* Invalid Kp or Ki values */
    MAHONY_INVALID_RATE      = 0x6003,    /* Invalid sample rate */
    MAHONY_NULL_PTR          = 0x6004     /* NULL pointer passed to function */
} Mahony_ErrorCode_t;

/* ============================================================================
 * ERROR HANDLING PATTERNS
 * ============================================================================ */

/**
 * PATTERN 1: Function parameter validation
 * ========================================
 * ALWAYS validate pointer parameters before dereferencing
 *
 * Example:
 *   bool MyFunction(MyStruct_t* output) {
 *       if (!output) return false;
 *       // ... rest of function
 *   }
 */

/**
 * PATTERN 2: DMA busy checking with timeout
 * ==========================================
 * When using DMA, ALWAYS check busy flag with timeout
 *
 * Example:
 *   uint32_t start = HAL_GetTick();
 *   while (*dma_busy_flag) {
 *       if ((HAL_GetTick() - start) > TIMEOUT_MS) {
 *           return HAL_TIMEOUT;
 *       }
 *   }
 */

/**
 * PATTERN 3: Critical sections for shared resources
 * ==================================================
 * Use atomic check-and-set for shared state variables
 *
 * Example:
 *   __disable_irq();
 *   if (resource_busy) {
 *       __enable_irq();
 *       return HAL_BUSY;
 *   }
 *   resource_busy = true;
 *   __enable_irq();
 */

/**
 * PATTERN 4: Validity flags for sensor data
 * ==========================================
 * ALWAYS check validity flags before using sensor data
 *
 * Example:
 *   SensorManager_RawData_t raw_data;
 *   SensorManager_ReadRaw(&raw_data);
 *   if (raw_data.imu_valid) {
 *       // Use IMU data
 *   }
 */

/**
 * PATTERN 5: Graceful degradation on sensor failure
 * ==================================================
 * System should continue operating even if sensors fail
 *
 * Example:
 *   if (MAG_Init() != HAL_OK) {
 *       // Log error but continue
 *       mag_available = false;
 *       // Mahony filter can work without magnetometer
 *   }
 */

/**
 * PATTERN 6: Metadata validation with checksums
 * ==============================================
 * Critical data structures should have checksums
 *
 * Example:
 *   if (!Metadata_Validate(&loaded_metadata)) {
 *       // Metadata corrupt, use defaults
 *       Metadata_Clear();
 *   }
 */

/**
 * PATTERN 7: Flash wear management
 * ================================
 * Use dirty flags to reduce flash write cycles
 *
 * Example:
 *   metadata_dirty = true;  // Mark dirty instead of immediate save
 *
 *   // In periodic update function:
 *   if (metadata_dirty && (HAL_GetTick() - last_save > INTERVAL)) {
 *       Metadata_Save();
 *       metadata_dirty = false;
 *   }
 */

/**
 * PATTERN 8: Timeout handling for blocking operations
 * ====================================================
 * NEVER block indefinitely - always use timeouts
 *
 * Example:
 *   uint32_t start = HAL_GetTick();
 *   while (condition) {
 *       if ((HAL_GetTick() - start) > TIMEOUT_MS) {
 *           return HAL_TIMEOUT;
 *       }
 *       // Brief delay to avoid busy-waiting
 *       HAL_Delay(1);
 *   }
 */

/**
 * PATTERN 9: State machine error handling
 * ========================================
 * Check state before operations, transition on errors
 *
 * Example:
 *   if (current_state != EXPECTED_STATE) {
 *       return ERROR_WRONG_STATE;
 *   }
 *   // Perform operation
 *   if (operation_failed) {
 *       current_state = ERROR_STATE;
 *       return ERROR_CODE;
 *   }
 */

/**
 * PATTERN 10: Error counters for diagnostics
 * ===========================================
 * Track errors for debugging and health monitoring
 *
 * Example:
 *   typedef struct {
 *       uint32_t imu_error_count;
 *       uint32_t mag_error_count;
 *       uint32_t timeout_count;
 *   } SensorManager_Status_t;
 *
 *   // In error path:
 *   status.imu_error_count++;
 */

/* ============================================================================
 * ASSERTIONS AND DEBUG
 * ============================================================================ */

/**
 * Compile-time assertions for structure sizes
 * Use _Static_assert to catch alignment issues at compile time
 */
#define ASSERT_STRUCT_SIZE(type, expected_size) \
    _Static_assert(sizeof(type) == (expected_size), #type " must be exactly " #expected_size " bytes")

/**
 * Example usage in data_logger.h:
 *   ASSERT_STRUCT_SIZE(DataRecord_t, 192);
 */

/**
 * Runtime assertions (only in debug builds)
 * For production, these should be disabled to save code space
 */
#ifdef DEBUG
    #define RUNTIME_ASSERT(condition) \
        do { \
            if (!(condition)) { \
                while(1); /* Hang in debug mode */ \
            } \
        } while(0)
#else
    #define RUNTIME_ASSERT(condition) ((void)0)
#endif

/* ============================================================================
 * COMMON ERROR SCENARIOS AND SOLUTIONS
 * ============================================================================ */

/**
 * SCENARIO 1: I2C Bus Conflicts
 * ==============================
 * Problem: Multiple sensors on same I2C bus trying to access simultaneously
 * Solution: Use I2C DMA Arbiter with priority-based scheduling
 * See: i2c_dma_arbiter.h for implementation
 */

/**
 * SCENARIO 2: DMA Race Conditions
 * ================================
 * Problem: Code increments address before DMA write completes
 * Solution: Use volatile busy flags and wait for completion with timeout
 * See: data_logger.c DataLogger_RecordData() for example
 */

/**
 * SCENARIO 3: Flash Wear from Excessive Writes
 * =============================================
 * Problem: Writing metadata too frequently wears out flash
 * Solution: Use dirty flag + periodic save (e.g., every 10 seconds)
 * See: data_logger.c DataLogger_Update() for example
 */

/**
 * SCENARIO 4: Function-Scope Static Variables
 * ============================================
 * Problem: Static variables in function scope are hard to test/reset
 * Solution: Move to file scope with explicit reset function
 * See: sensor_manager.c SensorManager_ResetState() for example
 */

/**
 * SCENARIO 5: Tight Coupling to Specific Modules
 * ===============================================
 * Problem: Module has extern dependencies on other modules
 * Solution: Use provider pattern with function pointers
 * See: data_logger.h NavigationProvider_t for example
 */

/**
 * SCENARIO 6: Stale Sensor Data
 * ==============================
 * Problem: Using old sensor data that's no longer valid
 * Solution: Check timestamp and validity flags
 * Example:
 *   if ((HAL_GetTick() - gps_data.last_update) > 5000) {
 *       // Data is stale (>5 seconds old)
 *       return GPS_STALE_DATA;
 *   }
 */

/**
 * SCENARIO 7: Integer Overflow in Time Calculations
 * ==================================================
 * Problem: HAL_GetTick() wraps around after ~49 days
 * Solution: Use unsigned arithmetic (wraps correctly)
 * Example:
 *   uint32_t elapsed = HAL_GetTick() - start_time;
 *   // This works correctly even if HAL_GetTick() wrapped
 */

/* ============================================================================
 * BEST PRACTICES SUMMARY
 * ============================================================================ */

/**
 * 1. Always validate pointers before dereferencing
 * 2. Always use timeouts for blocking operations
 * 3. Always check HAL return values (especially HAL_BUSY for DMA)
 * 4. Always check validity flags before using sensor data
 * 5. Always use critical sections for shared state modifications
 * 6. Always implement graceful degradation on sensor failures
 * 7. Always validate checksums for persistent data
 * 8. Always use dirty flags to reduce flash writes
 * 9. Always track error counts for diagnostics
 * 10. Always use compile-time assertions for critical struct sizes
 */

#endif /* ERROR_CODES_H_ */
