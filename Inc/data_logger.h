/**
  ******************************************************************************
  * @file    data_logger.h
  * @brief   Data logging module header - 192-BYTE STRUCTURE
  * @note    Adapted for current FlightComputer codebase (no EKF)
  * @note    Uses SensorManager, Mahony filter, and GPS module
  ******************************************************************************
  */

#ifndef DATA_LOGGER_H_
#define DATA_LOGGER_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* === CONSTANTS === */
#define RECORD_SIZE            192          // Optimized record size
#define CURRENT_RECORD_SIZE    192          // Alias for compatibility

/* === FLASH LAYOUT === */
#define METADATA_MAGIC          0xDEADBEEF
#define METADATA_VERSION        2
#define METADATA_SECTOR_SIZE    4096
#define METADATA_SECTOR_ADDR    (4 * 1024 * 1024 - METADATA_SECTOR_SIZE)  // Last 4KB
#define DATA_AREA_SIZE          METADATA_SECTOR_ADDR                       // Data area

/* === FORCE BYTE PACKING === */
#pragma pack(push, 1)

/**
 * @brief Optimized flight data record with EKF debugging - EXACTLY 192 bytes
 */
typedef struct {
    // === HEADER === (4 bytes)
    uint32_t timestamp_ms;       // HAL_GetTick()

    // === CRITICAL SENSOR DATA === (24 bytes) - Reduced from 52
    float accel[3];              // m/s² - Raw accelerometer (keep for coordinate transform verification)
    float gyro[3];               // rad/s - Raw gyroscope (keep for motion detection)
    // REMOVED: mag[3], pressure, temperature, battery_voltage

    // === BASIC GPS DATA === (18 bytes) - Reduced from 22
    float gps_lat;               // degrees - Latitude
    float gps_lon;               // degrees - Longitude
    float gps_alt;               // m - Altitude MSL
    float gps_speed;             // m/s - Ground speed
    uint8_t gps_satellites;      // count - Satellite count
    char gps_fix_status;         // 'A'=valid, 'V'=invalid
    // REMOVED: gps_heading (can derive from velocity)

    // === ESSENTIAL GPS QUALITY === (4 bytes) - Reduced from 8
    float gps_hdop;              // Horizontal DOP (critical for debugging)
    // REMOVED: gps_accuracy (can derive from other data)

    // === GPS VELOCITY COMPONENTS === (12 bytes) - Expanded from 8
    float gps_vel_n;             // m/s - North velocity
    float gps_vel_e;             // m/s - East velocity
    float gps_vel_d;             // m/s - Down velocity (added for complete debugging)

    // === BASIC NAVIGATION DATA === (36 bytes) - Reduced from 52
    float quat[4];               // w,x,y,z - Attitude quaternion
    float pos_ned[3];            // m - EKF position NED
    float vel_ned[3];            // m/s - EKF velocity NED
    // REMOVED: euler angles (derive from quat), ground_speed, heading_deg, climb_rate (derive from vel_ned)

    // === SYSTEM HEALTH === (3 bytes)
    uint8_t nav_valid;           // 1=Navigation valid, 0=invalid

    // === EKF UNCERTAINTY ESTIMATES === (24 bytes) - Kept same
    float pos_uncertainty[3];    // m - Position std dev NED
    float vel_uncertainty[3];    // m/s - Velocity std dev NED

    // === **NEW: INNOVATION VALUES** === (24 bytes) - **MOST CRITICAL**
    float innovation_pos[3];     // m - GPS position innovation (GPS - EKF_predicted)
    float innovation_vel[3];     // m/s - GPS velocity innovation (GPS - EKF_predicted)

    // === **NEW: KALMAN GAINS** === (24 bytes) - Shows correction magnitudes
    float kalman_gain_pos[3];    // Kalman gain for position measurements
    float kalman_gain_vel[3];    // Kalman gain for velocity measurements

    // === **NEW: COORDINATE TRANSFORM RESULTS** === (12 bytes) - Verifies -1.0f fix
    float accel_ned[3];          // m/s² - Accelerometer data transformed to NED

    // === **NEW: REJECTION COUNTERS** === (3 bytes) - Shows measurement rejections
    uint8_t gps_pos_rejected;    // GPS position measurement rejected this cycle
    uint8_t gps_vel_rejected;    // GPS velocity measurement rejected this cycle
    uint8_t zupt_applied;        // ZUPT applied this cycle

    // === REDUCED MOTION DETECTION === (4 bytes) - Reduced from 11
    uint8_t motion_state;        // 0=STAT,1=SLOW,2=FAST,3=UNKNOWN
    uint8_t gps_velocity_suspect; // 1=GPS velocity suspect, 0=trusted

} DataRecord_t;

#pragma pack(pop)

/* === FLASH METADATA === */
#pragma pack(push, 1)
typedef struct {
    uint32_t magic_number;          // METADATA_MAGIC
    uint32_t version;               // METADATA_VERSION
    uint32_t records_written;       // Number of records written
    uint32_t current_write_address; // Current write position
    uint32_t recording_start_time;  // HAL_GetTick() at start
    uint32_t last_record_time;      // HAL_GetTick() of last record
    uint8_t logger_status;          // LoggerStatus_t
    uint32_t record_size;           // Always RECORD_SIZE (192)
    uint32_t recording_session_id;  // Session counter
    uint32_t checksum;              // Validation checksum
} FlashMetadata_t;
#pragma pack(pop)

/* === LOGGER STATUS === */
typedef enum {
    LOGGER_IDLE = 0,
    LOGGER_RECORDING,
    LOGGER_DOWNLOADING,
    LOGGER_ERROR
} LoggerStatus_t;

typedef struct {
    LoggerStatus_t status;
    uint32_t records_written;
    uint32_t flash_bytes_used;
    uint32_t recording_start_time;
    uint32_t last_record_time;
    uint32_t recording_rate_hz;
    uint32_t estimated_time_remaining_ms;
    bool flash_ready;
    uint32_t record_size;           // Always 192
} LoggerStats_t;

/* === NAVIGATION PROVIDER INTERFACE === */

/**
 * @brief Navigation data provider interface
 * @note Register your navigation system (Mahony, EKF, etc.) using this interface
 * @note This decouples data_logger from specific navigation implementations
 * @note Optional callbacks can be NULL if not available (will log NAN/0)
 */
typedef struct {
    // === BASIC NAVIGATION DATA (Required) ===
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

    // === EKF UNCERTAINTY ESTIMATES (Optional - can be NULL) ===
    /**
     * @brief Get position uncertainty (standard deviation)
     * @param uncertainty Output array[3] for position std dev (N, E, D) in meters
     * @return true if uncertainty valid
     */
    bool (*get_position_uncertainty)(float uncertainty[3]);

    /**
     * @brief Get velocity uncertainty (standard deviation)
     * @param uncertainty Output array[3] for velocity std dev (N, E, D) in m/s
     * @return true if uncertainty valid
     */
    bool (*get_velocity_uncertainty)(float uncertainty[3]);

    // === EKF INNOVATION VALUES (Optional - can be NULL) ===
    /**
     * @brief Get GPS position innovation (GPS measurement - EKF prediction)
     * @param innovation Output array[3] for innovation (N, E, D) in meters
     * @return true if innovation valid
     */
    bool (*get_innovation_position)(float innovation[3]);

    /**
     * @brief Get GPS velocity innovation (GPS measurement - EKF prediction)
     * @param innovation Output array[3] for innovation (N, E, D) in m/s
     * @return true if innovation valid
     */
    bool (*get_innovation_velocity)(float innovation[3]);

    // === KALMAN GAINS (Optional - can be NULL) ===
    /**
     * @brief Get Kalman gain for position measurements
     * @param gain Output array[3] for Kalman gains (N, E, D)
     * @return true if gain valid
     */
    bool (*get_kalman_gain_position)(float gain[3]);

    /**
     * @brief Get Kalman gain for velocity measurements
     * @param gain Output array[3] for Kalman gains (N, E, D)
     * @return true if gain valid
     */
    bool (*get_kalman_gain_velocity)(float gain[3]);

    // === COORDINATE TRANSFORMS (Optional - can be NULL) ===
    /**
     * @brief Get accelerometer data transformed to NED frame
     * @param accel_ned Output array[3] for accel (N, E, D) in m/s²
     * @return true if transform valid
     */
    bool (*get_accel_ned)(float accel_ned[3]);

    // === REJECTION FLAGS (Optional - can be NULL) ===
    /**
     * @brief Get measurement rejection flags
     * @param gps_pos_rejected Output: 1 if GPS position rejected this cycle
     * @param gps_vel_rejected Output: 1 if GPS velocity rejected this cycle
     * @param zupt_applied Output: 1 if ZUPT applied this cycle
     * @return true if flags valid
     */
    bool (*get_rejection_flags)(uint8_t* gps_pos_rejected, uint8_t* gps_vel_rejected, uint8_t* zupt_applied);

    // === MOTION STATE (Optional - can be NULL) ===
    /**
     * @brief Get motion state and GPS velocity quality
     * @param motion_state Output: 0=STATIONARY, 1=SLOW, 2=FAST, 3=UNKNOWN
     * @param gps_velocity_suspect Output: 1 if GPS velocity suspect
     * @return true if state valid
     */
    bool (*get_motion_state)(uint8_t* motion_state, uint8_t* gps_velocity_suspect);

} NavigationProvider_t;

/* === FUNCTION PROTOTYPES === */

/**
 * @brief Register navigation data provider
 * @param provider Pointer to navigation provider structure
 * @note Call this during initialization to connect your EKF/Mahony filter
 * @note Provider can be NULL to disable navigation data logging
 */
void DataLogger_RegisterNavProvider(const NavigationProvider_t* provider);

// Core logging functions
bool DataLogger_Init(void);
bool DataLogger_StartRecording(void);
bool DataLogger_StopRecording(void);
bool DataLogger_RecordData(void);  // Reads from SensorManager, Mahony, GPS directly
void DataLogger_Update(void);      // Call from main loop for periodic metadata saves

// QSPI DMA callbacks (call from QSPI HAL callbacks)
void DataLogger_QSPI_WriteComplete(void);  // Call when QSPI write DMA completes
void DataLogger_QSPI_WriteError(void);      // Call when QSPI write DMA fails

// Status and control
bool DataLogger_GetStats(LoggerStats_t* stats);
bool DataLogger_GetStatusString(char* buffer, size_t buffer_size);
bool DataLogger_IsRecording(void);
bool DataLogger_IsDownloading(void);
bool DataLogger_EraseAll(void);

// Metadata management
bool Metadata_Save(void);
bool Metadata_Load(void);
void Metadata_Clear(void);
bool Metadata_Validate(const FlashMetadata_t* metadata);
uint32_t Metadata_CalculateChecksum(const FlashMetadata_t* metadata);

// Compile-time size verification
_Static_assert(sizeof(DataRecord_t) == RECORD_SIZE, "DataRecord_t must be exactly 192 bytes");

#endif /* DATA_LOGGER_H_ */
