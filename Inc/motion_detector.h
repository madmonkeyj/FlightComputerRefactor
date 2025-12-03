/**
 ******************************************************************************
 * @file    motion_detector.h
 * @brief   Motion Detection Module - Extracted from Navigation EKF
 * @note    Consolidates all motion state detection and filtering logic
 ******************************************************************************
 */

#ifndef MOTION_DETECTOR_H
#define MOTION_DETECTOR_H

#include "main.h"
#include "gps_module.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Motion detection filter size */
#define MOTION_FILTER_SIZE 2
#define GPS_SPEED_HISTORY_SIZE 10

/**
 * @brief Motion state enumeration
 */
typedef enum {
    MOTION_STATIONARY = 0,
    MOTION_SLOW,
    MOTION_FAST,
    MOTION_UNKNOWN
} MotionState_t;

/**
 * @brief Motion detector configuration
 */
typedef struct {
    float stationary_speed_ms;   // GPS speed threshold for stationary
    float slow_speed_ms;         // GPS speed threshold for slow motion
    float accel_variance_thresh; // Accelerometer variance threshold
    float gyro_activity_thresh;  // Gyroscope activity threshold
    float gravity_tolerance;     // Gravity magnitude tolerance
} MotionConfig_t;

/**
 * @brief Motion detector internal state
 */
typedef struct {
    /* Moving average filters */
    float accel_filter[MOTION_FILTER_SIZE];
    float gyro_filter[MOTION_FILTER_SIZE];
    int filter_index;
    bool filter_full;

    /* GPS velocity error detection */
    float gps_speed_history[GPS_SPEED_HISTORY_SIZE];
    uint32_t gps_speed_index;
    bool gps_velocity_suspect;

    /* Current motion indicators */
    float current_accel_variance;
    float current_gyro_activity;
    float current_gps_speed_ms;

    /* Detection results */
    bool gps_indicates_stationary;
    bool imu_indicates_stationary;
    MotionState_t current_state;
    MotionState_t previous_state;

    /* Statistics */
    uint32_t stationary_count;
    uint32_t slow_motion_count;
    uint32_t fast_motion_count;
    uint32_t state_change_count;

    /* Configuration */
    MotionConfig_t config;

} MotionDetector_t;

/**
 * @brief Motion sensor readings input
 */
typedef struct {
    float accel_magnitude;       // Total acceleration magnitude (m/s²)
    float accel_variance;        // Deviation from gravity magnitude
    float gyro_activity;         // Total gyroscope activity (rad/s)
    bool sensors_valid;          // Sensor data validity
} MotionSensorReading_t;

/**
 * @brief GPS motion data input
 */
typedef struct {
    float speed_ms;              // Ground speed (m/s)
    float vel_north;             // North velocity (m/s)
    float vel_east;              // East velocity (m/s)
    float speed_accuracy;        // Speed accuracy estimate (m/s)
    float course_accuracy;       // Course accuracy (degrees)
    float hdop;                  // Horizontal dilution of precision
    int satellites;              // Number of satellites
    uint8_t power_state;         // GPS power/tracking state
    bool data_valid;             // GPS data validity
} GPSMotionData_t;

/* Default configuration */
extern const MotionConfig_t MOTION_CONFIG_DEFAULT;

/**
 * @brief Initialize motion detector
 * @param detector Pointer to motion detector instance
 * @param config Pointer to configuration (NULL for default)
 * @return HAL_OK if successful
 */
HAL_StatusTypeDef MotionDetector_Init(MotionDetector_t* detector, const MotionConfig_t* config);

/**
 * @brief Update motion detection with new sensor data
 * @param detector Pointer to motion detector instance
 * @param sensor_reading Current sensor readings
 * @param gps_data Current GPS motion data (can be NULL)
 * @return Detected motion state
 */
MotionState_t MotionDetector_Update(MotionDetector_t* detector,
                                   const MotionSensorReading_t* sensor_reading,
                                   const GPSMotionData_t* gps_data);

/**
 * @brief Get current motion detection results
 * @param detector Pointer to motion detector instance
 * @param gps_stationary Pointer to store GPS stationary indication
 * @param imu_stationary Pointer to store IMU stationary indication
 * @param velocity_suspect Pointer to store GPS velocity suspect flag
 */
void MotionDetector_GetResults(const MotionDetector_t* detector,
                              bool* gps_stationary,
                              bool* imu_stationary,
                              bool* velocity_suspect);

/**
 * @brief Get filtered motion values
 * @param detector Pointer to motion detector instance
 * @param avg_accel_var Pointer to store filtered acceleration variance
 * @param avg_gyro_activity Pointer to store filtered gyro activity
 */
void MotionDetector_GetFiltered(const MotionDetector_t* detector,
                               float* avg_accel_var,
                               float* avg_gyro_activity);

/**
 * @brief Get motion detection statistics
 * @param detector Pointer to motion detector instance
 * @param stationary_count Pointer to store stationary detection count
 * @param state_changes Pointer to store number of state changes
 */
void MotionDetector_GetStats(const MotionDetector_t* detector,
                            uint32_t* stationary_count,
                            uint32_t* state_changes);

/**
 * @brief Reset motion detector to initial state
 * @param detector Pointer to motion detector instance
 */
void MotionDetector_Reset(MotionDetector_t* detector);

/**
 * @brief Update motion detector configuration
 * @param detector Pointer to motion detector instance
 * @param config New configuration
 * @return HAL_OK if successful
 */
HAL_StatusTypeDef MotionDetector_SetConfig(MotionDetector_t* detector, const MotionConfig_t* config);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_DETECTOR_H */
