// navigation_ekf.h - Updated to 6-state EKF
/**
  ******************************************************************************
  * @file    navigation_ekf.h
  * @brief   Enhanced Navigation EKF Header - 6-State Implementation
  ******************************************************************************
  */

#ifndef NAVIGATION_EKF_H
#define NAVIGATION_EKF_H

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#include "mahony_filter.h"
#include "motion_detector.h"
#include "coord_transform.h"


#ifdef __cplusplus
extern "C" {
#endif

/* Navigation EKF state size - reduced from 9 to 6 */
#define NAV_STATE_SIZE 6  /* [Pn, Pe, Pd, Vn, Ve, Vd] */

/* Math constants */
#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

/**
 * @brief Motion detection structure
 */
typedef struct {
    /* IMU-based detection */
    float accel_variance;
    float gyro_activity;
    float accel_magnitude;

    /* GPS-based detection */
    float gps_speed_ms;

    /* Hybrid detection results */
    bool gps_indicates_stationary;
    bool imu_indicates_stationary;
    MotionState_t current_state;
    MotionState_t previous_state;

    /* Detection statistics */
    uint32_t stationary_count;
    uint32_t slow_motion_count;
    uint32_t fast_motion_count;
    uint32_t state_change_count;

} MotionDetection_t;

/**
 * @brief EKF debugging data structure
 */
typedef struct {
    /* Innovation values (measurement - prediction) */
    float innovation_pos[3];     // Position innovation [N, E, D] in meters
    float innovation_vel[3];     // Velocity innovation [N, E, D] in m/s
    bool innovation_pos_valid;   // Position innovation data valid
    bool innovation_vel_valid;   // Velocity innovation data valid

    /* Kalman gains applied */
    float kalman_gain_pos[3];    // Position Kalman gains
    float kalman_gain_vel[3];    // Velocity Kalman gains
    bool kalman_gain_pos_valid;  // Position Kalman gain data valid
    bool kalman_gain_vel_valid;  // Velocity Kalman gain data valid

    /* Coordinate transformation results */
    float accel_ned[3];          // Last accelerometer reading transformed to NED
    bool accel_ned_valid;        // Accelerometer NED data valid

    /* Measurement rejection counters */
    uint8_t gps_pos_rejected;    // GPS position rejected this cycle (0/1)
    uint8_t gps_vel_rejected;    // GPS velocity rejected this cycle (0/1)
    uint8_t zupt_applied;        // ZUPT applied this cycle (0/1)

    /* Update timestamp */
    uint32_t last_debug_update;  // HAL_GetTick() of last debug update

} EKFDebugData_t;

/**
 * @brief Navigation EKF structure - Enhanced with debugging
 */
typedef struct {
    /* State vector [Pn, Pe, Pd, Vn, Ve, Vd] */
    float state[NAV_STATE_SIZE];
    float P[NAV_STATE_SIZE][NAV_STATE_SIZE];

    /* Filter parameters */
    float dt;
    bool initialized;

    /* GPS reference frame */
    double gps_ref_lat, gps_ref_lon;
    float gps_ref_alt;
    bool gps_reference_set;

    /* REFACTORED: Motion detection now encapsulated */
    MotionDetector_t motion_detector;
    MotionState_t current_motion_state;  // For backward compatibility

    // Keep these for transition period - they mirror motion detector state
    struct {
        bool gps_indicates_stationary;
        bool imu_indicates_stationary;
        float accel_variance;
        float gyro_activity;
        float gps_speed_ms;
        uint32_t stationary_count;
        uint32_t slow_motion_count;
        uint32_t fast_motion_count;
        uint32_t state_change_count;
        MotionState_t current_state;
        MotionState_t previous_state;
    } motion;  // Compatibility layer

    /* Keep existing fields */
    AccelCalibration_t accel_cal;
    float current_gps_vel_noise;
    float current_gps_pos_noise;
    uint32_t update_count;
    uint32_t last_update_time;
    uint32_t zupt_count;
    EKFDebugData_t debug;

} NavigationEKF_t;

/**
 * @brief Navigation solution output structure
 */
typedef struct {
    /* Position in NED frame (meters) */
    float position_ned[3];

    /* Velocity in NED frame (m/s) */
    float velocity_ned[3];

    /* Position uncertainty (1-sigma, meters) */
    float position_uncertainty[3];

    /* Velocity uncertainty (1-sigma, m/s) */
    float velocity_uncertainty[3];

    /* GPS coordinates */
    double gps_lat;
    double gps_lon;
    float gps_alt;

    /* Derived navigation parameters */
    float ground_speed;          /* 2D ground speed (m/s) */
    float heading;               /* Heading angle (radians, North=0) */
    float climb_rate;            /* Vertical speed (m/s, positive=up) */

    /* Enhanced adaptive information */
    MotionState_t motion_state;
    float adaptive_gps_noise;
    bool zupt_active;

    /* Solution status */
    bool valid;
    uint32_t update_count;

} NavSolution_t;

/* Core Functions - Same public interface maintained */

HAL_StatusTypeDef NavEKF_Init(NavigationEKF_t *nav_ekf, float sample_freq);

HAL_StatusTypeDef NavEKF_SetGPSReference(NavigationEKF_t *nav_ekf,
                                        double ref_lat, double ref_lon, float ref_alt);

/* Backward compatible prediction */
HAL_StatusTypeDef NavEKF_Predict(NavigationEKF_t *nav_ekf,
                                const Quaternion_t *attitude,
                                float accel_body_x, float accel_body_y, float accel_body_z);

/* Enhanced prediction with motion detection */
HAL_StatusTypeDef NavEKF_PredictWithMotionDetection(NavigationEKF_t *nav_ekf,
                                                   const Quaternion_t *attitude,
                                                   float accel_body_x, float accel_body_y, float accel_body_z,
                                                   float gyro_x, float gyro_y, float gyro_z);

/* Measurement updates */
HAL_StatusTypeDef NavEKF_UpdateGPS(NavigationEKF_t *nav_ekf,
                                  double gps_lat, double gps_lon, float gps_alt);

HAL_StatusTypeDef NavEKF_UpdateBarometer(NavigationEKF_t *nav_ekf, float baro_alt);

/* Backward compatible GPS velocity update */
HAL_StatusTypeDef NavEKF_UpdateGPSVelocity(NavigationEKF_t *nav_ekf,
                                          float vel_north, float vel_east, float vel_down);

/* Adaptive GPS velocity update */
HAL_StatusTypeDef NavEKF_UpdateGPSVelocityAdaptive(NavigationEKF_t *nav_ekf,
                                                  float vel_north, float vel_east, float vel_down,
                                                  float gps_speed_ms);

/* Zero-velocity update */
HAL_StatusTypeDef NavEKF_UpdateZeroVelocity(NavigationEKF_t *nav_ekf);

/* Motion detection */
MotionState_t NavEKF_DetectMotionState(NavigationEKF_t *nav_ekf,
                                      float accel_body_x, float accel_body_y, float accel_body_z,
                                      float gyro_x, float gyro_y, float gyro_z,
                                      float gps_speed_ms);

/* Calibration */
HAL_StatusTypeDef NavEKF_CalibrateAccelerometer(NavigationEKF_t *nav_ekf,
                                               float accel_body_x, float accel_body_y, float accel_body_z);

/* Get solution and statistics */
HAL_StatusTypeDef NavEKF_GetSolution(NavigationEKF_t *nav_ekf, NavSolution_t *solution);

HAL_StatusTypeDef NavEKF_Reset(NavigationEKF_t *nav_ekf);

HAL_StatusTypeDef NavEKF_GetStats(NavigationEKF_t *nav_ekf,
                                 float *position_error_rms,
                                 float *velocity_error_rms,
                                 float *update_rate,
                                 MotionState_t *motion_state,
                                 float *zupt_percentage);

#ifdef __cplusplus
}
#endif

#endif /* NAVIGATION_EKF_H */
