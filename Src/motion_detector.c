/**
 ******************************************************************************
 * @file    motion_detector.c
 * @brief   Motion Detection Module Implementation
 * @note    Extracted and consolidated from Navigation EKF
 ******************************************************************************
 */

#include "motion_detector.h"
#include <math.h>
#include <string.h>

/* Default configuration - matches original nav_config values */
const MotionConfig_t MOTION_CONFIG_DEFAULT = {
    .stationary_speed_ms = 0.83f,    // 3 km/h = 0.83 m/s
    .slow_speed_ms = 8.33f,          // 30 km/h = 8.33 m/s
    .accel_variance_thresh = 0.5f,   // m/s²
    .gyro_activity_thresh = 0.02f,   // rad/s
    .gravity_tolerance = 2.0f        // m/s²
};

/**
 * @brief Initialize motion detector
 */
HAL_StatusTypeDef MotionDetector_Init(MotionDetector_t* detector, const MotionConfig_t* config) {
    if (!detector) {
        return HAL_ERROR;
    }

    /* Clear all state */
    memset(detector, 0, sizeof(MotionDetector_t));

    /* Set configuration */
    if (config) {
        detector->config = *config;
    } else {
        detector->config = MOTION_CONFIG_DEFAULT;
    }

    /* Initialize state */
    detector->current_state = MOTION_UNKNOWN;
    detector->previous_state = MOTION_UNKNOWN;
    detector->gps_velocity_suspect = false;
    detector->filter_index = 0;
    detector->filter_full = false;
    detector->gps_speed_index = 0;

    return HAL_OK;
}

/**
 * @brief Update moving average filters (extracted from original UpdateMotionFilter)
 */
static void UpdateFilters(MotionDetector_t* detector, float accel_var, float gyro_activity) {
    detector->accel_filter[detector->filter_index] = accel_var;
    detector->gyro_filter[detector->filter_index] = gyro_activity;

    detector->filter_index = (detector->filter_index + 1) % MOTION_FILTER_SIZE;
    if (detector->filter_index == 0) {
        detector->filter_full = true;
    }
}

/**
 * @brief Get filtered motion values (extracted from original GetFilteredMotion)
 */
void MotionDetector_GetFiltered(const MotionDetector_t* detector,
                               float* avg_accel_var,
                               float* avg_gyro_activity) {
    if (!detector || !avg_accel_var || !avg_gyro_activity) {
        return;
    }

    int samples = detector->filter_full ? MOTION_FILTER_SIZE : detector->filter_index;

    *avg_accel_var = 0.0f;
    *avg_gyro_activity = 0.0f;

    for (int i = 0; i < samples; i++) {
        *avg_accel_var += detector->accel_filter[i];
        *avg_gyro_activity += detector->gyro_filter[i];
    }

    if (samples > 0) {
        *avg_accel_var /= samples;
        *avg_gyro_activity /= samples;
    }
}

/**
 * @brief Update GPS velocity error detection (extracted from original logic)
 */
static void UpdateGPSVelocityTracking(MotionDetector_t* detector, const GPSMotionData_t* gps_data) {
    if (!gps_data || !gps_data->data_valid) {
        return;
    }

    /* Update GPS speed history */
    detector->gps_speed_history[detector->gps_speed_index] = gps_data->speed_ms;
    detector->gps_speed_index = (detector->gps_speed_index + 1) % GPS_SPEED_HISTORY_SIZE;

    /* Check if GPS speed is suspiciously constant */
    bool gps_speed_constant = true;
    for (int i = 1; i < GPS_SPEED_HISTORY_SIZE; i++) {
        if (fabsf(detector->gps_speed_history[i] - detector->gps_speed_history[0]) > 0.03f) {
            gps_speed_constant = false;
            break;
        }
    }

    /* Enhanced velocity error detection using UBX data */
    bool velocity_suspect_basic = (gps_speed_constant &&
                                  detector->imu_indicates_stationary &&
                                  gps_data->speed_ms > 0.08f &&
                                  gps_data->speed_ms < 0.83f);

    /* Additional UBX-based velocity error detection */
    bool poor_velocity_accuracy = (gps_data->speed_accuracy > 5.0f);
    bool poor_course_accuracy = (gps_data->course_accuracy > 30.0f && gps_data->speed_ms < 1.39f);
    bool suspect_power_state = (gps_data->power_state != 1); // Not tracking

    detector->gps_velocity_suspect = velocity_suspect_basic ||
                                    poor_velocity_accuracy ||
                                    poor_course_accuracy ||
                                    suspect_power_state;

    /* Clear suspect flag for high speeds or non-constant GPS */
    if (gps_data->speed_ms > 1.39f || !gps_speed_constant) {
        detector->gps_velocity_suspect = false;
    }
}

/**
 * @brief Perform GPS-based stationary detection (extracted from original logic)
 */
static bool DetectGPSStationary(MotionDetector_t* detector, const GPSMotionData_t* gps_data) {
    if (!gps_data || !gps_data->data_valid) {
        return false;
    }

    /* Enhanced GPS stationary detection using UBX data */
    float adjusted_threshold = detector->config.stationary_speed_ms;
    if (gps_data->speed_accuracy > 1.0f) {
        adjusted_threshold += gps_data->speed_accuracy; // Increase threshold for poor accuracy
    }

    bool gps_stationary_enhanced = (gps_data->speed_ms < adjusted_threshold);

    /* Additional check: if course accuracy is poor at low speed, assume stationary */
    if (gps_data->speed_ms < 1.39f && gps_data->course_accuracy > 30.0f) {
        gps_stationary_enhanced = true;
    }

    return gps_stationary_enhanced;
}

/**
 * @brief Perform IMU-based stationary detection (extracted from original logic)
 */
static bool DetectIMUStationary(MotionDetector_t* detector, const MotionSensorReading_t* sensor_reading) {
    if (!sensor_reading || !sensor_reading->sensors_valid) {
        return false;
    }

    /* Get filtered values */
    float avg_accel_var, avg_gyro_activity;
    MotionDetector_GetFiltered(detector, &avg_accel_var, &avg_gyro_activity);

    /* IMU-based stationary detection with gyro priority */
    bool imu_stationary_accel = (avg_accel_var < detector->config.accel_variance_thresh) &&
                               (fabsf(sensor_reading->accel_magnitude - 9.80665f) < detector->config.gravity_tolerance);

    bool imu_stationary_gyro = (avg_gyro_activity < detector->config.gyro_activity_thresh);

    /* Gyro has priority for motion detection */
    bool definitely_moving = (avg_gyro_activity > 0.015f);
    bool definitely_stationary = (avg_gyro_activity < 0.008f) && imu_stationary_accel;

    return definitely_stationary || (!definitely_moving && imu_stationary_accel && imu_stationary_gyro);
}

/**
 * @brief Determine final motion state using multi-sensor fusion (extracted from original logic)
 */
static MotionState_t FuseMotionDetection(MotionDetector_t* detector,
                                        const MotionSensorReading_t* sensor_reading,
                                        const GPSMotionData_t* gps_data) {
    MotionState_t new_state;

    /* Get filtered gyro activity for definitive motion detection */
    float avg_accel_var, avg_gyro_activity;
    MotionDetector_GetFiltered(detector, &avg_accel_var, &avg_gyro_activity);

    bool definitely_moving = (avg_gyro_activity > 0.015f);
    bool definitely_stationary = (avg_gyro_activity < 0.008f) &&
                                 (avg_accel_var < detector->config.accel_variance_thresh);

    float gps_speed = gps_data && gps_data->data_valid ? gps_data->speed_ms : 0.0f;

    /* Multi-sensor fusion for motion state */
    /* High confidence stationary: Both sensors agree AND enhanced checks pass */
    if (detector->gps_indicates_stationary && detector->imu_indicates_stationary) {
        new_state = MOTION_STATIONARY;
    }
    /* GPS velocity error case: Trust IMU when GPS is clearly wrong */
    else if (detector->gps_velocity_suspect && definitely_stationary) {
        new_state = MOTION_STATIONARY;
    }
    /* Definite motion: Gyro shows clear activity */
    else if (definitely_moving) {
        new_state = (gps_speed < detector->config.slow_speed_ms) ? MOTION_SLOW : MOTION_FAST;
    }
    /* GPS-based classification for unclear cases */
    else if (gps_speed < detector->config.slow_speed_ms) {
        new_state = MOTION_SLOW;
    } else {
        new_state = MOTION_FAST;
    }

    /* Enhanced: State validation using GPS quality */
    if (gps_data && gps_data->data_valid && new_state == MOTION_STATIONARY) {
        /* Require good GPS quality for stationary detection */
        if (gps_data->hdop > 3.0f || gps_data->satellites < 4) {
            /* Don't trust GPS for stationary detection with poor quality */
            if (!definitely_stationary) {
                new_state = MOTION_SLOW; // Conservative fallback
            }
        }
    }

    return new_state;
}

/**
 * @brief Update motion detection with new sensor data
 */
MotionState_t MotionDetector_Update(MotionDetector_t* detector,
                                   const MotionSensorReading_t* sensor_reading,
                                   const GPSMotionData_t* gps_data) {
    if (!detector || !sensor_reading) {
        return MOTION_UNKNOWN;
    }

    /* Store current measurements */
    detector->current_accel_variance = sensor_reading->accel_variance;
    detector->current_gyro_activity = sensor_reading->gyro_activity;
    detector->current_gps_speed_ms = gps_data && gps_data->data_valid ? gps_data->speed_ms : 0.0f;

    /* Update motion filters */
    UpdateFilters(detector, sensor_reading->accel_variance, sensor_reading->gyro_activity);

    /* GPS velocity error detection */
    if (gps_data) {
        UpdateGPSVelocityTracking(detector, gps_data);
    }

    /* Individual sensor detection */
    detector->gps_indicates_stationary = DetectGPSStationary(detector, gps_data);
    detector->imu_indicates_stationary = DetectIMUStationary(detector, sensor_reading);

    /* Multi-sensor fusion */
    MotionState_t new_state = FuseMotionDetection(detector, sensor_reading, gps_data);

    /* Update statistics */
    if (new_state != detector->current_state) {
        detector->state_change_count++;
        detector->previous_state = detector->current_state;
        detector->current_state = new_state;
    }

    /* Update counters */
    switch (new_state) {
        case MOTION_STATIONARY:
            detector->stationary_count++;
            break;
        case MOTION_SLOW:
            detector->slow_motion_count++;
            break;
        case MOTION_FAST:
            detector->fast_motion_count++;
            break;
        default:
            break;
    }

    return new_state;
}

/**
 * @brief Get current motion detection results
 */
void MotionDetector_GetResults(const MotionDetector_t* detector,
                              bool* gps_stationary,
                              bool* imu_stationary,
                              bool* velocity_suspect) {
    if (!detector) {
        return;
    }

    if (gps_stationary) {
        *gps_stationary = detector->gps_indicates_stationary;
    }

    if (imu_stationary) {
        *imu_stationary = detector->imu_indicates_stationary;
    }

    if (velocity_suspect) {
        *velocity_suspect = detector->gps_velocity_suspect;
    }
}

/**
 * @brief Get motion detection statistics
 */
void MotionDetector_GetStats(const MotionDetector_t* detector,
                            uint32_t* stationary_count,
                            uint32_t* state_changes) {
    if (!detector) {
        return;
    }

    if (stationary_count) {
        *stationary_count = detector->stationary_count;
    }

    if (state_changes) {
        *state_changes = detector->state_change_count;
    }
}

/**
 * @brief Reset motion detector to initial state
 */
void MotionDetector_Reset(MotionDetector_t* detector) {
    if (!detector) {
        return;
    }

    /* Preserve configuration */
    MotionConfig_t saved_config = detector->config;

    /* Clear all state */
    memset(detector, 0, sizeof(MotionDetector_t));

    /* Restore configuration */
    detector->config = saved_config;
    detector->current_state = MOTION_UNKNOWN;
    detector->previous_state = MOTION_UNKNOWN;
}

/**
 * @brief Update motion detector configuration
 */
HAL_StatusTypeDef MotionDetector_SetConfig(MotionDetector_t* detector, const MotionConfig_t* config) {
    if (!detector || !config) {
        return HAL_ERROR;
    }

    detector->config = *config;
    return HAL_OK;
}
