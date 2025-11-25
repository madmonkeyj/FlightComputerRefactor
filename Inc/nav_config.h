// nav_config.h
#ifndef NAV_CONFIG_H
#define NAV_CONFIG_H

#include <stdint.h>

typedef struct {
    // Update rates
    struct {
        float ahrs_hz;
        float ekf_hz;
        uint32_t ekf_decimation;  // AHRS updates per EKF update
    } rates;

    // Process noise parameters
    struct {
        float position;    // m²/s³
        float velocity;    // m²/s³
    } process_noise;

    // Measurement noise (adaptive)
    struct {
        float gps_pos_base;           // m²
        float gps_vel_stationary;     // m²/s²
        float gps_vel_slow;           // m²/s²
        float gps_vel_fast;           // m²/s²
        float baro_alt;               // m²
        float zupt;                   // m²/s² (zero velocity update)
    } meas_noise;

    // Motion detection thresholds
    struct {
        float stationary_speed_ms;   // GPS speed threshold
        float slow_speed_ms;         // Slow motion threshold
        float accel_variance;         // m/s²
        float gyro_activity;          // rad/s
        float gravity_tolerance;      // m/s²
    } motion_thresh;

    // Calibration parameters
    struct {
        uint32_t samples;             // Number of samples to average
        uint32_t min_stationary_ms;   // Minimum stationary time
        uint32_t interval_ms;         // Time between calibrations
    } calibration;

} NavConfig_t;

// Default configuration
extern const NavConfig_t NAV_CONFIG_DEFAULT;

// Get default configuration
void NavConfig_GetDefault(NavConfig_t* config);

#endif
