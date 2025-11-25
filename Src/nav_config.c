// nav_config.c
#include "nav_config.h"
#include <stddef.h>

const NavConfig_t NAV_CONFIG_DEFAULT = {
    .rates = {
        .ahrs_hz = 500.0f,
        .ekf_hz = 50.0f,
        .ekf_decimation = 10
    },

    .process_noise = {
        .position = 10.0f,
        .velocity = 50.0f
    },

    .meas_noise = {
        .gps_pos_base = 4.0f,
        .gps_vel_stationary = 0.5f,
        .gps_vel_slow = 0.5f,
        .gps_vel_fast = 0.25f,
        .baro_alt = 2.0f,
        .zupt = 0.0001f
    },

    .motion_thresh = {
        .stationary_speed_ms = 0.83f,
        .slow_speed_ms = 8.33f,
        .accel_variance = 0.5f,
        .gyro_activity = 0.02f,
        .gravity_tolerance = 2.0f
    },

    .calibration = {
        .samples = 100,
        .min_stationary_ms = 10000,
        .interval_ms = 300000  // 5 minutes
    }
};

void NavConfig_GetDefault(NavConfig_t* config) {
    if (config != NULL) {
        *config = NAV_CONFIG_DEFAULT;
    }
}
