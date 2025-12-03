/**
  ******************************************************************************
  * @file    navigation_ekf.c
  * @brief   Enhanced Navigation EKF - 6-State Implementation
  ******************************************************************************
  */

#include "navigation_ekf.h"
#include "nav_matrix.h"
#include "nav_config.h"
#include "coord_transform.h"
#include <math.h>
#include <string.h>
#include "stdio.h"
#include "gps_module.h"
#include "debug_utils.h"


/* Mathematical constants */
#define GRAVITY_MAGNITUDE 9.80665f
#define EARTH_RADIUS 6378137.0
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Static configuration instance */
static NavConfig_t ekf_config;
static bool config_loaded = false;

/* Moving average filter for motion detection */
#define MOTION_FILTER_SIZE 2

/* Private function prototypes */
static void GPS_ToNED(NavigationEKF_t *nav_ekf, double lat, double lon, float alt,
                      float *north, float *east, float *down);

static HAL_StatusTypeDef EKF_MeasurementUpdate(NavigationEKF_t *nav_ekf,
                                               const float *z, const float *h_x,
                                               const float H[][NAV_STATE_SIZE], const float R[][3],
                                               int meas_dim, const int *state_indices);

// NEW: GPS velocity validation function
static bool ValidateGPSVelocity(const GPS_Data_t *gps_data, float vel_n, float vel_e, float vel_d);

// NEW: Covariance matrix conditioning
static void ConditionCovarianceMatrix(float P[NAV_STATE_SIZE][NAV_STATE_SIZE]);



static void UpdateMotionCompatibilityLayer(NavigationEKF_t *nav_ekf);

/**
 * @brief Update compatibility layer with motion detector state
 */
static void UpdateMotionCompatibilityLayer(NavigationEKF_t *nav_ekf) {
    if (!nav_ekf) return;

    // Update compatibility fields from motion detector
    MotionDetector_GetResults(&nav_ekf->motion_detector,
                             &nav_ekf->motion.gps_indicates_stationary,
                             &nav_ekf->motion.imu_indicates_stationary,
                             NULL);

    MotionDetector_GetFiltered(&nav_ekf->motion_detector,
                              &nav_ekf->motion.accel_variance,
                              &nav_ekf->motion.gyro_activity);

    uint32_t stationary_count, state_changes;
    MotionDetector_GetStats(&nav_ekf->motion_detector, &stationary_count, &state_changes);

    nav_ekf->motion.stationary_count = stationary_count;
    nav_ekf->motion.state_change_count = state_changes;
    nav_ekf->motion.current_state = nav_ekf->current_motion_state;
    nav_ekf->motion.gps_speed_ms = 0.0f; // Will be updated in detection function
}

/**
 * @brief Enhanced GPS velocity validation
 *
 * REPLACE the existing ValidateGPSVelocity function with this version
 * (Keep the same function name and signature)
 */
static bool ValidateGPSVelocity(const GPS_Data_t *gps_data, float vel_n, float vel_e, float vel_d) {
    if (!gps_data || gps_data->fix_status != 'A') {
        return false;
    }

    // Basic sanity checks
    if (!isfinite(vel_n) || !isfinite(vel_e) || !isfinite(vel_d)) {
        return false;
    }

    // RELAXED: Use your GPS's excellent quality data
    // Your GPS provides much better than these thresholds
    if (gps_data->hdop > 6.0f || gps_data->satellites < 4) {  // Was 3.0f
        return false;
    }

    // LEVERAGE COVARIANCE: Use actual GPS accuracy instead of fixed threshold
    if (gps_data->vel_cov_valid) {
        // Use actual velocity standard deviations from GPS
        float vel_std_max = sqrtf(fmaxf(gps_data->vel_cov_nn,
                                  fmaxf(gps_data->vel_cov_ee, gps_data->vel_cov_dd)));
        if (vel_std_max > 15.0f) {  // Reject only if GPS reports very poor accuracy
            return false;
        }
    } else {
        // Fallback for old accuracy field
        if (gps_data->sAcc > 25.0f) {  // Was 10.0f - much more reasonable
            return false;
        }
    }

    // ROCKET-APPROPRIATE: Increased velocity bounds
    float gps_2d_speed = sqrtf(gps_data->velN * gps_data->velN + gps_data->velE * gps_data->velE);
    if (gps_2d_speed > 83.33f) {  // 300 km/h = 83.33 m/s
            return false;
        }

    // Check 3D velocity magnitude
    float gps_3d_speed = sqrtf(gps_2d_speed * gps_2d_speed + gps_data->velD * gps_data->velD);
    if (gps_3d_speed > 111.11f) {  // 400 km/h = 111.11 m/s
        return false;
    }

    // REMOVED: Velocity consistency check (redundant with single GPS source)

    return true;
}


static bool CheckInnovationBounds(const float *innovation, int meas_dim) {
    // First check for NaN/Inf
    for (int i = 0; i < meas_dim; i++) {
        if (!isfinite(innovation[i])) {
            return false;
        }
    }

    float threshold = (meas_dim == 3 && innovation[0] < 50.0f) ? 100.0f : 10.0f;

    for (int i = 0; i < meas_dim; i++) {
        if (fabsf(innovation[i]) > threshold) {
            return false;
        }
    }
    return true;
}

/**
 * @brief NEW: Adaptive covariance matrix conditioning based on GPS quality
 */
static void ConditionCovarianceMatrix(float P[NAV_STATE_SIZE][NAV_STATE_SIZE]) {
    // === ADAPTIVE MINIMUMS BASED ON GPS QUALITY ===
    float min_pos_var = 1.0f;      // Default: 1m std dev
    float min_vel_var = 0.25f;     // Default: 0.5m/s std dev

    // Get current GPS quality
    GPS_Data_t current_gps;
    bool gps_available = GPS_GetCurrentData(&current_gps) &&
                         current_gps.fix_status == 'A' &&
                         (HAL_GetTick() - current_gps.last_update) < 5000;  // Fresh data

    if (gps_available) {
        // === POSITION UNCERTAINTY ADAPTATION ===
        if (current_gps.hAcc > 0.1f && current_gps.hAcc < 50.0f) {
            // Use GPS reported horizontal accuracy (with safety bounds)
            min_pos_var = current_gps.hAcc * current_gps.hAcc;

            // Scale based on HDOP quality
            if (current_gps.hdop > 2.0f) {
                min_pos_var *= (1.0f + (current_gps.hdop - 2.0f) * 0.5f);
            }

            // Scale based on satellite count
            if (current_gps.satellites < 6) {
                min_pos_var *= (2.0f - current_gps.satellites / 6.0f);
            }
        }

        // === VELOCITY UNCERTAINTY ADAPTATION ===
        if (current_gps.sAcc > 0.1f && current_gps.sAcc < 20.0f) {
            // Use GPS reported speed accuracy
            min_vel_var = current_gps.sAcc * current_gps.sAcc;

            // Scale based on vehicle speed (higher speed = potentially better GPS velocity)
            float gps_speed = sqrtf(current_gps.velN * current_gps.velN +
                                   current_gps.velE * current_gps.velE);
            if (gps_speed > 5.0f) {
                min_vel_var *= 0.8f;  // Reduce minimum when moving fast
            } else if (gps_speed < 1.0f) {
                min_vel_var *= 2.0f;  // Increase minimum when nearly stationary
            }
        }

        // === ENHANCED GPS QUALITY CHECKS ===
        // Poor fix quality - increase minimums
        if (current_gps.hdop > 5.0f || current_gps.satellites < 4) {
            min_pos_var *= 4.0f;  // Much less confident
            min_vel_var *= 3.0f;
        }

        // Check for spoofing/jamming
        if (GPS_IsSpoofingDetected()) {
            min_pos_var *= 10.0f;  // Very uncertain if spoofing detected
            min_vel_var *= 10.0f;
        }

    } else {
        // === NO GPS OR STALE GPS - INCREASE UNCERTAINTY ===
        // GPS denied/unavailable - grow uncertainty over time
        static uint32_t last_valid_gps_time = 0;
        static bool first_gps_loss = true;

        if (first_gps_loss) {
            last_valid_gps_time = HAL_GetTick();
            first_gps_loss = false;
        }

        uint32_t time_since_gps = HAL_GetTick() - last_valid_gps_time;
        float gps_denial_seconds = time_since_gps / 1000.0f;

        // Grow uncertainty during GPS denial
        min_pos_var = 1.0f + (gps_denial_seconds * gps_denial_seconds * 0.1f);  // Quadratic growth
        min_vel_var = 0.25f + (gps_denial_seconds * 0.05f);                    // Linear growth

        // Reset flag when GPS returns
        if (gps_available) {
            first_gps_loss = true;
        }
    }

    // === SAFETY BOUNDS ===
    // Don't let adaptive minimums get too small or too large
    if (min_pos_var < 0.25f) min_pos_var = 0.25f;      // Never better than 50cm
    if (min_pos_var > 100.0f) min_pos_var = 100.0f;    // Never worse than 10m

    if (min_vel_var < 0.04f) min_vel_var = 0.04f;      // Never better than 20cm/s
    if (min_vel_var > 25.0f) min_vel_var = 25.0f;      // Never worse than 5m/s

    // === APPLY ADAPTIVE MINIMUMS ===
    for (int i = 0; i < 3; i++) {
        if (P[i][i] < min_pos_var) P[i][i] = min_pos_var;
        if (P[i+3][i+3] < min_vel_var) P[i+3][i+3] = min_vel_var;
    }

    // === EXISTING MAXIMUM BOUNDS (unchanged) ===
    const float max_pos_var = 10000.0f;  // 100m std dev maximum
    const float max_vel_var = 100.0f;    // 10m/s std dev maximum

    for (int i = 0; i < 3; i++) {
        if (P[i][i] > max_pos_var) P[i][i] = max_pos_var;
        if (P[i+3][i+3] > max_vel_var) P[i+3][i+3] = max_vel_var;
    }

    // === EXISTING OFF-DIAGONAL CONDITIONING (unchanged) ===
    for (int i = 0; i < NAV_STATE_SIZE; i++) {
        for (int j = 0; j < NAV_STATE_SIZE; j++) {
            if (i != j && !isfinite(P[i][j])) {
                P[i][j] = 0.0f;
            }
        }
    }
}


static bool IsGPSMeasurementValid(const GPS_Data_t *gps_data) {
    if (!gps_data || gps_data->fix_status != 'A') return false;

    // Check for spoofing/jamming
    if (GPS_IsSpoofingDetected()) return false;

    // Power state check
    if (gps_data->power_state == 3) return false; // Inactive

    // Basic quality checks
    if (gps_data->hdop > 4.0f || gps_data->satellites < 4) return false;

    return true;
}

/**
 * @brief Convert GPS lat/lon to NED coordinates
 */
static void GPS_ToNED(NavigationEKF_t *nav_ekf, double lat, double lon, float alt,
                      float *north, float *east, float *down) {
    if (!nav_ekf->gps_reference_set) {
        *north = 0.0f;
        *east = 0.0f;
        *down = 0.0f;
        return;
    }

    /* Simple flat earth approximation */
    double dlat = lat - nav_ekf->gps_ref_lat;
    double dlon = lon - nav_ekf->gps_ref_lon;

    *north = (float)(dlat * DEG_TO_RAD * EARTH_RADIUS);
    *east = (float)(dlon * DEG_TO_RAD * EARTH_RADIUS * cos(nav_ekf->gps_ref_lat * DEG_TO_RAD));
    *down = nav_ekf->gps_ref_alt - alt;
}

/**
 * @brief Enhanced EKF measurement update with debugging data capture
 */
static HAL_StatusTypeDef EKF_MeasurementUpdate(NavigationEKF_t *nav_ekf,
                                               const float *z, const float *h_x,
                                               const float H[][NAV_STATE_SIZE], const float R[][3],
                                               int meas_dim, const int *state_indices) {
    // Calculate innovation y = z - h(x)
    float y[3];
    for (int i = 0; i < meas_dim; i++) {
        y[i] = z[i] - h_x[i];
    }

    // **CAPTURE INNOVATION VALUES FOR DEBUGGING - ALWAYS (even if rejected)**
    if (state_indices && meas_dim <= 3) {
        bool is_position = (state_indices[0] < 3);
        if (is_position && meas_dim == 3) {
            // Position measurement
            for (int i = 0; i < 3; i++) {
                nav_ekf->debug.innovation_pos[i] = y[i];
            }
            nav_ekf->debug.innovation_pos_valid = true;
            nav_ekf->debug.last_debug_update = HAL_GetTick();  // ADD THIS
        } else if (!is_position && meas_dim == 3) {
            // Velocity measurement
            for (int i = 0; i < 3; i++) {
                nav_ekf->debug.innovation_vel[i] = y[i];
            }
            nav_ekf->debug.innovation_vel_valid = true;
            nav_ekf->debug.last_debug_update = HAL_GetTick();  // ADD THIS
        }
    }

    if (!CheckInnovationBounds(y, meas_dim)) {
        // **CAPTURE REJECTION FOR DEBUGGING**
        if (state_indices && meas_dim == 3) {
            bool is_position = (state_indices[0] < 3);
            if (is_position) {
                nav_ekf->debug.gps_pos_rejected = 1;
            } else {
                nav_ekf->debug.gps_vel_rejected = 1;
            }
        }
        return HAL_ERROR;
    }

    // Clear rejection flags if we get here
    /*if (state_indices && meas_dim == 3) {
        bool is_position = (state_indices[0] < 3);
        if (is_position) {
            nav_ekf->debug.gps_pos_rejected = 0;
        } else {
            nav_ekf->debug.gps_vel_rejected = 0;
        }
    }*/
    // Acceptance is implicit - rejection flag stays 0 if not set
    // This allows us to see the LAST rejection, not just current cycle
    // Flags are cleared at start of prediction step (see Fix 3)

    // Innovation covariance S = H*P*H' + R
    float HP[3][NAV_STATE_SIZE];
    float S[3][3];

    // HP = H * P
    for (int i = 0; i < meas_dim; i++) {
        for (int j = 0; j < NAV_STATE_SIZE; j++) {
            HP[i][j] = 0.0f;
            for (int k = 0; k < NAV_STATE_SIZE; k++) {
                HP[i][j] += H[i][k] * nav_ekf->P[k][j];
            }
        }
    }

    // S = HP * H' + R
    for (int i = 0; i < meas_dim; i++) {
        for (int j = 0; j < meas_dim; j++) {
            S[i][j] = R[i][j];
            for (int k = 0; k < NAV_STATE_SIZE; k++) {
                S[i][j] += HP[i][k] * H[j][k];
            }
        }
    }

    // Enhanced matrix conditioning
    for (int i = 0; i < meas_dim; i++) {
        if (S[i][i] < 1e-6f) {
            return HAL_ERROR;
        }
    }

    // Invert S
    float S_inv[3][3];
    if (meas_dim == 1) {
        if (fabsf(S[0][0]) < 1e-6f) return HAL_ERROR;
        S_inv[0][0] = 1.0f / S[0][0];
    } else if (meas_dim == 3) {
        if (NavMatrix_Invert3x3(S, S_inv) != HAL_OK) {
            return HAL_ERROR;
        }
    } else {
        return HAL_ERROR;
    }

    // Calculate Kalman gain K = P * H' * S_inv
    float K[NAV_STATE_SIZE][3];
    for (int i = 0; i < NAV_STATE_SIZE; i++) {
        for (int j = 0; j < meas_dim; j++) {
            K[i][j] = 0.0f;
            for (int k = 0; k < meas_dim; k++) {
                float PH_ik = 0.0f;
                for (int l = 0; l < NAV_STATE_SIZE; l++) {
                    PH_ik += nav_ekf->P[i][l] * H[k][l];
                }
                K[i][j] += PH_ik * S_inv[k][j];
            }
        }
    }

    // **CAPTURE KALMAN GAINS FOR DEBUGGING**
    if (state_indices && meas_dim == 3) {
        bool is_position = (state_indices[0] < 3);
        if (is_position) {
            // Position measurement - capture gains for position states
            for (int i = 0; i < 3; i++) {
                nav_ekf->debug.kalman_gain_pos[i] = K[i][i]; // Gain for state i from measurement 0
            }
            nav_ekf->debug.kalman_gain_pos_valid = true;
        } else {
            // Velocity measurement - capture gains for velocity states
            for (int i = 0; i < 3; i++) {
                nav_ekf->debug.kalman_gain_vel[i] = K[i+3][i]; // Gain for velocity state i from measurement 0
            }
            nav_ekf->debug.kalman_gain_vel_valid = true;
        }
    }

    // Limit Kalman gain to prevent overcorrection
    for (int i = 0; i < NAV_STATE_SIZE; i++) {
        for (int j = 0; j < meas_dim; j++) {
            if (fabsf(K[i][j]) > 5.0f) {
                K[i][j] = (K[i][j] > 0) ? 5.0f : -5.0f;
            }
        }
    }

    // Store original state for validation
    float original_state[NAV_STATE_SIZE];
    memcpy(original_state, nav_ekf->state, sizeof(original_state));

    // Apply state update: x = x + K*y
    for (int i = 0; i < NAV_STATE_SIZE; i++) {
        for (int j = 0; j < meas_dim; j++) {
            nav_ekf->state[i] += K[i][j] * y[j];
        }
    }

    // Validate updated state
    for (int i = 0; i < NAV_STATE_SIZE; i++) {
        if (!isfinite(nav_ekf->state[i])) {
            memcpy(nav_ekf->state, original_state, sizeof(original_state));
            return HAL_ERROR;
        }
    }

    // Check for unreasonable velocity changes
    if (state_indices && state_indices[0] >= 3) {
        float vel_change_sq = 0.0f;
        for (int i = 0; i < meas_dim && i < 3; i++) {
            if (state_indices[i] >= 3 && state_indices[i] < NAV_STATE_SIZE) {
                float change = nav_ekf->state[state_indices[i]] - original_state[state_indices[i]];
                vel_change_sq += change * change;
            }
        }

        if (vel_change_sq > 400.0f) {  // > 20 m/s change
            memcpy(nav_ekf->state, original_state, sizeof(original_state));
            return HAL_ERROR;
        }
    }

    // Update covariance: P = (I - K*H)*P
    float I_KH[NAV_STATE_SIZE][NAV_STATE_SIZE];
    NavMatrix_Eye6x6(I_KH);

    for (int i = 0; i < NAV_STATE_SIZE; i++) {
        for (int j = 0; j < NAV_STATE_SIZE; j++) {
            for (int k = 0; k < meas_dim; k++) {
                I_KH[i][j] -= K[i][k] * H[k][j];
            }
        }
    }

    float new_P[NAV_STATE_SIZE][NAV_STATE_SIZE];
    NavMatrix_Multiply6x6(I_KH, nav_ekf->P, new_P);
    NavMatrix_Copy6x6(new_P, nav_ekf->P);

    ConditionCovarianceMatrix(nav_ekf->P);

    // **UPDATE DEBUG TIMESTAMP**
    /*nav_ekf->debug.last_debug_update = HAL_GetTick();*/ // ← DELETE THIS (already set above)

    return HAL_OK;
}

/**
 * @brief Initialize Navigation EKF - Updated to include motion detector
 */
HAL_StatusTypeDef NavEKF_Init(NavigationEKF_t *nav_ekf, float sample_freq) {
    if (nav_ekf == NULL) {
        return HAL_ERROR;
    }

    /* Load configuration if not already loaded */
    if (!config_loaded) {
        NavConfig_GetDefault(&ekf_config);
        config_loaded = true;
    }

    /* Initialize state vector and covariance (unchanged) */
    memset(nav_ekf->state, 0, sizeof(nav_ekf->state));
    memset(nav_ekf->P, 0, sizeof(nav_ekf->P));
    nav_ekf->P[0][0] = 100.0f;
    nav_ekf->P[1][1] = 100.0f;
    nav_ekf->P[2][2] = 100.0f;
    nav_ekf->P[3][3] = 25.0f;
    nav_ekf->P[4][4] = 25.0f;
    nav_ekf->P[5][5] = 25.0f;

    /* Set parameters (unchanged) */
    nav_ekf->dt = (sample_freq > 0.0f) ? (1.0f / sample_freq) : (1.0f / ekf_config.rates.ekf_hz);
    nav_ekf->initialized = false;
    nav_ekf->gps_reference_set = false;
    nav_ekf->update_count = 0;
    nav_ekf->zupt_count = 0;
    nav_ekf->last_update_time = HAL_GetTick();

    /* REFACTORED: Initialize motion detector */
    MotionConfig_t motion_config = {
        .stationary_speed_ms = ekf_config.motion_thresh.stationary_speed_ms,
        .slow_speed_ms = ekf_config.motion_thresh.slow_speed_ms,
        .accel_variance_thresh = ekf_config.motion_thresh.accel_variance,
        .gyro_activity_thresh = ekf_config.motion_thresh.gyro_activity,
        .gravity_tolerance = ekf_config.motion_thresh.gravity_tolerance
    };

    if (MotionDetector_Init(&nav_ekf->motion_detector, &motion_config) != HAL_OK) {
        return HAL_ERROR;
    }

    /* ADD THESE LINES: Initialize coordinate transformation module */
    if (CoordTransform_Init() != HAL_OK) {
        return HAL_ERROR;
    }

    nav_ekf->current_motion_state = MOTION_UNKNOWN;

    /* Initialize accelerometer calibration (unchanged) */
    memset(&nav_ekf->accel_cal, 0, sizeof(nav_ekf->accel_cal));
    nav_ekf->accel_cal.scale[0] = 1.0f;
    nav_ekf->accel_cal.scale[1] = 1.0f;
    nav_ekf->accel_cal.scale[2] = 1.0f;

    /* Initialize debug data structure (unchanged) */
    memset(&nav_ekf->debug, 0, sizeof(nav_ekf->debug));
    for (int i = 0; i < 3; i++) {
        nav_ekf->debug.innovation_pos[i] = NAN;
        nav_ekf->debug.innovation_vel[i] = NAN;
        nav_ekf->debug.kalman_gain_pos[i] = NAN;
        nav_ekf->debug.kalman_gain_vel[i] = NAN;
        nav_ekf->debug.accel_ned[i] = NAN;
    }

    return HAL_OK;
}


/**
 * @brief Enhanced GPS reference setting with velocity initialization
 *
 * REPLACE the existing NavEKF_SetGPSReference function with this version
 * (Keep the same function name and signature)
 */
HAL_StatusTypeDef NavEKF_SetGPSReference(NavigationEKF_t *nav_ekf,
                                        double ref_lat, double ref_lon, float ref_alt) {
    if (nav_ekf == NULL) {
        return HAL_ERROR;
    }

    /* Get current GPS data for quality assessment */
    GPS_Data_t current_gps;
    if (GPS_GetCurrentData(&current_gps)) {
        /* Basic quality checks for reference setting */
        if (current_gps.hdop > 5.0f ||
            current_gps.satellites < 4 ||
            current_gps.hAcc > 10.0f ||
            GPS_IsSpoofingDetected()) {
            return HAL_ERROR;
        }
    }

    /* Set GPS reference frame */
    nav_ekf->gps_ref_lat = ref_lat;
    nav_ekf->gps_ref_lon = ref_lon;
    nav_ekf->gps_ref_alt = ref_alt;
    nav_ekf->gps_reference_set = true;

    /* Initialize position state to origin */
    nav_ekf->state[0] = 0.0f;  /* North */
    nav_ekf->state[1] = 0.0f;  /* East */
    nav_ekf->state[2] = 0.0f;  /* Down */

    /* 🔧 KEEP GPS velocity initialization - this is correct */
    if (GPS_GetCurrentData(&current_gps) && current_gps.fix_status == 'A') {
        nav_ekf->state[3] = current_gps.velN;  // North velocity
        nav_ekf->state[4] = current_gps.velE;  // East velocity
        nav_ekf->state[5] = current_gps.velD;  // Down velocity

        // Reduce velocity uncertainty since we have GPS velocity
        nav_ekf->P[3][3] = 4.0f;  // Reduced from 25.0f
        nav_ekf->P[4][4] = 4.0f;
        nav_ekf->P[5][5] = 4.0f;

        // Reduce position uncertainty if GPS accuracy is good
        if (current_gps.hAcc < 5.0f) {
            nav_ekf->P[0][0] = current_gps.hAcc * current_gps.hAcc;
            nav_ekf->P[1][1] = current_gps.hAcc * current_gps.hAcc;
        }
        if (current_gps.vAcc < 10.0f) {
            nav_ekf->P[2][2] = current_gps.vAcc * current_gps.vAcc;
        }
    }

    nav_ekf->initialized = true;
    return HAL_OK;
}

/**
 * @brief REFACTORED: Motion detection now uses dedicated module
 */
MotionState_t NavEKF_DetectMotionState(NavigationEKF_t *nav_ekf,
                                      float accel_body_x, float accel_body_y, float accel_body_z,
                                      float gyro_x, float gyro_y, float gyro_z,
                                      float gps_speed_ms) {
    if (nav_ekf == NULL) {
        return MOTION_UNKNOWN;
    }

    /* Calculate values FIRST, before struct initialization */
    float accel_mag = sqrtf(accel_body_x * accel_body_x +
                           accel_body_y * accel_body_y +
                           accel_body_z * accel_body_z);
    float accel_var = fabsf(accel_mag - 9.80665f);
    float gyro_act = sqrtf(gyro_x * gyro_x + gyro_y * gyro_y + gyro_z * gyro_z);

    MotionSensorReading_t sensor_reading = {
        .accel_magnitude = accel_mag,
        .accel_variance = accel_var,
        .gyro_activity = gyro_act,
        .sensors_valid = true
    };

    /* Prepare GPS data if available */
    GPSMotionData_t gps_motion_data = {0};
    GPS_Data_t current_gps;
    bool has_gps = GPS_GetCurrentData(&current_gps);

    if (has_gps && current_gps.fix_status == 'A') {
        gps_motion_data.speed_ms = current_gps.speed;
        gps_motion_data.vel_north = current_gps.velN;
        gps_motion_data.vel_east = current_gps.velE;
        gps_motion_data.speed_accuracy = current_gps.sAcc;
        gps_motion_data.course_accuracy = current_gps.course_acc;
        gps_motion_data.hdop = current_gps.hdop;
        gps_motion_data.satellites = current_gps.satellites;
        gps_motion_data.power_state = current_gps.power_state;
        gps_motion_data.data_valid = true;
    } else {
        gps_motion_data.speed_ms = gps_speed_ms; // Fallback to provided speed
        gps_motion_data.data_valid = false;
    }

    /* Update motion detector */
    MotionState_t detected_state = MotionDetector_Update(&nav_ekf->motion_detector,
                                                        &sensor_reading,
                                                        has_gps ? &gps_motion_data : NULL);

    /* Update backward compatibility fields */
    nav_ekf->current_motion_state = detected_state;

    // At the end, update both the new field and compatibility layer
    nav_ekf->current_motion_state = detected_state;  // New field
    nav_ekf->motion.current_state = detected_state;  // Compatibility
    nav_ekf->motion.gps_speed_ms = gps_speed_ms;     // Compatibility

    // Update other compatibility fields
    UpdateMotionCompatibilityLayer(nav_ekf);

    return detected_state;
}

/**
 * @brief EKF Prediction Step - Backward compatible
 */
HAL_StatusTypeDef NavEKF_Predict(NavigationEKF_t *nav_ekf,
                                const Quaternion_t *attitude,
                                float accel_body_x, float accel_body_y, float accel_body_z) {
    /* Call enhanced version with zero gyro values */
    return NavEKF_PredictWithMotionDetection(nav_ekf, attitude,
                                           accel_body_x, accel_body_y, accel_body_z,
                                           0.0f, 0.0f, 0.0f);
}

/**
 * @brief Revert coordinate frame changes back to original
 *
 * REPLACE the coordinate transformation section in NavEKF_PredictWithMotionDetection()
 * in navigation_ekf.c with this ORIGINAL version.
 *
 * This restores the coordinate transformation that was working before.
 */
HAL_StatusTypeDef NavEKF_PredictWithMotionDetection(NavigationEKF_t *nav_ekf,
                                                   const Quaternion_t *attitude,
                                                   float accel_body_x, float accel_body_y, float accel_body_z,
                                                   float gyro_x, float gyro_y, float gyro_z) {
    if (nav_ekf == NULL || !nav_ekf->initialized || attitude == NULL) {
        return HAL_ERROR;
    }

    /* Force normalize quaternion to prevent drift issues */
    Quaternion_t safe_attitude;
    float q_norm = sqrtf(attitude->q0*attitude->q0 + attitude->q1*attitude->q1 +
                        attitude->q2*attitude->q2 + attitude->q3*attitude->q3);

    if (q_norm < 0.001f || !isfinite(q_norm)) {
        safe_attitude.q0 = 1.0f;
        safe_attitude.q1 = 0.0f;
        safe_attitude.q2 = 0.0f;
        safe_attitude.q3 = 0.0f;
    } else {
        float inv_norm = 1.0f / q_norm;
        safe_attitude.q0 = attitude->q0 * inv_norm;
        safe_attitude.q1 = attitude->q1 * inv_norm;
        safe_attitude.q2 = attitude->q2 * inv_norm;
        safe_attitude.q3 = attitude->q3 * inv_norm;
    }

    float dt = nav_ekf->dt;

    float accel_body[3] = {accel_body_x, accel_body_y, accel_body_z};
    CoordTransformResult_t coord_result;

    HAL_StatusTypeDef coord_status = CoordTransform_BodyToNED(&safe_attitude,  // Use safe_attitude
                                                             accel_body,
                                                             &nav_ekf->accel_cal,
                                                             &coord_result);

    if (coord_status != HAL_OK || !coord_result.valid) {
        return HAL_ERROR;
    }

    memcpy(nav_ekf->debug.accel_ned, coord_result.accel_ned, sizeof(nav_ekf->debug.accel_ned));
    nav_ekf->debug.accel_ned_valid = true;

    float accel_ned_n = coord_result.accel_ned_with_gravity[0];
    float accel_ned_e = coord_result.accel_ned_with_gravity[1];
    float accel_ned_d = coord_result.accel_ned_with_gravity[2];

    /* State prediction */
    float new_state[NAV_STATE_SIZE];
    float dt2 = dt * dt;

    new_state[0] = nav_ekf->state[0] + nav_ekf->state[3] * dt + 0.5f * accel_ned_n * dt2;
    new_state[1] = nav_ekf->state[1] + nav_ekf->state[4] * dt + 0.5f * accel_ned_e * dt2;
    new_state[2] = nav_ekf->state[2] + nav_ekf->state[5] * dt + 0.5f * accel_ned_d * dt2;
    new_state[3] = nav_ekf->state[3] + accel_ned_n * dt;
    new_state[4] = nav_ekf->state[4] + accel_ned_e * dt;
    new_state[5] = nav_ekf->state[5] + accel_ned_d * dt;

    memcpy(nav_ekf->state, new_state, sizeof(new_state));

    nav_ekf->update_count++;
    nav_ekf->last_update_time = HAL_GetTick();
    return HAL_OK;


     //UNCHANGED: Keep all the rest of your existing function exactly as is
     //State transition matrix F
    float F[NAV_STATE_SIZE][NAV_STATE_SIZE];
    NavMatrix_Eye6x6(F);
    F[0][3] = dt;
    F[1][4] = dt;
    F[2][5] = dt;

     //Process noise matrix Q
    float Q[NAV_STATE_SIZE][NAV_STATE_SIZE];
    memset(Q, 0, sizeof(Q));

    float dt3 = dt2 * dt;
    Q[0][0] = ekf_config.process_noise.position * dt3 / 3.0f;
    Q[1][1] = ekf_config.process_noise.position * dt3 / 3.0f;
    Q[2][2] = ekf_config.process_noise.position * dt3 / 3.0f;
    Q[3][3] = ekf_config.process_noise.velocity * dt;
    Q[4][4] = ekf_config.process_noise.velocity * dt;
    Q[5][5] = ekf_config.process_noise.velocity * dt;

     //Covariance prediction
    float FP[NAV_STATE_SIZE][NAV_STATE_SIZE];
    float FPFt[NAV_STATE_SIZE][NAV_STATE_SIZE];

    NavMatrix_Multiply6x6(F, nav_ekf->P, FP);

    for (int i = 0; i < NAV_STATE_SIZE; i++) {
        for (int j = 0; j < NAV_STATE_SIZE; j++) {
            FPFt[i][j] = 0.0f;
            for (int k = 0; k < NAV_STATE_SIZE; k++) {
                FPFt[i][j] += FP[i][k] * F[j][k];
            }
        }
    }

    NavMatrix_Add6x6(FPFt, Q, nav_ekf->P);

    ConditionCovarianceMatrix(nav_ekf->P);

    nav_ekf->update_count++;
    nav_ekf->last_update_time = HAL_GetTick();

    return HAL_OK;
}


/**
 * @brief Update with GPS position
 */
HAL_StatusTypeDef NavEKF_UpdateGPS(NavigationEKF_t *nav_ekf,
                                  double gps_lat, double gps_lon, float gps_alt) {
    if (nav_ekf == NULL || !nav_ekf->initialized) {
        return HAL_ERROR;
    }

    GPS_Data_t current_gps;
    if (!GPS_GetCurrentData(&current_gps)) {
        return HAL_ERROR;
    }

    if (!IsGPSMeasurementValid(&current_gps)) {
        return HAL_ERROR;
    }

    // Convert GPS to NED
    float meas_north, meas_east, meas_down;
    GPS_ToNED(nav_ekf, gps_lat, gps_lon, gps_alt, &meas_north, &meas_east, &meas_down);

    float z[3] = {meas_north, meas_east, meas_down};
    float h[3] = {nav_ekf->state[0], nav_ekf->state[1], nav_ekf->state[2]};

    float H[3][NAV_STATE_SIZE];
    memset(H, 0, sizeof(H));
    H[0][0] = 1.0f;
    H[1][1] = 1.0f;
    H[2][2] = 1.0f;

    // ✅ USE GPS COVARIANCE DIRECTLY - This is the key improvement!
    float R[3][3];
    memset(R, 0, sizeof(R));

    if (current_gps.pos_cov_valid) {
        // Use actual GPS covariance - no scaling needed!
        R[0][0] = current_gps.pos_cov_nn;
        R[1][1] = current_gps.pos_cov_ee;
        R[2][2] = current_gps.pos_cov_dd;

        // Ensure minimum noise floor (much lower now)
        if (R[0][0] < 0.01f) R[0][0] = 0.01f;  // 10cm std dev minimum
        if (R[1][1] < 0.01f) R[1][1] = 0.01f;
        if (R[2][2] < 0.01f) R[2][2] = 0.01f;

    } else {
        // Fallback - trust your GPS accuracy estimates
        float h_var = current_gps.hAcc * current_gps.hAcc;
        float v_var = current_gps.vAcc * current_gps.vAcc;
        R[0][0] = h_var;
        R[1][1] = h_var;
        R[2][2] = v_var;

        // Much more reasonable minimum
        if (R[0][0] < 0.25f) R[0][0] = 0.25f;
        if (R[1][1] < 0.25f) R[1][1] = 0.25f;
        if (R[2][2] < 0.25f) R[2][2] = 0.25f;
    }

    // ✅ MINIMAL SCALING: Your GPS is already excellent
    if (current_gps.hdop > 2.0f) {
        float dop_scale = 1.0f + (current_gps.hdop - 2.0f) * 0.2f;  // Much less aggressive
        for (int i = 0; i < 3; i++) {
            R[i][i] *= dop_scale;
        }
    }

    int indices[3] = {0, 1, 2};
    return EKF_MeasurementUpdate(nav_ekf, z, h, H, R, 3, indices);
}


/**
 * @brief Update with barometric altitude
 */
HAL_StatusTypeDef NavEKF_UpdateBarometer(NavigationEKF_t *nav_ekf, float baro_alt) {
    if (nav_ekf == NULL || !nav_ekf->initialized) {
        return HAL_ERROR;
    }

    /* Convert to NED down */
    float z[1] = {nav_ekf->gps_ref_alt - baro_alt};
    float h[1] = {nav_ekf->state[2]};

    /* Measurement matrix */
    float H[1][NAV_STATE_SIZE];
    memset(H, 0, sizeof(H));
    H[0][2] = 1.0f;

    /* Measurement noise */
    float R[1][3];  /* Padded to match function signature */
    R[0][0] = ekf_config.meas_noise.baro_alt;

    int indices[1] = {2};

    return EKF_MeasurementUpdate(nav_ekf, z, h, (const float (*)[NAV_STATE_SIZE])H,
                                (const float (*)[3])R, 1, indices);
}

/**
 * @brief Update with GPS velocity - backward compatible
 */
HAL_StatusTypeDef NavEKF_UpdateGPSVelocity(NavigationEKF_t *nav_ekf,
                                          float vel_north, float vel_east, float vel_down) {
    /* Calculate GPS speed in m/s (no conversion needed) */
    float gps_speed_ms = sqrtf(vel_north * vel_north + vel_east * vel_east);  // FIXED: remove unused variable

    return NavEKF_UpdateGPSVelocityAdaptive(nav_ekf, vel_north, vel_east, vel_down, gps_speed_ms);
}


/**
 * @brief Enhanced GPS velocity update with all safeguards
 *
 * REPLACE the existing NavEKF_UpdateGPSVelocityAdaptive function with this version
 * (Keep the same function name and signature)
 */
HAL_StatusTypeDef NavEKF_UpdateGPSVelocityAdaptive(NavigationEKF_t *nav_ekf,
                                                  float vel_north, float vel_east, float vel_down,
                                                  float gps_speed_ms) {  // FIXED: parameter name
    if (nav_ekf == NULL || !nav_ekf->initialized) {
        return HAL_ERROR;
    }

     //Get current GPS data for enhanced processing
    GPS_Data_t current_gps;
    if (!GPS_GetCurrentData(&current_gps)) {
         //Enhanced fallback validation
        if (!isfinite(vel_north) || !isfinite(vel_east) || !isfinite(vel_down)) {
            return HAL_ERROR;
        }

        // Check reasonable velocity bounds even without GPS data
        float vel_2d = sqrtf(vel_north * vel_north + vel_east * vel_east);
        if (vel_2d > 100.0f) {  // Conservative limit
            return HAL_ERROR;
        }
    } else {
         //Enhanced GPS velocity validation
        if (!ValidateGPSVelocity(&current_gps, vel_north, vel_east, vel_down)) {
            return HAL_ERROR;  // Reject invalid GPS velocity
        }

         //Enhanced validation for GPS measurement quality
        if (!IsGPSMeasurementValid(&current_gps)) {
            return HAL_ERROR;
        }

         //Check velocity-specific quality
        if (current_gps.sAcc > 5.0f) return HAL_ERROR;
    }

     //Motion state detection
    MotionState_t motion_state = nav_ekf->motion.current_state;

     //Apply ZUPT if appropriate
    if (motion_state == MOTION_STATIONARY) {
        // For stationary motion, apply ZUPT instead of GPS velocity
        if (gps_speed_ms < 0.42f && current_gps.hdop < 2.0f) {  // FIXED: 1.5 km/h = 0.42 m/s
            return NavEKF_UpdateZeroVelocity(nav_ekf);
        }
        // If GPS shows significant motion during stationary, be very conservative
        if (gps_speed_ms > 1.39f) {  // FIXED: 5 km/h = 1.39 m/s
            return HAL_ERROR;  // Likely GPS error
        }
    }

     //Use GPS velocity components (prefer UBX data if available)
    float z[3];
    if (GPS_GetCurrentData(&current_gps)) {
        z[0] = current_gps.velN;
        z[1] = current_gps.velE;
        z[2] = current_gps.velD;
    } else {
        z[0] = vel_north;
        z[1] = vel_east;
        z[2] = vel_down;
    }

    float h[3] = {nav_ekf->state[3], nav_ekf->state[4], nav_ekf->state[5]};

     //Measurement matrix
    float H[3][NAV_STATE_SIZE];
    memset(H, 0, sizeof(H));
    H[0][3] = 1.0f;
    H[1][4] = 1.0f;
    H[2][5] = 1.0f;

     //Enhanced dynamic velocity noise calculation
    float R[3][3];
    memset(R, 0, sizeof(R));

     //Base noise depending on motion state
    float base_noise;
    switch (motion_state) {
        case MOTION_STATIONARY:
            base_noise = ekf_config.meas_noise.gps_vel_stationary;
            break;
        case MOTION_SLOW:
            base_noise = ekf_config.meas_noise.gps_vel_slow;
            break;
        case MOTION_FAST:
            base_noise = ekf_config.meas_noise.gps_vel_fast;
            break;
        default:
            base_noise = 2.0f;
            break;
    }

     //Apply GPS quality scaling if available
    float quality_scale = 1.0f;
    if (GPS_GetCurrentData(&current_gps)) {
        if (current_gps.hdop > 2.0f) quality_scale *= 2.0f;
        if (current_gps.satellites < 6) quality_scale *= 1.5f;
        if (current_gps.sAcc > 2.0f) quality_scale *= (current_gps.sAcc / 2.0f);
    }

    float final_noise = base_noise * quality_scale;

    // Ensure reasonable bounds
    if (final_noise < 0.1f) final_noise = 0.1f;
    if (final_noise > 100.0f) final_noise = 100.0f;

    R[0][0] = final_noise;
    R[1][1] = final_noise;
    R[2][2] = final_noise;

    int indices[3] = {3, 4, 5};

     //Use the enhanced measurement update function
    return EKF_MeasurementUpdate(nav_ekf, z, h, H, R, 3, indices);
}

/**
 * @brief UPDATED: Functions that access motion state now use the motion detector
 */
HAL_StatusTypeDef NavEKF_UpdateZeroVelocity(NavigationEKF_t *nav_ekf) {
    if (nav_ekf == NULL || !nav_ekf->initialized) {
        return HAL_ERROR;
    }

    /* Enhanced ZUPT validation using motion detector */
    GPS_Data_t current_gps;
    if (GPS_GetCurrentData(&current_gps)) {
        bool gps_velocity_suspect;
        MotionDetector_GetResults(&nav_ekf->motion_detector, NULL, NULL, &gps_velocity_suspect);

        if (current_gps.speed > 5.0f && current_gps.sAcc < 2.0f &&
            current_gps.hdop < 2.0f && !gps_velocity_suspect) {
            nav_ekf->debug.zupt_applied = 0; // ZUPT rejected
            return HAL_ERROR;
        }
    }

    /* Get filtered gyro activity from motion detector */
    float avg_accel_var, avg_gyro_activity;
    MotionDetector_GetFiltered(&nav_ekf->motion_detector, &avg_accel_var, &avg_gyro_activity);

    if (avg_gyro_activity > 0.02f) {
        nav_ekf->debug.zupt_applied = 0; // ZUPT rejected
        return HAL_ERROR;
    }

    /* Apply ZUPT (measurement update logic unchanged) */
    float z[3] = {0.0f, 0.0f, 0.0f};
    float h[3] = {nav_ekf->state[3], nav_ekf->state[4], nav_ekf->state[5]};

    float H[3][NAV_STATE_SIZE];
    memset(H, 0, sizeof(H));
    H[0][3] = 1.0f;
    H[1][4] = 1.0f;
    H[2][5] = 1.0f;

    /* Adaptive ZUPT noise (unchanged) */
    float zupt_noise = ekf_config.meas_noise.zupt;

    if (GPS_GetCurrentData(&current_gps)) {
        if (current_gps.hdop > 2.0f) zupt_noise *= 10.0f;
        if (current_gps.satellites < 6) zupt_noise *= 5.0f;
    }

    if (avg_gyro_activity < 0.005f && avg_accel_var < 0.2f) {
        zupt_noise *= 0.1f;
    }

    float R[3][3];
    memset(R, 0, sizeof(R));
    R[0][0] = zupt_noise;
    R[1][1] = zupt_noise;
    R[2][2] = zupt_noise;

    int indices[3] = {3, 4, 5};
    nav_ekf->zupt_count++;

    /* Apply measurement update */
    HAL_StatusTypeDef result = EKF_MeasurementUpdate(nav_ekf, z, h, H, R, 3, indices);

    if (result == HAL_OK) {
        nav_ekf->debug.zupt_applied = 1; // ZUPT successfully applied
    } else {
        nav_ekf->debug.zupt_applied = 0; // ZUPT failed
    }

    return result;
}

/**
 * @brief UPDATED: Get navigation solution with motion detector data
 */
HAL_StatusTypeDef NavEKF_GetSolution(NavigationEKF_t *nav_ekf, NavSolution_t *solution) {
    if (nav_ekf == NULL || solution == NULL || !nav_ekf->initialized) {
        return HAL_ERROR;
    }

    /* Position and velocity (unchanged) */
    for (int i = 0; i < 3; i++) {
        solution->position_ned[i] = nav_ekf->state[i];
        solution->velocity_ned[i] = nav_ekf->state[i + 3];
        solution->position_uncertainty[i] = sqrtf(nav_ekf->P[i][i]);
        solution->velocity_uncertainty[i] = sqrtf(nav_ekf->P[i + 3][i + 3]);
    }

    /* Convert back to GPS (unchanged) */
    if (nav_ekf->gps_reference_set) {
        solution->gps_lat = nav_ekf->gps_ref_lat +
                           (nav_ekf->state[0] / 6378137.0) * (180.0f / M_PI);
        solution->gps_lon = nav_ekf->gps_ref_lon +
                           (nav_ekf->state[1] / (6378137.0 * cos(nav_ekf->gps_ref_lat * M_PI / 180.0))) * (180.0f / M_PI);
        solution->gps_alt = nav_ekf->gps_ref_alt - nav_ekf->state[2];
    } else {
        solution->gps_lat = 0.0;
        solution->gps_lon = 0.0;
        solution->gps_alt = 0.0f;
    }

    /* Derived parameters (unchanged) */
    solution->ground_speed = sqrtf(nav_ekf->state[3] * nav_ekf->state[3] +
                                  nav_ekf->state[4] * nav_ekf->state[4]);
    solution->heading = atan2f(nav_ekf->state[4], nav_ekf->state[3]);
    solution->climb_rate = -nav_ekf->state[5];

    /* UPDATED: Use motion detector state */
    solution->motion_state = nav_ekf->current_motion_state;
    solution->adaptive_gps_noise = nav_ekf->current_gps_vel_noise;
    solution->zupt_active = (nav_ekf->current_motion_state == MOTION_STATIONARY);

    solution->valid = true;
    solution->update_count = nav_ekf->update_count;

    return HAL_OK;
}

/**
 * @brief Reset EKF
 */
HAL_StatusTypeDef NavEKF_Reset(NavigationEKF_t *nav_ekf) {
    if (nav_ekf == NULL) {
        return HAL_ERROR;
    }

    /* Reset state */
    memset(nav_ekf->state, 0, sizeof(nav_ekf->state));

    /* Reset covariance */
    memset(nav_ekf->P, 0, sizeof(nav_ekf->P));
    nav_ekf->P[0][0] = 100.0f;
    nav_ekf->P[1][1] = 100.0f;
    nav_ekf->P[2][2] = 100.0f;
    nav_ekf->P[3][3] = 25.0f;
    nav_ekf->P[4][4] = 25.0f;
    nav_ekf->P[5][5] = 25.0f;

    /* Reset statistics */
    nav_ekf->update_count = 0;
    nav_ekf->zupt_count = 0;
    nav_ekf->last_update_time = HAL_GetTick();

    /* UPDATED: Reset motion detector instead of old static variables */
    MotionDetector_Reset(&nav_ekf->motion_detector);

    /* Reset compatibility layer */
    nav_ekf->current_motion_state = MOTION_UNKNOWN;
    memset(&nav_ekf->motion, 0, sizeof(nav_ekf->motion));
    nav_ekf->motion.current_state = MOTION_UNKNOWN;
    nav_ekf->motion.previous_state = MOTION_UNKNOWN;

    return HAL_OK;
}

/**
 * @brief Get statistics
 */
HAL_StatusTypeDef NavEKF_GetStats(NavigationEKF_t *nav_ekf,
                                 float *position_error_rms,
                                 float *velocity_error_rms,
                                 float *update_rate,
                                 MotionState_t *motion_state,
                                 float *zupt_percentage) {
    if (nav_ekf == NULL) {
        return HAL_ERROR;
    }

    if (position_error_rms != NULL) {
        *position_error_rms = sqrtf((nav_ekf->P[0][0] + nav_ekf->P[1][1] + nav_ekf->P[2][2]) / 3.0f);
    }

    if (velocity_error_rms != NULL) {
        *velocity_error_rms = sqrtf((nav_ekf->P[3][3] + nav_ekf->P[4][4] + nav_ekf->P[5][5]) / 3.0f);
    }

    if (update_rate != NULL) {
        uint32_t elapsed = HAL_GetTick() - nav_ekf->last_update_time;
        if (elapsed > 0 && nav_ekf->update_count > 0) {
            *update_rate = (float)nav_ekf->update_count * 1000.0f / elapsed;
        } else {
            *update_rate = 0.0f;
        }
    }

    if (motion_state != NULL) {
        *motion_state = nav_ekf->motion.current_state;
    }

    if (zupt_percentage != NULL) {
        if (nav_ekf->update_count > 0) {
            *zupt_percentage = (float)nav_ekf->zupt_count * 100.0f / nav_ekf->update_count;
        } else {
            *zupt_percentage = 0.0f;
        }
    }

    return HAL_OK;
}

/**
 * @brief Calibrate accelerometer (placeholder - actual implementation in navigation_manager)
 */
HAL_StatusTypeDef NavEKF_CalibrateAccelerometer(NavigationEKF_t *nav_ekf,
                                               float accel_body_x, float accel_body_y, float accel_body_z) {
    if (nav_ekf == NULL) {
        return HAL_ERROR;
    }

    /* Actual calibration happens in navigation_manager where we have access to AHRS */
    nav_ekf->accel_cal.last_calibration_time = HAL_GetTick();
    nav_ekf->accel_cal.calibration_count++;

    return HAL_OK;
}
