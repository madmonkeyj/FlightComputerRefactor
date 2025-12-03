/**
  ******************************************************************************
  * @file    navigation_manager.c
  * @brief   Navigation Manager implementation - Adapted for sensor_manager
  * @note    Uses sensor_adapter to bridge with actual hardware (ICM42688/MMC5983MA/BMP581)
  ******************************************************************************
  */

#include "navigation_manager.h"
#include "nav_config.h"
#include "navigation_ekf.h"
#include "sensor_adapter.h"    // Adapter for sensor_manager → navigation format
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "mahony_filter.h"     // For Quaternion_t structure
#include "debug_utils.h"

/* Mathematical constants */
#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)
#define GRAVITY_MAGNITUDE 9.80665f

/* Navigation context - single static instance */
typedef struct {
    /* Core components */
    MahonyFilter_t mahony;
    NavigationEKF_t ekf;

    /* Configuration */
    NavConfig_t config;

    /* State management */
    enum {
        NAV_STATE_UNINITIALIZED,
        NAV_STATE_AHRS_ONLY,
        NAV_STATE_WAITING_GPS,
        NAV_STATE_OPERATIONAL
    } state;

    /* Calibration data */
    struct {
        float accel_bias[3];
        float baro_ref_pressure;
        bool accel_calibrated;
        bool baro_calibrated;
        uint32_t last_cal_time;
    } calibration;

    /* Current solution */
    NavigationSolution_t solution;
    bool solution_valid;

    /* Timing */
    uint32_t ahrs_decimation_counter;

    /* Statistics */
    struct {
        uint32_t ahrs_updates;
        uint32_t ahrs_errors;
        uint32_t ekf_updates;
        uint32_t ekf_errors;
        uint32_t gps_updates;
        uint32_t last_gps_time;
    } stats;
} NavigationContext_t;

/* Single instance */
static NavigationContext_t nav_ctx = {0};

/* EKF stability counter - prevents reading debug data before EKF is stable */
static uint32_t ekf_stable_counter = 0;
#define EKF_STABLE_THRESHOLD 100  // 100 EKF cycles (~2 seconds at 50Hz)

/* Private function prototypes */
static bool CalibrateAccelerometer(void);
static void UpdateNavigationSolution(void);

/**
 * @brief Calibrate accelerometer using AHRS for any orientation
 */
static bool CalibrateAccelerometer(void) {
    /* Check if AHRS is ready */
    if (nav_ctx.state < NAV_STATE_AHRS_ONLY || !nav_ctx.solution.ahrs_valid) {
        return false;
    }

    /* Get current attitude */
    Quaternion_t q = nav_ctx.solution.attitude;

    /* Collect accelerometer samples */
    float accel_sum[3] = {0.0f, 0.0f, 0.0f};
    int valid_samples = 0;
    const int CALIBRATION_SAMPLES = nav_ctx.config.calibration.samples;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        NavSensorData_t cal_data;
        if (SensorAdapter_Read(&cal_data) && cal_data.accel_valid) {
            accel_sum[0] += cal_data.accel[0];
            accel_sum[1] += cal_data.accel[1];
            accel_sum[2] += cal_data.accel[2];
            valid_samples++;

            /* Keep AHRS updated */
            NavigationManager_UpdateAHRS(&cal_data);
        }
        HAL_Delay(10);
    }

    if (valid_samples < CALIBRATION_SAMPLES / 2) {
        return false;
    }

    /* Calculate average */
    float accel_avg[3];
    accel_avg[0] = accel_sum[0] / valid_samples;
    accel_avg[1] = accel_sum[1] / valid_samples;
    accel_avg[2] = accel_sum[2] / valid_samples;

    /* Get final attitude */
    q = nav_ctx.solution.attitude;

    /* Calculate rotation matrix elements for gravity transformation */
    float q0 = q.q0, q1 = q.q1, q2 = q.q2, q3 = q.q3;
    float R13 = 2.0f * (q1*q3 - q0*q2);
    float R23 = 2.0f * (q2*q3 + q0*q1);
    float R33 = 1.0f - 2.0f * (q1*q1 + q2*q2);

    /* Expected gravity in body frame */
    float gravity_body[3];
    gravity_body[0] = R13 * GRAVITY_MAGNITUDE;
    gravity_body[1] = R23 * GRAVITY_MAGNITUDE;
    gravity_body[2] = R33 * GRAVITY_MAGNITUDE;

    /* Calculate bias */
    nav_ctx.calibration.accel_bias[0] = accel_avg[0] - gravity_body[0];
    nav_ctx.calibration.accel_bias[1] = accel_avg[1] - gravity_body[1];
    nav_ctx.calibration.accel_bias[2] = accel_avg[2] - gravity_body[2];

    /* Update EKF calibration */
    nav_ctx.ekf.accel_cal.bias[0] = nav_ctx.calibration.accel_bias[0];
    nav_ctx.ekf.accel_cal.bias[1] = nav_ctx.calibration.accel_bias[1];
    nav_ctx.ekf.accel_cal.bias[2] = nav_ctx.calibration.accel_bias[2];
    nav_ctx.ekf.accel_cal.calibrated = true;

    nav_ctx.calibration.accel_calibrated = true;
    nav_ctx.calibration.last_cal_time = HAL_GetTick();

    return true;
}

/**
 * @brief Update navigation solution structure
 */
static void UpdateNavigationSolution(void) {
    /* Get EKF solution if available */
    if (nav_ctx.state == NAV_STATE_OPERATIONAL) {
        NavSolution_t ekf_solution;
        if (NavEKF_GetSolution(&nav_ctx.ekf, &ekf_solution) == HAL_OK) {
            /* Copy EKF data */
            memcpy(nav_ctx.solution.position_ned, ekf_solution.position_ned, sizeof(float) * 3);
            memcpy(nav_ctx.solution.velocity_ned, ekf_solution.velocity_ned, sizeof(float) * 3);

            nav_ctx.solution.ground_speed = ekf_solution.ground_speed;
            nav_ctx.solution.climb_rate = ekf_solution.climb_rate;
            nav_ctx.solution.navigation_valid = true;

            /* No wind in 6-state EKF */
            memset(nav_ctx.solution.wind_ned, 0, sizeof(float) * 3);
        }
    }
}

/**
 * @brief Initialize Navigation Manager
 */
bool NavigationManager_Init(void) {
    /* Clear context */
    memset(&nav_ctx, 0, sizeof(nav_ctx));

    /* Load configuration */
    NavConfig_GetDefault(&nav_ctx.config);

    /* Initialize Mahony AHRS */
    HAL_StatusTypeDef status = Mahony_InitLaunchPad(&nav_ctx.mahony,
                                                   MAHONY_KP_DEFAULT,
                                                   MAHONY_KI_DEFAULT,
                                                   nav_ctx.config.rates.ahrs_hz);

    if (status != HAL_OK) {
        return false;
    }

    /* State transition */
    nav_ctx.state = NAV_STATE_AHRS_ONLY;
    nav_ctx.solution_valid = false;

    /* EKF will be initialized after GPS fix */

    return true;
}

/**
 * @brief Update AHRS with sensor data
 */
bool NavigationManager_UpdateAHRS(const NavSensorData_t* sensors) {
    if (!sensors || nav_ctx.state < NAV_STATE_AHRS_ONLY) {
        return false;
    }

    /* Validate sensor data */
    if (!sensors->valid || !sensors->accel_valid || !sensors->gyro_valid) {
        nav_ctx.stats.ahrs_errors++;
        return false;
    }

    /* Update Mahony filter */
    HAL_StatusTypeDef status = Mahony_Update(&nav_ctx.mahony,
                                            sensors->gyro[0], sensors->gyro[1], sensors->gyro[2],
                                            sensors->accel[0], sensors->accel[1], sensors->accel[2],
                                            sensors->mag[0], sensors->mag[1], sensors->mag[2]);

    if (status == HAL_OK) {
        /* Update solution */
        Mahony_GetQuaternion(&nav_ctx.mahony, &nav_ctx.solution.attitude);
        Mahony_GetEulerAngles(&nav_ctx.mahony, &nav_ctx.solution.euler);
        Mahony_GetRotationMatrix(&nav_ctx.mahony, &nav_ctx.solution.dcm);

        nav_ctx.solution.ahrs_valid = true;
        nav_ctx.solution.ahrs_update_count++;
        nav_ctx.solution.timestamp = HAL_GetTick();

        /* Calculate heading */
        nav_ctx.solution.heading_deg = nav_ctx.solution.euler.yaw * RAD_TO_DEG;
        if (nav_ctx.solution.heading_deg < 0.0f) {
            nav_ctx.solution.heading_deg += 360.0f;
        }

        nav_ctx.solution_valid = true;
        nav_ctx.stats.ahrs_updates++;

        /* Update Navigation EKF if ready */
        if (nav_ctx.state == NAV_STATE_OPERATIONAL) {
            NavigationManager_UpdateNavigation(sensors);
        }

        return true;
    } else {
        nav_ctx.solution.ahrs_valid = false;
        nav_ctx.stats.ahrs_errors++;
        return false;
    }
}
/**
 * @brief Update Navigation EKF
 */
bool NavigationManager_UpdateNavigation(const NavSensorData_t* sensors) {
    if (!sensors || nav_ctx.state != NAV_STATE_OPERATIONAL || !nav_ctx.solution.ahrs_valid) {
        return false;
    }

    /* Decimation FIRST - skip EKF processing most cycles */
    nav_ctx.ahrs_decimation_counter++;
    if (nav_ctx.ahrs_decimation_counter < nav_ctx.config.rates.ekf_decimation) {
        return true;  // Skip EKF this cycle
    }
    nav_ctx.ahrs_decimation_counter = 0;

    /* Apply accelerometer bias correction */
    float accel_corrected[3];
    if (nav_ctx.calibration.accel_calibrated) {
        accel_corrected[0] = sensors->accel[0] - nav_ctx.calibration.accel_bias[0];
        accel_corrected[1] = sensors->accel[1] - nav_ctx.calibration.accel_bias[1];
        accel_corrected[2] = sensors->accel[2] - nav_ctx.calibration.accel_bias[2];
    } else {
        memcpy(accel_corrected, sensors->accel, sizeof(float) * 3);
    }

    // Motion detection
    float gyro_activity = sqrtf(sensors->gyro[0]*sensors->gyro[0] +
                               sensors->gyro[1]*sensors->gyro[1] +
                               sensors->gyro[2]*sensors->gyro[2]);

    MotionState_t motion_state = NavEKF_DetectMotionState(&nav_ctx.ekf,
                                                         accel_corrected[0], accel_corrected[1], accel_corrected[2],
                                                         sensors->gyro[0], sensors->gyro[1], sensors->gyro[2],
                                                         0.0f);

    /* STEP 1: EKF Prediction */
    HAL_StatusTypeDef status = NavEKF_PredictWithMotionDetection(&nav_ctx.ekf,
                                                               &nav_ctx.solution.attitude,
                                                               accel_corrected[0],
                                                               accel_corrected[1],
                                                               accel_corrected[2],
                                                               sensors->gyro[0],
                                                               sensors->gyro[1],
                                                               sensors->gyro[2]);

    if (status != HAL_OK) {
        nav_ctx.stats.ekf_errors++;
        nav_ctx.solution.navigation_valid = false;
        return false;
    }

    // *** PHASE 2 FIX: Add GPS Position & Velocity Updates Here ***
    GPS_Data_t current_gps;
    if (GPS_GetCurrentData(&current_gps) && current_gps.fix_status == 'A') {

        // GPS Position Updates - Every 5 EKF cycles (~10Hz for stationary, ~10Hz for flight)
        static uint32_t gps_pos_counter = 0;
        if (++gps_pos_counter >= 5) {
            NavEKF_UpdateGPS(&nav_ctx.ekf, current_gps.latitude,
                            current_gps.longitude, current_gps.altitude);
            gps_pos_counter = 0;
        }

        // GPS Velocity Updates - Every cycle when available
        if (current_gps.speed >= 0.0f) {
            NavEKF_UpdateGPSVelocityAdaptive(&nav_ctx.ekf,
                                           current_gps.velN, current_gps.velE, current_gps.velD,
                                           current_gps.speed);
        }
    }

    /* STEP 2: ZUPT Measurement Update (after GPS updates) */
    bool zupt_applied = false;
    bool actually_moving = (gyro_activity > 0.015f);
    bool sensors_stationary = !actually_moving;

    if (sensors_stationary || motion_state == MOTION_STATIONARY) {
        if (NavEKF_UpdateZeroVelocity(&nav_ctx.ekf) == HAL_OK) {
            zupt_applied = true;
        }
    }

    /* Update with barometer if available */
    if (sensors->baro_valid && nav_ctx.calibration.baro_calibrated) {
        float baro_alt = 44330.0f * (1.0f - powf(sensors->pressure / nav_ctx.calibration.baro_ref_pressure, 0.1903f));
        NavEKF_UpdateBarometer(&nav_ctx.ekf, baro_alt);
    }

    /* Update solution */
    UpdateNavigationSolution();
    nav_ctx.stats.ekf_updates++;
    nav_ctx.solution.nav_update_count++;

    /* Increment stability counter */
    if (ekf_stable_counter < EKF_STABLE_THRESHOLD) {
        ekf_stable_counter++;

        // DIAGNOSTIC: Log when EKF becomes stable
        if (ekf_stable_counter == EKF_STABLE_THRESHOLD) {
            DebugPrint("NAV: EKF stabilized after %lu updates - debug data collection enabled\r\n",
                      ekf_stable_counter);
        }
    }

    return true;
}

/**
 * @brief Update with GPS data
 */
bool NavigationManager_UpdateGPS(const GPS_Data_t* gps_data) {
    if (!gps_data || gps_data->fix_status != 'A') {
        return false;
    }

    nav_ctx.stats.gps_updates++;
    nav_ctx.stats.last_gps_time = HAL_GetTick();

    /* Initialize EKF on first GPS fix ONLY */
    if (nav_ctx.state == NAV_STATE_AHRS_ONLY) {
        // DIAGNOSTIC: Log GPS fix received while waiting for EKF init
        static uint32_t gps_fix_log_counter = 0;
        if (gps_fix_log_counter < 5) {
            DebugPrint("NAV: GPS fix received (state=AHRS_ONLY), attempting EKF init...\r\n");
            gps_fix_log_counter++;
        }

        float reference_altitude = (gps_data->altitude > 5.0f && gps_data->altitude < 1000.0f) ?
                                  gps_data->altitude : 128.6f;

        nav_ctx.solution.gps_ref_lat = gps_data->latitude;
        nav_ctx.solution.gps_ref_lon = gps_data->longitude;
        nav_ctx.solution.gps_ref_alt = reference_altitude;

        NavSensorData_t current_sensors;
        bool sensor_read_ok = SensorAdapter_Read(&current_sensors);

        // DIAGNOSTIC: Show why EKF init might fail
        if (!sensor_read_ok) {
            static uint32_t sensor_fail_counter = 0;
            if (sensor_fail_counter < 3) {
                DebugPrint("NAV: EKF init blocked - SensorAdapter_Read() failed\r\n");
                sensor_fail_counter++;
            }
        } else if (!current_sensors.baro_valid) {
            static uint32_t baro_fail_counter = 0;
            if (baro_fail_counter < 3) {
                DebugPrint("NAV: EKF init blocked - barometer not valid\r\n");
                baro_fail_counter++;
            }
        }

        // ⚠️ TEMPORARY: Allow EKF init without valid barometer for testing
        // PRODUCTION: Should require: sensor_read_ok && current_sensors.baro_valid
        bool allow_ekf_init = sensor_read_ok && current_sensors.baro_valid;

        // TESTING WORKAROUND: Allow init even without baro if GPS altitude is reasonable
        if (!allow_ekf_init && reference_altitude > 0.0f && reference_altitude < 5000.0f) {
            allow_ekf_init = true;
            nav_ctx.calibration.baro_ref_pressure = 101325.0f;  // Standard sea level
            nav_ctx.calibration.baro_calibrated = false;
            DebugPrint("NAV: TESTING MODE - EKF init allowed without barometer\r\n");
        }

        if (allow_ekf_init) {
            if (current_sensors.baro_valid) {
                nav_ctx.calibration.baro_ref_pressure = current_sensors.pressure *
                                                       powf(1.0f - (reference_altitude / 44330.0f), -5.255f);
                nav_ctx.calibration.baro_calibrated = true;
            }

            if (!nav_ctx.calibration.accel_calibrated) {
                CalibrateAccelerometer();
            }

            if (NavEKF_Init(&nav_ctx.ekf, nav_ctx.config.rates.ekf_hz) == HAL_OK) {
                if (NavEKF_SetGPSReference(&nav_ctx.ekf, gps_data->latitude,
                                          gps_data->longitude, reference_altitude) == HAL_OK) {

                    nav_ctx.state = NAV_STATE_OPERATIONAL;
                    nav_ctx.solution.gps_reference_set = true;
                    ekf_stable_counter = 0;  // Reset stability counter on initialization

                    // DIAGNOSTIC: Log EKF initialization
                    DebugPrint("NAV: EKF initialized - GPS ref: lat=%.6f lon=%.6f alt=%.1fm\r\n",
                              gps_data->latitude, gps_data->longitude, reference_altitude);
                    DebugPrint("NAV: Debug data collection will start after %d EKF updates (~2 sec)\r\n",
                              EKF_STABLE_THRESHOLD);

                    // Apply first GPS measurement
                    NavEKF_UpdateGPS(&nav_ctx.ekf, gps_data->latitude, gps_data->longitude, gps_data->altitude);

                    if (gps_data->speed >= 0.0f) {
                        NavEKF_UpdateGPSVelocityAdaptive(&nav_ctx.ekf, gps_data->velN, gps_data->velE, gps_data->velD, gps_data->speed);
                    }
                }
            }
        }
    }

    //GPS updates for OPERATIONAL state are handled in NavigationManager_UpdateNavigation()
    //DO NOT add duplicate updates here - causes EKF numerical instability over time

    return true;
}


/**
 * @brief Get current navigation solution
 */
bool NavigationManager_GetSolution(NavigationSolution_t* solution) {
    if (!solution || !nav_ctx.solution_valid) {
        return false;
    }

    *solution = nav_ctx.solution;
    return true;
}

/**
 * @brief Get current attitude
 */
bool NavigationManager_GetAttitude(Quaternion_t* attitude) {
    if (!attitude || !nav_ctx.solution.ahrs_valid) {
        return false;
    }

    *attitude = nav_ctx.solution.attitude;
    return true;
}

/**
 * @brief Get current Euler angles
 */
bool NavigationManager_GetEulerAngles(EulerAngles_t* euler) {
    if (!euler || !nav_ctx.solution.ahrs_valid) {
        return false;
    }

    *euler = nav_ctx.solution.euler;
    return true;
}

/**
 * @brief Get position in NED frame
 */
bool NavigationManager_GetPositionNED(float* north, float* east, float* down) {
    if (!north || !east || !down || !nav_ctx.solution.navigation_valid) {
        return false;
    }

    *north = nav_ctx.solution.position_ned[0];
    *east = nav_ctx.solution.position_ned[1];
    *down = nav_ctx.solution.position_ned[2];
    return true;
}

/**
 * @brief Get velocity in NED frame
 */
bool NavigationManager_GetVelocityNED(float* north, float* east, float* down) {
    if (!north || !east || !down || !nav_ctx.solution.navigation_valid) {
        return false;
    }

    *north = nav_ctx.solution.velocity_ned[0];
    *east = nav_ctx.solution.velocity_ned[1];
    *down = nav_ctx.solution.velocity_ned[2];
    return true;
}

/**
 * @brief Check if GPS reference is set
 */
bool NavigationManager_IsGPSReferenceSet(void) {
    return nav_ctx.state == NAV_STATE_OPERATIONAL && nav_ctx.ekf.gps_reference_set;
}

/**
 * @brief Reset AHRS
 */
void NavigationManager_ResetAHRS(void) {
    if (nav_ctx.state >= NAV_STATE_AHRS_ONLY) {
        Mahony_Reset(&nav_ctx.mahony);
        nav_ctx.solution.ahrs_valid = false;
        nav_ctx.ahrs_decimation_counter = 0;
    }
}

/**
 * @brief Reset Navigation
 */
void NavigationManager_ResetNavigation(void) {
    if (nav_ctx.state == NAV_STATE_OPERATIONAL) {
        NavEKF_Reset(&nav_ctx.ekf);
        nav_ctx.solution.navigation_valid = false;
        nav_ctx.solution.gps_reference_set = false;
        ekf_stable_counter = 0;  // Reset stability counter
    }
}

/**
 * @brief Reset all
 */
void NavigationManager_ResetAll(void) {
    NavigationManager_ResetAHRS();
    NavigationManager_ResetNavigation();

    /* Reset statistics */
    memset(&nav_ctx.stats, 0, sizeof(nav_ctx.stats));
    nav_ctx.solution_valid = false;
}

/**
 * @brief Set AHRS gains
 */
void NavigationManager_SetAHRSGains(float kp, float ki) {
    if (nav_ctx.state >= NAV_STATE_AHRS_ONLY) {
        Mahony_SetGains(&nav_ctx.mahony, kp, ki);
    }
}

/**
 * @brief Verify launch pad orientation
 */
bool NavigationManager_VerifyLaunchPadOrientation(void) {
    if (!nav_ctx.solution.ahrs_valid) {
        return false;
    }

    EulerAngles_t euler = nav_ctx.solution.euler;
    bool roll_ok = fabsf(euler.roll * RAD_TO_DEG) < 10.0f;
    bool pitch_ok = fabsf(euler.pitch * RAD_TO_DEG) < 10.0f;

    return roll_ok && pitch_ok;
}

/**
 * @brief Set initial orientation
 */
bool NavigationManager_SetInitialOrientation(float roll_deg, float pitch_deg, float yaw_deg) {
    if (nav_ctx.state < NAV_STATE_AHRS_ONLY) {
        return false;
    }

    float roll_rad = roll_deg * DEG_TO_RAD;
    float pitch_rad = pitch_deg * DEG_TO_RAD;
    float yaw_rad = yaw_deg * DEG_TO_RAD;

    float cr = cosf(roll_rad * 0.5f);
    float sr = sinf(roll_rad * 0.5f);
    float cp = cosf(pitch_rad * 0.5f);
    float sp = sinf(pitch_rad * 0.5f);
    float cy = cosf(yaw_rad * 0.5f);
    float sy = sinf(yaw_rad * 0.5f);

    Quaternion_t custom_q;
    custom_q.q0 = cr * cp * cy + sr * sp * sy;
    custom_q.q1 = sr * cp * cy - cr * sp * sy;
    custom_q.q2 = cr * sp * cy + sr * cp * sy;
    custom_q.q3 = cr * cp * sy - sr * sp * cy;

    HAL_StatusTypeDef status = Mahony_SetInitialOrientation(&nav_ctx.mahony, &custom_q);

    if (status == HAL_OK) {
        nav_ctx.solution.ahrs_valid = false;
        return true;
    }

    return false;
}

/**
 * @brief Get diagnostics
 */
void NavigationManager_GetDiagnostics(NavigationDiagnostics_t* diag) {
    if (!diag) {
        return;
    }

    /* Health status */
    diag->ahrs_healthy = (nav_ctx.state >= NAV_STATE_AHRS_ONLY) && nav_ctx.solution.ahrs_valid;
    diag->navigation_healthy = (nav_ctx.state == NAV_STATE_OPERATIONAL) && nav_ctx.solution.navigation_valid;
    diag->gps_healthy = (HAL_GetTick() - nav_ctx.stats.last_gps_time) < 10000;

    /* Update rates */
    uint32_t uptime = HAL_GetTick();
    if (uptime > 1000) {
        diag->ahrs_update_rate = (float)nav_ctx.stats.ahrs_updates * 1000.0f / uptime;
        diag->nav_update_rate = (float)nav_ctx.stats.ekf_updates * 1000.0f / uptime;
    } else {
        diag->ahrs_update_rate = 0.0f;
        diag->nav_update_rate = 0.0f;
    }

    /* Error counts */
    diag->ahrs_error_count = nav_ctx.stats.ahrs_errors;
    diag->nav_error_count = nav_ctx.stats.ekf_errors;
    diag->gps_update_count = nav_ctx.stats.gps_updates;
    diag->last_gps_update = nav_ctx.stats.last_gps_time;

    /* Uncertainty */
    if (nav_ctx.state == NAV_STATE_OPERATIONAL) {
        diag->position_uncertainty = sqrtf((nav_ctx.ekf.P[0][0] + nav_ctx.ekf.P[1][1] + nav_ctx.ekf.P[2][2]) / 3.0f);
        diag->velocity_uncertainty = sqrtf((nav_ctx.ekf.P[3][3] + nav_ctx.ekf.P[4][4] + nav_ctx.ekf.P[5][5]) / 3.0f);
    } else {
        diag->position_uncertainty = 999.9f;
        diag->velocity_uncertainty = 999.9f;
    }
}

/**
 * @brief Get diagnostics string
 */
void NavigationManager_GetDiagnosticsString(char* buffer, size_t buffer_size) {
    if (!buffer || buffer_size == 0) {
        return;
    }

    NavigationDiagnostics_t diag;
    NavigationManager_GetDiagnostics(&diag);

    const char* state_str;
    switch (nav_ctx.state) {
        case NAV_STATE_UNINITIALIZED: state_str = "UNINIT"; break;
        case NAV_STATE_AHRS_ONLY: state_str = "AHRS"; break;
        case NAV_STATE_WAITING_GPS: state_str = "WAIT_GPS"; break;
        case NAV_STATE_OPERATIONAL: state_str = "OPERATIONAL"; break;
        default: state_str = "UNKNOWN"; break;
    }

    snprintf(buffer, buffer_size,
        "State: %s\r\n"
        "AHRS: %s (%.1f Hz)\r\n"
        "Nav: %s (%.1f Hz)\r\n"
        "GPS: %s (%lu updates)\r\n"
        "Pos σ: %.1f m\r\n"
        "Vel σ: %.1f m/s\r\n",
        state_str,
        diag.ahrs_healthy ? "OK" : "FAIL", diag.ahrs_update_rate,
        diag.navigation_healthy ? "OK" : "FAIL", diag.nav_update_rate,
        diag.gps_healthy ? "OK" : "STALE", diag.gps_update_count,
        diag.position_uncertainty,
        diag.velocity_uncertainty
    );
}

/**
 * @brief Check if healthy
 */
bool NavigationManager_IsHealthy(void) {
    NavigationDiagnostics_t diag;
    NavigationManager_GetDiagnostics(&diag);
    return diag.ahrs_healthy;
}

/**
 * @brief Get direct access to EKF for diagnostic logging
 * @return Pointer to EKF instance if operational, NULL otherwise
 * @note Use carefully - for logging diagnostics only
 */
NavigationEKF_t* NavigationManager_GetEKF(void) {
    if (nav_ctx.state == NAV_STATE_OPERATIONAL) {
        return &nav_ctx.ekf;
    }
    return NULL;
}

/* === EKF DIAGNOSTIC DATA GETTERS === */

/**
 * @brief Get position uncertainty (standard deviation from EKF covariance)
 */
bool NavigationManager_GetPositionUncertainty(float uncertainty[3]) {
    if (!uncertainty || !nav_ctx.ekf.initialized) return false;

    // SAFETY: Wait for EKF to stabilize before reading debug data
    if (ekf_stable_counter < EKF_STABLE_THRESHOLD) return false;

    // CRITICAL SECTION: Disable interrupts while reading EKF covariance
    // This prevents race condition if EKF updates during read
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    // Copy covariance values atomically
    float p00 = nav_ctx.ekf.P[0][0];
    float p11 = nav_ctx.ekf.P[1][1];
    float p22 = nav_ctx.ekf.P[2][2];

    // Re-enable interrupts immediately
    __set_PRIMASK(primask);

    // SAFETY: Check if covariance values are valid before sqrt
    if (!isfinite(p00) || p00 < 0.0f ||
        !isfinite(p11) || p11 < 0.0f ||
        !isfinite(p22) || p22 < 0.0f) {
        return false;  // Invalid covariance - skip this update
    }

    // Extract std dev from diagonal of covariance matrix (sqrt of variance)
    uncertainty[0] = sqrtf(p00);
    uncertainty[1] = sqrtf(p11);
    uncertainty[2] = sqrtf(p22);

    return true;
}

/**
 * @brief Get velocity uncertainty (standard deviation from EKF covariance)
 */
bool NavigationManager_GetVelocityUncertainty(float uncertainty[3]) {
    if (!uncertainty || !nav_ctx.ekf.initialized) return false;

    // SAFETY: Wait for EKF to stabilize before reading debug data
    if (ekf_stable_counter < EKF_STABLE_THRESHOLD) return false;

    // CRITICAL SECTION: Disable interrupts while reading EKF covariance
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    // Copy covariance values atomically
    float p33 = nav_ctx.ekf.P[3][3];
    float p44 = nav_ctx.ekf.P[4][4];
    float p55 = nav_ctx.ekf.P[5][5];

    // Re-enable interrupts immediately
    __set_PRIMASK(primask);

    // SAFETY: Check if covariance values are valid before sqrt
    if (!isfinite(p33) || p33 < 0.0f ||
        !isfinite(p44) || p44 < 0.0f ||
        !isfinite(p55) || p55 < 0.0f) {
        return false;  // Invalid covariance - skip this update
    }

    // Extract std dev from diagonal of covariance matrix (sqrt of variance)
    uncertainty[0] = sqrtf(p33);
    uncertainty[1] = sqrtf(p44);
    uncertainty[2] = sqrtf(p55);

    return true;
}

/**
 * @brief Get GPS position innovation (GPS measurement - EKF prediction)
 */
bool NavigationManager_GetInnovationPosition(float innovation[3]) {
    if (!innovation || !nav_ctx.ekf.initialized) return false;

    // SAFETY: Wait for EKF to stabilize before reading debug data
    if (ekf_stable_counter < EKF_STABLE_THRESHOLD) return false;

    // CRITICAL SECTION: Disable interrupts while reading debug data
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    bool valid = nav_ctx.ekf.debug.innovation_pos_valid;
    float innov0 = nav_ctx.ekf.debug.innovation_pos[0];
    float innov1 = nav_ctx.ekf.debug.innovation_pos[1];
    float innov2 = nav_ctx.ekf.debug.innovation_pos[2];

    __set_PRIMASK(primask);

    if (!valid) return false;

    // SAFETY: Validate debug data before returning
    if (!isfinite(innov0) || !isfinite(innov1) || !isfinite(innov2)) {
        return false;
    }

    innovation[0] = innov0;
    innovation[1] = innov1;
    innovation[2] = innov2;

    return true;
}

/**
 * @brief Get GPS velocity innovation (GPS measurement - EKF prediction)
 */
bool NavigationManager_GetInnovationVelocity(float innovation[3]) {
    if (!innovation || !nav_ctx.ekf.initialized) return false;
    if (!nav_ctx.ekf.debug.innovation_vel_valid) return false;

    innovation[0] = nav_ctx.ekf.debug.innovation_vel[0];
    innovation[1] = nav_ctx.ekf.debug.innovation_vel[1];
    innovation[2] = nav_ctx.ekf.debug.innovation_vel[2];

    return true;
}

/**
 * @brief Get Kalman gain for position measurements
 */
bool NavigationManager_GetKalmanGainPosition(float gain[3]) {
    if (!gain || !nav_ctx.ekf.initialized) return false;

    // SAFETY: Wait for EKF to stabilize before reading debug data
    if (ekf_stable_counter < EKF_STABLE_THRESHOLD) return false;

    // CRITICAL SECTION: Disable interrupts while reading debug data
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    bool valid = nav_ctx.ekf.debug.kalman_gain_pos_valid;
    float k0 = nav_ctx.ekf.debug.kalman_gain_pos[0];
    float k1 = nav_ctx.ekf.debug.kalman_gain_pos[1];
    float k2 = nav_ctx.ekf.debug.kalman_gain_pos[2];

    __set_PRIMASK(primask);

    if (!valid) return false;

    // SAFETY: Validate debug data before returning
    if (!isfinite(k0) || !isfinite(k1) || !isfinite(k2)) {
        return false;
    }

    gain[0] = k0;
    gain[1] = k1;
    gain[2] = k2;

    return true;
}

/**
 * @brief Get Kalman gain for velocity measurements
 */
bool NavigationManager_GetKalmanGainVelocity(float gain[3]) {
    if (!gain || !nav_ctx.ekf.initialized) return false;

    // SAFETY: Wait for EKF to stabilize before reading debug data
    if (ekf_stable_counter < EKF_STABLE_THRESHOLD) return false;

    // CRITICAL SECTION: Disable interrupts while reading debug data
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    bool valid = nav_ctx.ekf.debug.kalman_gain_vel_valid;
    float k0 = nav_ctx.ekf.debug.kalman_gain_vel[0];
    float k1 = nav_ctx.ekf.debug.kalman_gain_vel[1];
    float k2 = nav_ctx.ekf.debug.kalman_gain_vel[2];

    __set_PRIMASK(primask);

    if (!valid) return false;

    // SAFETY: Validate debug data before returning
    if (!isfinite(k0) || !isfinite(k1) || !isfinite(k2)) {
        return false;
    }

    gain[0] = k0;
    gain[1] = k1;
    gain[2] = k2;

    return true;
}

/**
 * @brief Get accelerometer data transformed to NED frame
 */
bool NavigationManager_GetAccelNED(float accel_ned[3]) {
    if (!accel_ned || !nav_ctx.ekf.initialized) return false;

    // SAFETY: Wait for EKF to stabilize before reading debug data
    if (ekf_stable_counter < EKF_STABLE_THRESHOLD) return false;

    // CRITICAL SECTION: Disable interrupts while reading debug data
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    bool valid = nav_ctx.ekf.debug.accel_ned_valid;
    float a0 = nav_ctx.ekf.debug.accel_ned[0];
    float a1 = nav_ctx.ekf.debug.accel_ned[1];
    float a2 = nav_ctx.ekf.debug.accel_ned[2];

    __set_PRIMASK(primask);

    if (!valid) return false;

    // SAFETY: Validate debug data before returning
    if (!isfinite(a0) || !isfinite(a1) || !isfinite(a2)) {
        return false;
    }

    accel_ned[0] = a0;
    accel_ned[1] = a1;
    accel_ned[2] = a2;

    return true;
}

/**
 * @brief Get measurement rejection flags
 */
bool NavigationManager_GetRejectionFlags(uint8_t* gps_pos_rejected,
                                          uint8_t* gps_vel_rejected,
                                          uint8_t* zupt_applied) {
    if (!gps_pos_rejected || !gps_vel_rejected || !zupt_applied) return false;
    if (!nav_ctx.ekf.initialized) return false;

    // SAFETY: Wait for EKF to stabilize before reading debug data
    if (ekf_stable_counter < EKF_STABLE_THRESHOLD) return false;

    // CRITICAL SECTION: Disable interrupts while reading debug data
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint8_t pos_rej = nav_ctx.ekf.debug.gps_pos_rejected;
    uint8_t vel_rej = nav_ctx.ekf.debug.gps_vel_rejected;
    uint8_t zupt = nav_ctx.ekf.debug.zupt_applied;

    __set_PRIMASK(primask);

    *gps_pos_rejected = pos_rej;
    *gps_vel_rejected = vel_rej;
    *zupt_applied = zupt;

    return true;
}

/**
 * @brief Get motion state and GPS velocity quality
 */
bool NavigationManager_GetMotionState(uint8_t* motion_state, uint8_t* gps_velocity_suspect) {
    if (!motion_state || !gps_velocity_suspect) return false;
    if (!nav_ctx.ekf.initialized) return false;

    // SAFETY: Wait for EKF to stabilize before reading debug data
    if (ekf_stable_counter < EKF_STABLE_THRESHOLD) return false;

    // CRITICAL SECTION: Disable interrupts while reading debug data
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint8_t state = (uint8_t)nav_ctx.ekf.motion.current_state;

    __set_PRIMASK(primask);

    *motion_state = state;
    *gps_velocity_suspect = 0;  // Reserved for future use

    return true;
}
