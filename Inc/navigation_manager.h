/**
  ******************************************************************************
  * @file    navigation_manager.h
  * @brief   Navigation Manager - AHRS + EKF coordination (Phase 2)
  * @note    Handles Mahony filter + Navigation EKF integration
  ******************************************************************************
  */

#ifndef NAVIGATION_MANAGER_H_
#define NAVIGATION_MANAGER_H_

#include "main.h"
#include "sensor_adapter.h"  // For NavSensorData_t (adapter for actual hardware)
#include "mahony_filter.h"
#include "navigation_ekf.h"
#include "gps_module.h"     // For GPS_Data_t
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Complete navigation solution combining AHRS and EKF
 */
typedef struct {
    // AHRS data (500Hz updates)
    Quaternion_t attitude;       // Current attitude quaternion (NED frame)
    EulerAngles_t euler;         // Euler angles in radians
    RotationMatrix_t dcm;        // Direction cosine matrix
    bool ahrs_valid;             // AHRS solution valid

    // Navigation data (50Hz updates)
    float position_ned[3];       // Position [North, East, Down] in meters
    float velocity_ned[3];       // Velocity [North, East, Down] in m/s
    float wind_ned[3];           // Wind estimate [North, East, Down] in m/s
    bool navigation_valid;       // Navigation solution valid

    // Derived navigation parameters
    float ground_speed;          // 2D ground speed (m/s)
    float heading_deg;           // Heading angle (degrees, North=0)
    float climb_rate;            // Vertical speed (m/s, positive=up)

    // GPS reference status
    bool gps_reference_set;      // GPS reference frame established
    double gps_ref_lat;          // Reference latitude (degrees)
    double gps_ref_lon;          // Reference longitude (degrees)
    float gps_ref_alt;           // Reference altitude (meters)

    // Solution quality and timing
    uint32_t ahrs_update_count;  // Number of AHRS updates
    uint32_t nav_update_count;   // Number of navigation updates
    uint32_t timestamp;          // Last update timestamp

} NavigationSolution_t;

/**
 * @brief Navigation system health and performance metrics
 */
typedef struct {
    bool ahrs_healthy;           // AHRS filter operating normally
    bool navigation_healthy;     // EKF operating normally
    bool gps_healthy;            // GPS providing valid data

    float ahrs_update_rate;      // Actual AHRS update rate (Hz)
    float nav_update_rate;       // Actual navigation update rate (Hz)

    uint32_t ahrs_error_count;   // AHRS update failures
    uint32_t nav_error_count;    // Navigation update failures
    uint32_t gps_update_count;   // GPS updates received

    uint32_t last_gps_update;    // Time of last GPS update
    float position_uncertainty;  // RMS position error (meters)
    float velocity_uncertainty;  // RMS velocity error (m/s)

} NavigationDiagnostics_t;

/**
 * @brief Initialize the Navigation Manager
 * @retval true if initialization successful
 * @retval false if critical failure
 */
bool NavigationManager_Init(void);

/**
 * @brief Update AHRS with sensor data - call at 500Hz
 * @param sensors Pointer to current sensor data
 * @retval true if AHRS update successful
 * @retval false if update failed
 */
bool NavigationManager_UpdateAHRS(const NavSensorData_t* sensors);

/**
 * @brief Update Navigation EKF - call at 50Hz (decimated from AHRS)
 * @param sensors Pointer to current sensor data
 * @retval true if navigation update successful
 * @retval false if update failed
 * @note Automatically handles timing decimation from AHRS rate
 */
bool NavigationManager_UpdateNavigation(const NavSensorData_t* sensors);

/**
 * @brief Update navigation with GPS data - call when GPS data available
 * @param gps_data Pointer to GPS data structure
 * @retval true if GPS update successful
 * @retval false if GPS data invalid or update failed
 */
bool NavigationManager_UpdateGPS(const GPS_Data_t* gps_data);

/**
 * @brief Get current complete navigation solution
 * @param solution Pointer to NavigationSolution_t to fill
 * @retval true if solution valid and available
 * @retval false if no valid solution
 */
bool NavigationManager_GetSolution(NavigationSolution_t* solution);

/**
 * @brief Get current attitude (high rate data)
 * @param attitude Pointer to Quaternion_t to fill
 * @retval true if attitude valid
 * @retval false if AHRS not ready
 */
bool NavigationManager_GetAttitude(Quaternion_t* attitude);

/**
 * @brief Get current Euler angles (high rate data)
 * @param euler Pointer to EulerAngles_t to fill
 * @retval true if angles valid
 * @retval false if AHRS not ready
 */
bool NavigationManager_GetEulerAngles(EulerAngles_t* euler);

/**
 * @brief Get current position in NED frame
 * @param north Pointer to North position (meters)
 * @param east Pointer to East position (meters)
 * @param down Pointer to Down position (meters)
 * @retval true if position valid
 * @retval false if navigation not ready
 */
bool NavigationManager_GetPositionNED(float* north, float* east, float* down);

/**
 * @brief Get current velocity in NED frame
 * @param north Pointer to North velocity (m/s)
 * @param east Pointer to East velocity (m/s)
 * @param down Pointer to Down velocity (m/s)
 * @retval true if velocity valid
 * @retval false if navigation not ready
 */
bool NavigationManager_GetVelocityNED(float* north, float* east, float* down);

/**
 * @brief Check if GPS reference has been set
 * @retval true if GPS reference established
 * @retval false if waiting for GPS reference
 */
bool NavigationManager_IsGPSReferenceSet(void);

/**
 * @brief Reset AHRS filter to initial state
 */
void NavigationManager_ResetAHRS(void);

/**
 * @brief Reset Navigation EKF to initial state
 */
void NavigationManager_ResetNavigation(void);

/**
 * @brief Reset entire navigation system
 */
void NavigationManager_ResetAll(void);

/**
 * @brief Set AHRS filter gains
 * @param kp Proportional gain
 * @param ki Integral gain
 */
void NavigationManager_SetAHRSGains(float kp, float ki);

/**
 * @brief Verify launch pad orientation (rocket pointing up)
 * @retval true if orientation is acceptable for launch
 * @retval false if orientation needs adjustment
 */
bool NavigationManager_VerifyLaunchPadOrientation(void);

/**
 * @brief Set custom initial orientation
 * @param roll_deg Roll angle in degrees
 * @param pitch_deg Pitch angle in degrees
 * @param yaw_deg Yaw angle in degrees
 * @retval true if orientation set successfully
 * @retval false if failed to set orientation
 */
bool NavigationManager_SetInitialOrientation(float roll_deg, float pitch_deg, float yaw_deg);

/**
 * @brief Get navigation system diagnostics
 * @param diag Pointer to NavigationDiagnostics_t to fill
 */
void NavigationManager_GetDiagnostics(NavigationDiagnostics_t* diag);

/**
 * @brief Get formatted diagnostics string for debug output
 * @param buffer Buffer to write diagnostics
 * @param buffer_size Size of buffer
 */
void NavigationManager_GetDiagnosticsString(char* buffer, size_t buffer_size);

/**
 * @brief Check if navigation system is healthy
 * @retval true if system operating normally
 * @retval false if degraded performance detected
 */
bool NavigationManager_IsHealthy(void);

void NavigationManager_EnableDebugMode(bool enable_debug, bool force_zupt);

/**
 * @brief Get direct access to EKF for diagnostic purposes
 * @return Pointer to EKF instance if operational, NULL otherwise
 * @warning Use only for reading diagnostic data - do not modify EKF state
 */
NavigationEKF_t* NavigationManager_GetEKF(void);

/* === EKF DIAGNOSTIC DATA GETTERS === */

/**
 * @brief Get position uncertainty (standard deviation from EKF covariance)
 * @param uncertainty Output array[3] for position std dev (N, E, D) in meters
 * @retval true if uncertainty valid
 * @retval false if EKF not initialized
 */
bool NavigationManager_GetPositionUncertainty(float uncertainty[3]);

/**
 * @brief Get velocity uncertainty (standard deviation from EKF covariance)
 * @param uncertainty Output array[3] for velocity std dev (N, E, D) in m/s
 * @retval true if uncertainty valid
 * @retval false if EKF not initialized
 */
bool NavigationManager_GetVelocityUncertainty(float uncertainty[3]);

/**
 * @brief Get GPS position innovation (GPS measurement - EKF prediction)
 * @param innovation Output array[3] for innovation (N, E, D) in meters
 * @retval true if innovation valid
 * @retval false if no recent GPS update
 */
bool NavigationManager_GetInnovationPosition(float innovation[3]);

/**
 * @brief Get GPS velocity innovation (GPS measurement - EKF prediction)
 * @param innovation Output array[3] for innovation (N, E, D) in m/s
 * @retval true if innovation valid
 * @retval false if no recent GPS update
 */
bool NavigationManager_GetInnovationVelocity(float innovation[3]);

/**
 * @brief Get Kalman gain for position measurements
 * @param gain Output array[3] for Kalman gains (N, E, D)
 * @retval true if gain valid
 * @retval false if no recent GPS update
 */
bool NavigationManager_GetKalmanGainPosition(float gain[3]);

/**
 * @brief Get Kalman gain for velocity measurements
 * @param gain Output array[3] for Kalman gains (N, E, D)
 * @retval true if gain valid
 * @retval false if no recent GPS update
 */
bool NavigationManager_GetKalmanGainVelocity(float gain[3]);

/**
 * @brief Get accelerometer data transformed to NED frame
 * @param accel_ned Output array[3] for accel (N, E, D) in m/s²
 * @retval true if transform valid
 * @retval false if AHRS not initialized
 */
bool NavigationManager_GetAccelNED(float accel_ned[3]);

/**
 * @brief Get measurement rejection flags
 * @param gps_pos_rejected Output: 1 if GPS position rejected this cycle
 * @param gps_vel_rejected Output: 1 if GPS velocity rejected this cycle
 * @param zupt_applied Output: 1 if ZUPT applied this cycle
 * @retval true if flags valid
 * @retval false if EKF not initialized
 */
bool NavigationManager_GetRejectionFlags(uint8_t* gps_pos_rejected,
                                          uint8_t* gps_vel_rejected,
                                          uint8_t* zupt_applied);

/**
 * @brief Get motion state and GPS velocity quality
 * @param motion_state Output: 0=STATIONARY, 1=SLOW, 2=FAST, 3=UNKNOWN
 * @param gps_velocity_suspect Output: 1 if GPS velocity suspect (reserved for future use)
 * @retval true if state valid
 * @retval false if EKF not initialized
 */
bool NavigationManager_GetMotionState(uint8_t* motion_state, uint8_t* gps_velocity_suspect);


#endif /* NAVIGATION_MANAGER_H_ */
