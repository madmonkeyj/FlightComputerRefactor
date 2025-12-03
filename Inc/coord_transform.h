/**
 ******************************************************************************
 * @file    coord_transform.h
 * @brief   Coordinate Transformation Module - Extracted from Navigation EKF
 * @note    Handles body frame ↔ NED frame transformations and calibration
 ******************************************************************************
 */

#ifndef COORD_TRANSFORM_H
#define COORD_TRANSFORM_H

#include "main.h"
#include "mahony_filter.h"  // For Quaternion_t
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Physical constants */
#define GRAVITY_MAGNITUDE 9.80665f

/**
 * @brief Accelerometer calibration data
 */
typedef struct {
    float bias[3];               /* Bias correction [x, y, z] in body frame */
    float scale[3];              /* Scale factors [x, y, z] */
    bool calibrated;             /* Calibration status */
    uint32_t last_calibration_time;
    uint32_t calibration_count;
} AccelCalibration_t;

/**
 * @brief Coordinate transformation result
 */
typedef struct {
    float accel_ned[3];          /* Transformed acceleration [N, E, D] */
    float accel_ned_with_gravity[3]; /* After gravity removal [N, E, D] */
    bool valid;                  /* Transformation successful */
} CoordTransformResult_t;

/**
 * @brief Body frame sensor reading
 */
typedef struct {
    float accel[3];              /* Raw accelerometer [x, y, z] in body frame */
    float gyro[3];               /* Raw gyroscope [x, y, z] in body frame (for future use) */
    bool accel_valid;            /* Accelerometer data valid */
    bool gyro_valid;             /* Gyroscope data valid */
} BodyFrameReading_t;

/**
 * @brief Initialize coordinate transformation module
 * @return HAL_OK if successful
 */
HAL_StatusTypeDef CoordTransform_Init(void);

/**
 * @brief Transform body frame acceleration to NED frame
 * @param attitude Current attitude quaternion (NED frame)
 * @param accel_body Raw accelerometer reading [x, y, z] in body frame
 * @param calibration Accelerometer calibration data (can be NULL)
 * @param result Pointer to store transformation result
 * @return HAL_OK if successful, HAL_ERROR if invalid inputs
 * @note This applies calibration, transforms to NED, and removes gravity
 */
HAL_StatusTypeDef CoordTransform_BodyToNED(const Quaternion_t* attitude,
                                          const float accel_body[3],
                                          const AccelCalibration_t* calibration,
                                          CoordTransformResult_t* result);

/**
 * @brief Transform body frame acceleration to NED frame (advanced interface)
 * @param attitude Current attitude quaternion (NED frame)
 * @param body_reading Complete body frame sensor reading
 * @param calibration Accelerometer calibration data (can be NULL)
 * @param result Pointer to store transformation result
 * @return HAL_OK if successful, HAL_ERROR if invalid inputs
 */
HAL_StatusTypeDef CoordTransform_BodyToNED_Advanced(const Quaternion_t* attitude,
                                                   const BodyFrameReading_t* body_reading,
                                                   const AccelCalibration_t* calibration,
                                                   CoordTransformResult_t* result);

/**
 * @brief Apply accelerometer calibration to raw readings
 * @param raw_accel Raw accelerometer reading [x, y, z]
 * @param calibration Calibration parameters
 * @param calibrated_accel Output calibrated reading [x, y, z]
 * @return HAL_OK if successful, HAL_ERROR if invalid inputs
 */
HAL_StatusTypeDef CoordTransform_ApplyAccelCalibration(const float raw_accel[3],
                                                      const AccelCalibration_t* calibration,
                                                      float calibrated_accel[3]);

/**
 * @brief Convert quaternion to Direction Cosine Matrix (DCM)
 * @param quaternion Input quaternion
 * @param dcm Output 3x3 DCM matrix (flattened to 9 elements, row-major)
 * @return HAL_OK if successful, HAL_ERROR if invalid inputs
 * @note DCM[i][j] = dcm[i*3 + j]
 */
HAL_StatusTypeDef CoordTransform_QuaternionToDCM(const Quaternion_t* quaternion, float dcm[9]);

/**
 * @brief Transform 3D vector using Direction Cosine Matrix
 * @param vector_in Input vector [x, y, z]
 * @param dcm 3x3 DCM matrix (flattened to 9 elements, row-major)
 * @param vector_out Output transformed vector [x, y, z]
 * @return HAL_OK if successful, HAL_ERROR if invalid inputs
 */
HAL_StatusTypeDef CoordTransform_ApplyDCM(const float vector_in[3], const float dcm[9], float vector_out[3]);

/**
 * @brief Body frame to standard intermediate frame conversion
 * @param body_accel Body frame acceleration [x, y, z]
 * @param std_accel Standard frame acceleration [forward, right, down]
 * @note This handles the specific body frame convention:
 *       Body X+ = UP → Standard Z- = DOWN
 *       Body Y+ = NORTH → Standard X+ = FORWARD
 *       Body Z+ = EAST → Standard Y+ = RIGHT
 */
void CoordTransform_BodyToStandardFrame(const float body_accel[3], float std_accel[3]);

/**
 * @brief Remove gravity from NED acceleration
 * @param accel_ned NED acceleration [N, E, D] in m/s²
 * @param accel_no_gravity Output acceleration with gravity removed [N, E, D]
 * @note Adds GRAVITY_MAGNITUDE to the Down component
 */
void CoordTransform_RemoveGravity(const float accel_ned[3], float accel_no_gravity[3]);

/**
 * @brief Validate quaternion for coordinate transformation
 * @param quaternion Quaternion to validate
 * @return true if quaternion is valid for transformation, false otherwise
 */
bool CoordTransform_ValidateQuaternion(const Quaternion_t* quaternion);

/**
 * @brief Get coordinate transformation statistics
 * @param total_transforms Pointer to store total transformation count
 * @param failed_transforms Pointer to store failed transformation count
 * @param last_transform_time Pointer to store last transformation timestamp
 */
void CoordTransform_GetStats(uint32_t* total_transforms,
                            uint32_t* failed_transforms,
                            uint32_t* last_transform_time);

/**
 * @brief Reset coordinate transformation statistics
 */
void CoordTransform_ResetStats(void);

/**
 * @brief Debug: Step-by-step coordinate transformation verification
 * @param attitude Current attitude quaternion
 * @param accel_body Body frame acceleration [x, y, z]
 * @param calibration Calibration data (can be NULL)
 * @return HAL_OK if debug analysis successful
 * @note This function provides detailed debug output for coordinate transform verification
 */
HAL_StatusTypeDef CoordTransform_DebugAnalysis(const Quaternion_t* attitude,
                                              const float accel_body[3],
                                              const AccelCalibration_t* calibration);

#ifdef __cplusplus
}
#endif

#endif /* COORD_TRANSFORM_H */
