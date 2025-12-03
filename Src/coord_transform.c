/**
 ******************************************************************************
 * @file    coord_transform.c
 * @brief   Coordinate Transformation Module Implementation
 * @note    Extracted and consolidated from Navigation EKF
 ******************************************************************************
 */

#include "coord_transform.h"
#include "debug_utils.h"
#include <math.h>
#include <string.h>

/* Statistics tracking */
static uint32_t total_transforms = 0;
static uint32_t failed_transforms = 0;
static uint32_t last_transform_time = 0;
static bool module_initialized = false;

/**
 * @brief Initialize coordinate transformation module
 */
HAL_StatusTypeDef CoordTransform_Init(void) {
    /* Reset statistics */
    total_transforms = 0;
    failed_transforms = 0;
    last_transform_time = 0;
    module_initialized = true;

    return HAL_OK;
}

/**
 * @brief Validate quaternion for coordinate transformation
 */
bool CoordTransform_ValidateQuaternion(const Quaternion_t* quaternion) {
    if (!quaternion) {
        return false;
    }

    /* Check for finite values */
    if (!isfinite(quaternion->q0) || !isfinite(quaternion->q1) ||
        !isfinite(quaternion->q2) || !isfinite(quaternion->q3)) {
        return false;
    }

    /* Check quaternion norm (should be close to 1.0) */
    float norm = sqrtf(quaternion->q0 * quaternion->q0 + quaternion->q1 * quaternion->q1 +
                       quaternion->q2 * quaternion->q2 + quaternion->q3 * quaternion->q3);

    if (norm < 0.1f || norm > 2.0f) {
        return false;
    }

    return true;
}

/**
 * @brief Apply accelerometer calibration to raw readings
 */
HAL_StatusTypeDef CoordTransform_ApplyAccelCalibration(const float raw_accel[3],
                                                      const AccelCalibration_t* calibration,
                                                      float calibrated_accel[3]) {
    if (!raw_accel || !calibrated_accel) {
        return HAL_ERROR;
    }

    /* If no calibration provided or not calibrated, pass through raw values */
    if (!calibration || !calibration->calibrated) {
        calibrated_accel[0] = raw_accel[0];
        calibrated_accel[1] = raw_accel[1];
        calibrated_accel[2] = raw_accel[2];
        return HAL_OK;
    }

    /* Apply bias correction and scaling */
    calibrated_accel[0] = (raw_accel[0] - calibration->bias[0]) * calibration->scale[0];
    calibrated_accel[1] = (raw_accel[1] - calibration->bias[1]) * calibration->scale[1];
    calibrated_accel[2] = (raw_accel[2] - calibration->bias[2]) * calibration->scale[2];

    return HAL_OK;
}

/**
 * @brief Body frame to standard intermediate frame conversion
 * @note This implements the specific body frame convention from the original EKF:
 *       Body Frame: X+ = UP, Y+ = NORTH, Z+ = EAST
 *       Standard Frame: X = FORWARD (North), Y = RIGHT (East), Z = DOWN
 */
void CoordTransform_BodyToStandardFrame(const float body_accel[3], float std_accel[3]) {
    if (!body_accel || !std_accel) {
        return;
    }

    /* Original transformation from navigation_ekf.c */
    std_accel[0] = body_accel[1];   // Forward (North) = +Body Y
    std_accel[1] = body_accel[2];   // Right (East) = +Body Z
    std_accel[2] = -body_accel[0];  // Down = -Body X (CRITICAL: this is the -1.0f fix)
}

/**
 * @brief Convert quaternion to Direction Cosine Matrix
 */
HAL_StatusTypeDef CoordTransform_QuaternionToDCM(const Quaternion_t* quaternion, float dcm[9]) {
    if (!quaternion || !dcm) {
        return HAL_ERROR;
    }

    if (!CoordTransform_ValidateQuaternion(quaternion)) {
        return HAL_ERROR;
    }

    /* Extract quaternion components */
    float q0 = quaternion->q0, q1 = quaternion->q1, q2 = quaternion->q2, q3 = quaternion->q3;

    /* Calculate quaternion products (from original EKF) */
    float q0q0 = q0*q0, q1q1 = q1*q1, q2q2 = q2*q2, q3q3 = q3*q3;
    float q0q1 = q0*q1, q0q2 = q0*q2, q0q3 = q0*q3;
    float q1q2 = q1*q2, q1q3 = q1*q3, q2q3 = q2*q3;

    /* Build DCM matrix (row-major order) */
    dcm[0] = q0q0 + q1q1 - q2q2 - q3q3;  // DCM[0][0]
    dcm[1] = 2.0f * (q1q2 - q0q3);       // DCM[0][1]
    dcm[2] = 2.0f * (q1q3 + q0q2);       // DCM[0][2]

    dcm[3] = 2.0f * (q1q2 + q0q3);       // DCM[1][0]
    dcm[4] = q0q0 - q1q1 + q2q2 - q3q3;  // DCM[1][1]
    dcm[5] = 2.0f * (q2q3 - q0q1);       // DCM[1][2]

    dcm[6] = 2.0f * (q1q3 - q0q2);       // DCM[2][0]
    dcm[7] = 2.0f * (q2q3 + q0q1);       // DCM[2][1]
    dcm[8] = q0q0 - q1q1 - q2q2 + q3q3;  // DCM[2][2]

    return HAL_OK;
}

/**
 * @brief Transform 3D vector using Direction Cosine Matrix
 */
HAL_StatusTypeDef CoordTransform_ApplyDCM(const float vector_in[3], const float dcm[9], float vector_out[3]) {
    if (!vector_in || !dcm || !vector_out) {
        return HAL_ERROR;
    }

    /* Validate DCM elements are finite */
    for (int i = 0; i < 9; i++) {
        if (!isfinite(dcm[i])) {
            return HAL_ERROR;
        }
    }

    /* Apply DCM transformation: vector_out = DCM * vector_in */
    vector_out[0] = dcm[0] * vector_in[0] + dcm[1] * vector_in[1] + dcm[2] * vector_in[2];
    vector_out[1] = dcm[3] * vector_in[0] + dcm[4] * vector_in[1] + dcm[5] * vector_in[2];
    vector_out[2] = dcm[6] * vector_in[0] + dcm[7] * vector_in[1] + dcm[8] * vector_in[2];

    /* Validate output */
    if (!isfinite(vector_out[0]) || !isfinite(vector_out[1]) || !isfinite(vector_out[2])) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Remove gravity from NED acceleration
 */
void CoordTransform_RemoveGravity(const float accel_ned[3], float accel_no_gravity[3]) {
    if (!accel_ned || !accel_no_gravity) {
        return;
    }

    /* Copy North and East components unchanged */
    accel_no_gravity[0] = accel_ned[0];
    accel_no_gravity[1] = accel_ned[1];

    /* Remove gravity from Down component */
    accel_no_gravity[2] = accel_ned[2] + GRAVITY_MAGNITUDE;
}

/**
 * @brief Transform body frame acceleration to NED frame
 * @note This implements the exact same logic as the original EKF prediction function
 */
HAL_StatusTypeDef CoordTransform_BodyToNED(const Quaternion_t* attitude,
                                          const float accel_body[3],
                                          const AccelCalibration_t* calibration,
                                          CoordTransformResult_t* result) {
    if (!attitude || !accel_body || !result) {
        if (module_initialized) failed_transforms++;
        return HAL_ERROR;
    }

    /* Clear result */
    memset(result, 0, sizeof(CoordTransformResult_t));

    /* Validate inputs */
    if (!CoordTransform_ValidateQuaternion(attitude)) {
        if (module_initialized) failed_transforms++;
        return HAL_ERROR;
    }

    /* Check for finite accelerometer values */
    if (!isfinite(accel_body[0]) || !isfinite(accel_body[1]) || !isfinite(accel_body[2])) {
        if (module_initialized) failed_transforms++;
        return HAL_ERROR;
    }

    /* Step 1: Apply accelerometer calibration */
    float calibrated_accel[3];
    if (CoordTransform_ApplyAccelCalibration(accel_body, calibration, calibrated_accel) != HAL_OK) {
        if (module_initialized) failed_transforms++;
        return HAL_ERROR;
    }

    /* Step 2: Convert to standard intermediate frame */
    float std_accel[3];
    CoordTransform_BodyToStandardFrame(calibrated_accel, std_accel);

    /* Step 3: Convert quaternion to DCM */
    float dcm[9];
    if (CoordTransform_QuaternionToDCM(attitude, dcm) != HAL_OK) {
        if (module_initialized) failed_transforms++;
        return HAL_ERROR;
    }

    /* Step 4: Apply DCM transformation to get NED */
    if (CoordTransform_ApplyDCM(std_accel, dcm, result->accel_ned) != HAL_OK) {
        if (module_initialized) failed_transforms++;
        return HAL_ERROR;
    }

    /* Step 5: Remove gravity */
    CoordTransform_RemoveGravity(result->accel_ned, result->accel_ned_with_gravity);

    /* Update statistics */
    if (module_initialized) {
        total_transforms++;
        last_transform_time = HAL_GetTick();
    }

    result->valid = true;
    return HAL_OK;
}

/**
 * @brief Transform body frame acceleration to NED frame (advanced interface)
 */
HAL_StatusTypeDef CoordTransform_BodyToNED_Advanced(const Quaternion_t* attitude,
                                                   const BodyFrameReading_t* body_reading,
                                                   const AccelCalibration_t* calibration,
                                                   CoordTransformResult_t* result) {
    if (!body_reading || !body_reading->accel_valid) {
        return HAL_ERROR;
    }

    return CoordTransform_BodyToNED(attitude, body_reading->accel, calibration, result);
}

/**
 * @brief Get coordinate transformation statistics
 */
void CoordTransform_GetStats(uint32_t* total_transforms_out,
                            uint32_t* failed_transforms_out,
                            uint32_t* last_transform_time_out) {
    if (total_transforms_out) {
        *total_transforms_out = total_transforms;
    }

    if (failed_transforms_out) {
        *failed_transforms_out = failed_transforms;
    }

    if (last_transform_time_out) {
        *last_transform_time_out = last_transform_time;
    }
}

/**
 * @brief Reset coordinate transformation statistics
 */
void CoordTransform_ResetStats(void) {
    total_transforms = 0;
    failed_transforms = 0;
    last_transform_time = 0;
}
