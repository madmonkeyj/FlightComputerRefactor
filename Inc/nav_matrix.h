// nav_matrix.h
#ifndef NAV_MATRIX_H
#define NAV_MATRIX_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

// Matrix operations for 6-state EKF
#define NAV_STATE_SIZE_NEW 6

// Generic matrix multiply: C = A * B
void NavMatrix_Multiply(const float* A, const float* B, float* C,
                       int rows_A, int cols_A, int cols_B);

// Square matrix operations
void NavMatrix_Multiply6x6(const float A[6][6], const float B[6][6], float C[6][6]);
void NavMatrix_Copy6x6(const float src[6][6], float dst[6][6]);
void NavMatrix_Eye6x6(float M[6][6]);
void NavMatrix_Add6x6(const float A[6][6], const float B[6][6], float C[6][6]);

// 3x3 matrix inversion (for measurement updates)
HAL_StatusTypeDef NavMatrix_Invert3x3(const float in[3][3], float out[3][3]);

// Specialized operations for EKF
void NavMatrix_ComputeKalmanGain(const float P[6][6], const float* H, const float* R,
                                float* K, int meas_dim);

#endif /* NAV_MATRIX_H */
