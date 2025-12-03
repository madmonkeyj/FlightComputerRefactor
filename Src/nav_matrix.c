// nav_matrix.c
#include "nav_matrix.h"
#include <string.h>
#include <math.h>

void NavMatrix_Multiply(const float* A, const float* B, float* C,
                       int rows_A, int cols_A, int cols_B) {
    for (int i = 0; i < rows_A; i++) {
        for (int j = 0; j < cols_B; j++) {
            C[i * cols_B + j] = 0.0f;
            for (int k = 0; k < cols_A; k++) {
                C[i * cols_B + j] += A[i * cols_A + k] * B[k * cols_B + j];
            }
        }
    }
}

void NavMatrix_Multiply6x6(const float A[6][6], const float B[6][6], float C[6][6]) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            C[i][j] = 0.0f;
            for (int k = 0; k < 6; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void NavMatrix_Copy6x6(const float src[6][6], float dst[6][6]) {
    memcpy(dst, src, sizeof(float) * 36);
}

void NavMatrix_Eye6x6(float M[6][6]) {
    memset(M, 0, sizeof(float) * 36);
    for (int i = 0; i < 6; i++) {
        M[i][i] = 1.0f;
    }
}

void NavMatrix_Add6x6(const float A[6][6], const float B[6][6], float C[6][6]) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

HAL_StatusTypeDef NavMatrix_Invert3x3(const float in[3][3], float out[3][3]) {
    // Calculate determinant
    float det = in[0][0] * (in[1][1] * in[2][2] - in[1][2] * in[2][1]) -
                in[0][1] * (in[1][0] * in[2][2] - in[1][2] * in[2][0]) +
                in[0][2] * (in[1][0] * in[2][1] - in[1][1] * in[2][0]);

    if (fabsf(det) < 1e-6f) {
        return HAL_ERROR;
    }

    float inv_det = 1.0f / det;

    out[0][0] = (in[1][1] * in[2][2] - in[1][2] * in[2][1]) * inv_det;
    out[0][1] = (in[0][2] * in[2][1] - in[0][1] * in[2][2]) * inv_det;
    out[0][2] = (in[0][1] * in[1][2] - in[0][2] * in[1][1]) * inv_det;
    out[1][0] = (in[1][2] * in[2][0] - in[1][0] * in[2][2]) * inv_det;
    out[1][1] = (in[0][0] * in[2][2] - in[0][2] * in[2][0]) * inv_det;
    out[1][2] = (in[0][2] * in[1][0] - in[0][0] * in[1][2]) * inv_det;
    out[2][0] = (in[1][0] * in[2][1] - in[1][1] * in[2][0]) * inv_det;
    out[2][1] = (in[0][1] * in[2][0] - in[0][0] * in[2][1]) * inv_det;
    out[2][2] = (in[0][0] * in[1][1] - in[0][1] * in[1][0]) * inv_det;

    return HAL_OK;
}

void NavMatrix_ComputeKalmanGain(const float P[6][6], const float* H, const float* R,
                                float* K, int meas_dim) {
    // This function would compute the Kalman gain
    // K = P * H' * inv(H * P * H' + R)
    // Implementation depends on measurement dimension
    // For now, this is handled in the EKF measurement update function
}
