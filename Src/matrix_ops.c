#include "matrix_ops.h"
#include <string.h>

void Matrix3x3_MultiplyVector(float R[3][3], float vec[3], float result[3])
{
    float temp[3];

    temp[0] = R[0][0] * vec[0] + R[0][1] * vec[1] + R[0][2] * vec[2];
    temp[1] = R[1][0] * vec[0] + R[1][1] * vec[1] + R[1][2] * vec[2];
    temp[2] = R[2][0] * vec[0] + R[2][1] * vec[1] + R[2][2] * vec[2];


    memcpy(result, temp, sizeof(float) * 3);
}

void Matrix3x3_Transpose(float mat[3][3], float result[3][3])
{
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            result[j][i] = mat[i][j];
        }
    }
}


void Matrix3x3_Multiply(float A[3][3], float B[3][3], float result[3][3])
{
    float temp[3][3];

    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            temp[i][j] = 0;
            for(int k = 0; k < 3; k++) {
                temp[i][j] += A[i][k] * B[k][j];
            }
        }
    }

    memcpy(result, temp, sizeof(float) * 9);
}
