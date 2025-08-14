#include "MahonyAHRS.h"
#include <math.h>
#include <string.h>

void Mahony_Init(MahonyAHRS* ahrs, float samplePeriod, float Kp, float Ki) {
    ahrs->SamplePeriod = samplePeriod;
    ahrs->Quaternion[0] = 1.0f; // w
    ahrs->Quaternion[1] = 0.0f; // x
    ahrs->Quaternion[2] = 0.0f; // y
    ahrs->Quaternion[3] = 0.0f; // z
    ahrs->Kp = Kp;
    ahrs->Ki = Ki;
    memset(ahrs->eInt, 0, sizeof(ahrs->eInt));
}

void Mahony_UpdateIMU(MahonyAHRS* ahrs, float gyro[3], float accel[3]) {
    float q[4], qDot[4];
    float v[3], e[3];
    float accelNorm;

    // Normalize accelerometer measurement
    accelNorm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    if (accelNorm == 0.0f) return;
    accel[0] /= accelNorm;
    accel[1] /= accelNorm;
    accel[2] /= accelNorm;


    memcpy(q, ahrs->Quaternion, sizeof(q));

    // Yerçekimi yönü tahmini
    v[0] = 2.0f * (q[1]*q[3] - q[0]*q[2]);
    v[1] = 2.0f * (q[0]*q[1] + q[2]*q[3]);
    v[2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];


    Cross_Product(e, accel, v);


    if (ahrs->Ki > 0.0f) {
        ahrs->eInt[0] += e[0] * ahrs->SamplePeriod;
        ahrs->eInt[1] += e[1] * ahrs->SamplePeriod;
        ahrs->eInt[2] += e[2] * ahrs->SamplePeriod;
    } else {
        memset(ahrs->eInt, 0, sizeof(ahrs->eInt));
    }


    gyro[0] += ahrs->Kp * e[0] + ahrs->Ki * ahrs->eInt[0];
    gyro[1] += ahrs->Kp * e[1] + ahrs->Ki * ahrs->eInt[1];
    gyro[2] += ahrs->Kp * e[2] + ahrs->Ki * ahrs->eInt[2];


    qDot[0] = 0.5f * (-q[1]*gyro[0] - q[2]*gyro[1] - q[3]*gyro[2]);
    qDot[1] = 0.5f * ( q[0]*gyro[0] + q[2]*gyro[2] - q[3]*gyro[1]);
    qDot[2] = 0.5f * ( q[0]*gyro[1] - q[1]*gyro[2] + q[3]*gyro[0]);
    qDot[3] = 0.5f * ( q[0]*gyro[2] + q[1]*gyro[1] - q[2]*gyro[0]);


    q[0] += qDot[0] * ahrs->SamplePeriod;
    q[1] += qDot[1] * ahrs->SamplePeriod;
    q[2] += qDot[2] * ahrs->SamplePeriod;
    q[3] += qDot[3] * ahrs->SamplePeriod;


    Quaternion_Normalize(q);
    memcpy(ahrs->Quaternion, q, sizeof(q));
}


void Quaternion_Normalize(float q[4]) {
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm == 0.0f) return;
    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;
}

void Quaternion_Product(float result[4], float q1[4], float q2[4]) {
    result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

void Quaternion_Conjugate(float result[4], float q[4]) {
    result[0] = q[0];
    result[1] = -q[1];
    result[2] = -q[2];
    result[3] = -q[3];
}

void Cross_Product(float result[3], float a[3], float b[3]) {
    result[0] = a[1]*b[2] - a[2]*b[1];
    result[1] = a[2]*b[0] - a[0]*b[2];
    result[2] = a[0]*b[1] - a[1]*b[0];
}
