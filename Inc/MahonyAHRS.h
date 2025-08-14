#ifndef MAHONY_AHRS_H
#define MAHONY_AHRS_H

typedef struct {
    float SamplePeriod;
    float Quaternion[4];  // [w, x, y, z]
    float Kp;
    float Ki;
    float eInt[3];        // integral error
} MahonyAHRS;

void Mahony_Init(MahonyAHRS* ahrs, float samplePeriod, float Kp, float Ki);
void Mahony_UpdateIMU(MahonyAHRS* ahrs, float gyro[3], float accel[3]);
void Mahony_Update(MahonyAHRS* ahrs, float gyro[3], float accel[3], float mag[3]);

// Yardımcı fonksiyonlar
void Quaternion_Normalize(float q[4]);
void Quaternion_Product(float result[4], float q1[4], float q2[4]);
void Quaternion_Conjugate(float result[4], float q[4]);
void Cross_Product(float result[3], float a[3], float b[3]);

#endif
