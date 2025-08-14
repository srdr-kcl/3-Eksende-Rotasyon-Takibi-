#include "KalmanFilter.h"

void KalmanFilter_Init(KalmanFilter *kf, float q, float r, float initial_value) {
    kf->Q = q;
    kf->R = r;
    kf->x = initial_value;
    kf->P = 1.0f;
    kf->K = 0.0f;
}

float KalmanFilter_Update(KalmanFilter *kf, float measurement) {
    // Predict
    kf->P = kf->P + kf->Q;

    // Update
    kf->K = kf->P / (kf->P + kf->R);
    kf->x = kf->x + kf->K * (measurement - kf->x);
    kf->P = (1 - kf->K) * kf->P;

    return kf->x;
}
