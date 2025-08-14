#ifndef KALMANFILTER_H
#define KALMANFILTER_H

typedef struct {
    float x;
    float P;
    float Q;
    float R;
    float K;
} KalmanFilter;

void KalmanFilter_Init(KalmanFilter *kf, float q, float r, float initial_value);
float KalmanFilter_Update(KalmanFilter *kf, float measurement);

#endif
