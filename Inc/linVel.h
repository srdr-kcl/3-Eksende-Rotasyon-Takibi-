#ifndef LIN_VEL_H
#define LIN_VEL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 3 eksenli vektör boyutu
#define AXIS_DIM 3

// Başlatma fonksiyonu (hızı sıfırlar)
void LinVel_Init(void);

// İvme datasına göre hızı entegre eder
// linAcc: [m/s^2]  - anlık ivme vektörü (3 elemanlı)
// linVelOut: [m/s] - entegre hız vektörü çıktısı (3 elemanlı)
// samplePeriod: [s] örnekleme periyodu
void LinVel_Update(const float linAcc[AXIS_DIM], float linVelOut[AXIS_DIM], float samplePeriod);

extern float linVel[AXIS_DIM];

#ifdef __cplusplus
}
#endif

#endif // LIN_VEL_H
