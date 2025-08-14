#ifndef LIN_POS_H
#define LIN_POS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AXIS_DIM 3

// Başlatma fonksiyonu (konumu sıfırlar)
void LinPos_Init(void);

// Hız datasına göre konumu entegre eder
// linVel: [m/s] - anlık hız vektörü (3 elemanlı)
// linPosOut: [m] - entegre konum vektörü çıktısı (3 elemanlı)
// samplePeriod: [s] örnekleme periyodu
void LinPos_Update(const float linVel[AXIS_DIM], float linPosOut[AXIS_DIM], float samplePeriod);

// Gerekirse dışarıdan erişim için global değişken
extern float linPos[AXIS_DIM];

#ifdef __cplusplus
}
#endif

#endif // LIN_POS_H
