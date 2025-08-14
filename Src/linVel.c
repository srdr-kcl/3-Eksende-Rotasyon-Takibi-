#include "linVel.h"
#include <string.h>  // for memset

// hız vektörü (önceki değer)
float linVel[AXIS_DIM] = {0.0f, 0.0f, 0.0f};

// Başlatma: hız vektörünü sıfırla
void LinVel_Init(void) {
    memset(linVel, 0, sizeof(linVel));
}

// Güncelleme: ivmeyi entegre ederek hız hesapla
void LinVel_Update(const float linAcc[AXIS_DIM], float linVelOut[AXIS_DIM], float samplePeriod) {
    for (uint8_t i = 0; i < AXIS_DIM; ++i) {
        linVel[i] += linAcc[i] * samplePeriod;  // Basit entegrasyon (Euler yöntemi)
        linVelOut[i] = linVel[i];
    }
}
