#include "linPos.h"
#include <string.h>

// Statik konum vektörü
float linPos[AXIS_DIM] = {0.0f, 0.0f, 0.0f};

// Başlatma: konumu sıfırla
void LinPos_Init(void) {
    memset(linPos, 0, sizeof(linPos));
}

// Güncelleme: hızı entegre ederek konum hesapla
void LinPos_Update(const float linVel[AXIS_DIM], float linPosOut[AXIS_DIM], float samplePeriod) {
    for (uint8_t i = 0; i < AXIS_DIM; ++i) {
        linPos[i] += linVel[i] * samplePeriod;
        linPosOut[i] = linPos[i]; //mm
    }
}
