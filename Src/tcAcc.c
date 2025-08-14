#include "tcAcc.h"
#include "arm_math.h"  // CMSIS-DSP

// CMSIS-DSP matris nesneleri
static arm_matrix_instance_f32 matR;
static arm_matrix_instance_f32 matAccBody;
static arm_matrix_instance_f32 matAccEarth;
static arm_matrix_instance_f32 matGravity;
static arm_matrix_instance_f32 matLinAcc;

static float R_data[9];
static float acc_body_data[3];
static float acc_earth_data[3];
static float gravity_data[3] = {0.0f, 0.0f, 1.0f};         // yerçekimi (g biriminde)
static float lin_acc_data[3];

void ComputeTiltCompensatedAcc(const float R[3][3], const float acc_body[3], float acc_earth[3]) {
    // 1. Matrisleri 1D diziye dönüştür
    for (int i = 0; i < 3; i++) {
        acc_body_data[i] = acc_body[i];
        for (int j = 0; j < 3; j++) {
            R_data[i * 3 + j] = R[i][j];  // 3x3 -> 1D dönüşümü satır bazlı
        }
    }

    // 2. CMSIS matris yapılarını başlat
    arm_mat_init_f32(&matR,         3, 3, R_data);
    arm_mat_init_f32(&matAccBody,   3, 1, acc_body_data);
    arm_mat_init_f32(&matAccEarth,  3, 1, acc_earth_data);
    arm_mat_init_f32(&matGravity,   3, 1, gravity_data);
    arm_mat_init_f32(&matLinAcc,    3, 1, lin_acc_data);

    // 3. Dönüşüm: Dünya eksenine göre ivme = R * ivme_beden
    arm_mat_mult_f32(&matR, &matAccBody, &matAccEarth);

    // 4. Yerçekimini çıkar (g biriminde)
    arm_mat_sub_f32(&matAccEarth, &matGravity, &matLinAcc);

    // 5. Sonucu m/s^2 birimine çevir (9.81 ile çarp)
    lin_acc_data[0] *= 9.81f;
    lin_acc_data[1] *= 9.81f;
    lin_acc_data[2] *= 0.1f;

    // 6. Sonucu kullanıcıya aktar
    acc_earth[0] = lin_acc_data[0];
    acc_earth[1] = lin_acc_data[1];
    acc_earth[2] = lin_acc_data[2];
}
