#ifndef QUATERN2ROTMAT_H
#define QUATERN2ROTMAT_H

/**
 * @brief Quaternion'dan rotation matrise dönüşüm
 * @param q Giriş kuaterniyonu [w, x, y, z]
 * @param R Çıkış 3x3 dönüş matrisi (row-major)
 * @param transpose Matrisin transpozunu almak için 1, normal için 0
 */
void quatern2rotMat(const float q[4], float R[3][3], int transpose);

#endif // QUATERN2ROTMAT_H
