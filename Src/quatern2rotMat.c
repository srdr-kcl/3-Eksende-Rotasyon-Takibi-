#include "quatern2rotMat.h"
#include <math.h>

void quatern2rotMat(const float q[4], float R[3][3], int transpose) {
    // q[0] = w, q[1] = x, q[2] = y, q[3] = z


    R[0][0] = 2*q[0]*q[0] - 1 + 2*q[1]*q[1];
    R[0][1] = 2*(q[1]*q[2] + q[0]*q[3]);
    R[0][2] = 2*(q[1]*q[3] - q[0]*q[2]);

    R[1][0] = 2*(q[1]*q[2] - q[0]*q[3]);
    R[1][1] = 2*q[0]*q[0] - 1 + 2*q[2]*q[2];
    R[1][2] = 2*(q[2]*q[3] + q[0]*q[1]);

    R[2][0] = 2*(q[1]*q[3] + q[0]*q[2]);
    R[2][1] = 2*(q[2]*q[3] - q[0]*q[1]);
    R[2][2] = 2*q[0]*q[0] - 1 + 2*q[3]*q[3];

    if(transpose) {
        float temp;
        for(int i=0; i<3; i++) {
            for(int j=i+1; j<3; j++) {
                temp = R[i][j];
                R[i][j] = R[j][i];
                R[j][i] = temp;
            }
        }
    }
}
