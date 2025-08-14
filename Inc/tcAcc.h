#ifndef TC_ACC_H_
#define TC_ACC_H_

#include "arm_math.h"

void ComputeTiltCompensatedAcc(const float R[3][3], const float acc_body[3], float acc_earth[3]);

#endif /* TC_ACC_H_ */
