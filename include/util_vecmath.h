#ifndef UTIL_VECMATH_H
#define UTIL_VECMATH_H

#include <stdint.h>

void vecmath_matr_mul_3d(float mat[3][3], float vec[3], float res[3]);

float vecmath_norm_3d(float vec[3], float normalized[3]);

#endif // UTIL_VECMATH_H
