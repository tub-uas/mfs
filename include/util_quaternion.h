#ifndef UTIL_QUATERNION_H
#define UTIL_QUATERNION_H

#include <stdint.h>

void quaternion_prod(float p[4], float q[4], float res[4]);

void quaternion_inverse(float q[4], float res[4]);

float quaternion_norm(float p[4], float q[4]);

#endif // UTIL_QUATERNION_H
