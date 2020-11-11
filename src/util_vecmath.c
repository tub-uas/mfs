#include "util_vecmath.h"

#include <math.h>

#define M11 (mat[0][0])
#define M12 (mat[0][1])
#define M13 (mat[0][2])
#define M21 (mat[1][0])
#define M22 (mat[1][1])
#define M23 (mat[1][2])
#define M31 (mat[2][0])
#define M32 (mat[2][1])
#define M33 (mat[2][2])
#define V1  (vec[0])
#define V2  (vec[1])
#define V3  (vec[2])

void vecmath_matr_mul_3d(float mat[3][3], float vec[3], float res[3]) {
	res[0] = M11*V1+M12*V2+M13*V3;
	res[1] = M21*V1+M22*V2+M23*V3;
	res[2] = M31*V1+M32*V2+M33*V3;
}

float vecmath_norm_3d(float vec[3], float normalized[3]) {

	float abs = sqrt(V1*V1 + V2*V2 + V3*V3);

	normalized[0] = V1/abs;
	normalized[1] = V2/abs;
	normalized[2] = V3/abs;

	return abs;
}
