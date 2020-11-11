#include <util_quaternion.h>

#include <math.h>

#define P0 (p[0])
#define P1 (p[1])
#define P2 (p[2])
#define P3 (p[3])
#define Q0 (q[0])
#define Q1 (q[1])
#define Q2 (q[2])
#define Q3 (q[3])

void quaternion_prod(float p[4], float q[4], float res[4]) {

	res[0] = P0*Q0-P1*Q1-P2*Q2-P3*Q3;
	res[1] = P0*Q1+P1*Q0+P2*Q3-P3*Q2;
	res[2] = P0*Q2-P1*Q3+P2*Q0+P3*Q1;
	res[3] = P0*Q3+P1*Q2-P2*Q1+P3*Q0;
}

void quaternion_inverse(float q[4], float res[4]) {

	res[0] = Q0;
	res[1] = -Q1;
	res[2] = -Q2;
	res[3] = -Q3;
}

float quaternion_norm(float p[4], float q[4]) {

	float abs = sqrt(P0*P0 + P1*P1 + P2*P2 + P3*P3);

	Q0 = P0/abs;
	Q1 = P1/abs;
	Q2 = P2/abs;
	Q3 = P3/abs;

	return abs;
}
