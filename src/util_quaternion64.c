#include "../include/util_quaternion64.h"
#include "../include/util_math64.h"
#include <math.h>


#define P0 (p[0])
#define P1 (p[1])
#define P2 (p[2])
#define P3 (p[3])
#define Q0 (q[0])
#define Q1 (q[1])
#define Q2 (q[2])
#define Q3 (q[3])


void quaternion_prod64(double p[4], double q[4], double pq[4]) {

	pq[0] = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3];
	pq[1] = p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2];
	pq[2] = p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + q[3]*q[1];
	pq[3] = p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0];
}

void quaternion_inverse64(double q[4], double qinv[4]) {
	
	qinv[0] = q[0];
	qinv[1] = -1.0*q[1];
	qinv[2] = -1.0*q[2];
	qinv[3] = -1.0*q[3];

}

double quaternion_normalize64(double q[4], double qn[4]) {

	double norm = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
	qn[0] = q[0]/norm;
	qn[1] = q[1]/norm;
	qn[2] = q[2]/norm;
	qn[3] = q[3]/norm;

	
	return norm;
}

void quaternion_rotation64(double q[4], double v[4], double qrot[4]) {

	double qsqr[4] = {q[0]*q[0],q[1]*q[1],q[2]*q[2],q[3]*q[3]};
	
	qrot[0] = 0.0;
	qrot[1] = 2*v[1]*(0.5-qsqr[2]-qsqr[3]) + 2*v[2]*(q[0]*q[3]+q[1]*q[2]) + 2*v[3]*(q[1]*q[3]-q[0]*q[2]);
	qrot[2] = 2*v[1]*(q[1]*q[2]-q[0]*q[3]) + 2*v[2]*(0.5-qsqr[1]-qsqr[3]) + 2*v[3]*(q[0]*q[1]+q[2]*q[3]);
	qrot[3] = 2*v[1]*(q[0]*q[2]+q[1]*q[3]) + 2*v[2]*(q[2]*q[3]-q[0]*q[1]) + 2*v[3]*(0.5-qsqr[1]-qsqr[2]);
	
}

void quaternion_euler_conv64(double q[4],double att[3]) {
	
	double asinInput = 2*(q[0]*q[2]-q[3]*q[1]);
	if(asinInput > 1.0f)
		asinInput = 1.0;
	if(asinInput < -1.0f)
		asinInput = -1.0f;


	att[0] = atan2(2*(q[2]*q[3]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]));
	att[1] = asin(asinInput);
	att[2] = atan2(2*(q[1]*q[2]+q[0]*q[3]),1-2*(q[2]*q[2]+q[3]*q[3]));
}