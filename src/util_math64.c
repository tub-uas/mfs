/**
 * @file util_math.c
 * @author Christopher Ruwisch (christopher.ruwisch@gmail.com)
 * @brief Mathematical utility library
 * @version 0.1
 * @date 2021-01-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <math.h>


double math64_norm(double a[], int len) {
	double sum = 0;
	for(int i = 0; i < len; ++i) {
		sum += a[i]*a[i];
	}
	return sqrt(sum);
} 

void math64_crossproduct(double a[3], double b[3], double res[3]) {

	res[0] = a[1]*b[2] - a[2]*b[1];
	res[1] = a[2]*b[0] - a[0]*b[2];
	res[2] = a[0]*b[1] - a[1]*b[0];
}

double math64_angle(double a[3], double b[3]) {

	double n = sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])*sqrt(b[0]*b[0]+b[1]*b[1]+b[2]*b[2]);
	if(fabs(n) < 1e-10) {
		return 0.0;
	}
	else {
		double tmp = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
		return acos(tmp/n);
	}
}

