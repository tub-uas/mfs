#ifndef UTIL_QUATERNION64_H
#define UTIL_QUATERNION64_H

void quaternion_prod64(double p[4], double q[4], double pq[4]);
void quaternion_inverse64(double q[4], double qinv[4]);
double quaternion_normalize64(double q[4], double qn[4]);
void quaternion_rotation64(double q[4], double v[4], double qrot[4]);
void quaternion_euler_conv64(double q[4],double att[3]);

#endif // UTIL_QUATERNION32_H