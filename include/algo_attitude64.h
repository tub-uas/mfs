#ifndef ALGO_ATTITUDE64_H
#define ALGO_ATTITUDE64_H

#define ATTITUDE_PI64 3.141592653589793f
#define DEG2RAD     (ATTITUDE_PI64/180.0)
#define RAD2DEG     (180.0/ATTITUDE_PI64)

int algo_attitude_ts64(double qatt[4], double bgyr[3],
				double acc[3], double gyr[3], double mag[3],
				double kacc, double kmag, double kbiasacc,
				double kbiasmag, double deltaT);

#endif // ALGO_ATTITUDE_H