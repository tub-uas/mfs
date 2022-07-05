/**
 * @file algo_attitude.c
 * @author Christopher Ruwisch (christopher.ruwisch@gmail.com)
 * @brief Implementation of attitude estimation algorithm by Thomas Seel and Daniel Laidig 
 * @version 0.1
 * @date 2021-01-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "../include/util_quaternion64.h"
#include "../include/util_math64.h"
#include <math.h>


void attitude_gyroprediction64(double qatt[4], double gyr[3], double deltaT, double qpred[4]);
void attitude_x_correct64(double a[3], double b[3], double res[3]);

/**
 * [description]
 */
int algo_attitude_ts64(double qatt[4], double bgyr[3], double acc[3], double gyr[3], double mag[3],
					 double kacc, double kmag, double kbiasacc, double kbiasmag, double deltaT) {

	double rmag[4] = {0.0,1.0,0.0,0.0};
	double racc[4] = {0.0,0.0,0.0,-1.0};
	
	// NORMALIZATION OF ACC
	double tmp = math64_norm(acc,3);
	double accn[3] = {acc[0]/tmp, acc[1]/tmp, acc[2]/tmp};
	
	// NORMALIZATION OF MAG
	tmp = math64_norm(mag,3);
	double magn[3] = {mag[0]/tmp, mag[1]/tmp, mag[2]/tmp}; 

	// PREDICTION OF ATTITUDE BY GYRO
	double qOmega[3] = {gyr[0]+bgyr[0],gyr[1]+bgyr[1],gyr[2]+bgyr[2]};
	double qgyrpred[4] = {0};
	attitude_gyroprediction64(qatt,qOmega,deltaT,qgyrpred);
	quaternion_normalize64(qgyrpred,qgyrpred);

	// ROTATION OF MAG AND ACCEL VECTORS RMAG AND RACC
	double sracc[4] = {0,0,0,0};
	double srmag[4] = {0,0,0,0};
	quaternion_rotation64(qgyrpred,racc,sracc);
	quaternion_rotation64(qgyrpred,rmag,srmag);

	// ACCELEROMETER
	double alphaerr = math64_angle(accn,&sracc[1]);

	double xcorr[3] = {0,0,0};
	attitude_x_correct64(accn,&sracc[1],xcorr);

	double stmp = sin(0.5*kacc*alphaerr);
	double qcorracc[4] = {cos(0.5*kacc*alphaerr),
							stmp*xcorr[0],	// Adapted
							stmp*xcorr[1],
							stmp*xcorr[2]};

	quaternion_normalize64(qcorracc,qcorracc);
	double qcorrgyraccn[4] = {0};
	quaternion_prod64(qgyrpred,qcorracc,qcorrgyraccn);
	quaternion_normalize64(qcorrgyraccn,qcorrgyraccn);

	// MAGNETOMETER
	double magh[3] = {0,0,0};
	double skalarprod = magn[0]*sracc[1]+magn[1]*sracc[2]+magn[2]*sracc[3];
	
	magh[0] = magn[0] - (skalarprod*sracc[1]);
	magh[1] = magn[1] - (skalarprod*sracc[2]);
	magh[2] = magn[2] - (skalarprod*sracc[3]);

	double alphaerrmag = math64_angle(magh,&srmag[1]);

	double xcorrmag[3] = {0,0,0};
	attitude_x_correct64(magh,&srmag[1],xcorrmag);
	double stmpmag = sin(0.5*kmag*alphaerrmag);
	double qcorrmag[4] = {cos(0.5*kmag*alphaerrmag),
						stmpmag*xcorrmag[0],
						stmpmag*xcorrmag[1],
						stmpmag*xcorrmag[2]};

	quaternion_normalize64(qcorrmag,qcorrmag);
	double qcorrgyraccmagn[4] = {0};
	quaternion_prod64(qcorrgyraccn,qcorrmag,qcorrgyraccmagn);
	quaternion_normalize64(qcorrgyraccmagn,qcorrgyraccmagn);

	// GYRO BIAS ESTIMATION
	for(int i = 0; i < 3; ++i) {
		bgyr[i] = bgyr[i] + kbiasacc*alphaerr*xcorr[i] + kbiasmag*alphaerrmag*xcorrmag[i];
	}

	for(int i = 0; i < 4; ++i) {
		qatt[i] = qcorrgyraccmagn[i];
	}
	
	return 0;
}


void attitude_gyroprediction64(double qatt[4], double gyr[3], double deltaT, double qpred[4]) {
	
	double tmp[4] = {0,0,0,0};
	//quaternion_prod64(qatt,qgyr,tmp);
	qpred[0] = qatt[0] + 0.5*deltaT*(-gyr[0]*qatt[1] -gyr[1]*qatt[2] -gyr[2]*qatt[3]);
	qpred[1] = qatt[1] + 0.5*deltaT*( gyr[0]*qatt[0] + gyr[2]*qatt[2] - gyr[1]*qatt[3]);
	qpred[2] = qatt[2] + 0.5*deltaT*( gyr[1]*qatt[0] - gyr[2]*qatt[1] + gyr[0]*qatt[3]);
	qpred[3] = qatt[3] + 0.5*deltaT*( gyr[2]*qatt[0] + gyr[1]*qatt[1] - gyr[0]*qatt[2]);

}


void attitude_x_correct64(double a[3], double b[3], double res[3]) {
	double crossprod[3] = {0};
	
	math64_crossproduct(a,b,crossprod);
	
	double norm = sqrt(crossprod[0]*crossprod[0]+crossprod[1]*crossprod[1]+crossprod[2]*crossprod[2]);
	if(fabs(norm) < 1e-10) {
		res[0] = 0.0;
		res[1] = 0.0;
		res[2] = 0.0;
	}
	else {
		res[0] = crossprod[0]/norm;
		res[1] = crossprod[1]/norm;
		res[2] = crossprod[2]/norm;
	}

} 

