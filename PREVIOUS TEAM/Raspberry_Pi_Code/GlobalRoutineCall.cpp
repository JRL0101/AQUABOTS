// Implementation file
// function definitions are placed here

#include "GlobalRoutineCall.h"
#include <cmath>
#include <iostream>
#define PI 3.14159265358979323846
#define _CRT_SECURE_NO_WARNINGS

using namespace std;

// LLA2ECEF function - convert latitude,longitude, and altitude to earth-center, earth-fixed (ECEF) cartesian
// Input:
// LLA2ECEF(input[lat,lon,alt], output[x,y,z])
//
// x = ECEF X-coordinate (m)
// y = ECEF Y-coordinate (m)
// z = ECEF Z-coordinate (m)
// lat = geodetic latitude (radians)
// lon = longtiude (radians)
// alt = height above WGS84 ellipsoid (m)
//
// ASSUMPTION: The model is WGS84, Latitude is customary geodetic (not geocentric), input and output elements are scalars in a vector

void LLA2ECEF(double LLA[3], double ECEF[3]) {

	// WGS84 ellipsoid constants
	int a = 6378137;
	double e = 8.1819190842622e-2;

	// declaring each element of the input array for clarity

	double lat = LLA[0]; // latitude
	double lon = LLA[1]; // longitude
	double alt = LLA[2]; // altitude

	// intermediate calculation
	// (prime vertical radius of curvature)
	double N = a / sqrt(1 - pow(e,2) * pow(sin(lat),2));

	// return results

	ECEF[0] = (N + alt) * cos(lat) * cos(lon);        // X
	ECEF[1] = (N + alt) * cos(lat) * sin(lon);        // Y
	ECEF[2] = ((1 - pow(e, 2)) * N + alt) * sin(lat); // Z
}

// ECEF2LLA function - convert earth-center, earth-fixed (ECEF) cartesian to latitude,longitude, and altitude
// Input:
// ECEF2LLA(input[x,y,z],output[lat,lon,alt])
//
// lat = geodetic latitude (radians)
// lon = longtiude (radians)
// alt = height above WGS84 ellipsoid (m)
// x = ECEF X-coordinate (m)
// y = ECEF Y-coordinate (m)
// z = ECEF Z-coordinate (m)
//
// ASSUMPTION: The model is WGS84, Latitude is customary geodetic (not geocentric), not correcting for numerical instability
// in altitude near exact poles, input and output elements are scalars in a vector

void ECEF2LLA(double ECEF[3], double LLA[3]) {

	// WGS84 ellipsoid constants
	int a = 6378137;
	double e = 8.1819190842622e-2;

	// declaring each element of the input for clarity

	double x = ECEF[0];
	double y = ECEF[1];
	double z = ECEF[2];

	// Calculations

	double b = sqrt(pow(a, 2) * (1 - pow(e, 2)));
	double ep = sqrt((pow(a, 2) - pow(b, 2)) / pow(b, 2));
	double p = sqrt(pow(x, 2) + pow(y, 2));
	double th = atan2(a * z, b * p);
	double lon = atan2(y,x);
	double lat = atan2((z + pow(ep, 2) * b * pow(sin(th), 3)), (p - pow(e, 2) * a * pow(cos(th), 3)));
	double N = a / sqrt(1 - pow(e, 2) * pow(sin(lat), 2));
	double alt = p / cos(lat) - N;

	// return lon in range [0,2*PI]
	lon = fmod(lon, 2 * PI);

	//if (lon < 0) {
		//lon += 2 * PI;  // Ensure that lon is in the range [0, 2*pi)
	//}

	// declaring each element of the output array for return

	LLA[0] = lat; // latitude
	LLA[1] = lon; // longitude
	LLA[2] = alt; // altitude
}

// QInverse function - Form the conjugate of Quaternion Vector.
// 
// If Quaternion Vector is a Unit Quaternion then the Conjugate is the 
// Inverse Quarternion Vector such that QVector * Conjugate QVector equals
// the Identity Quaternion Vector.
// Input:  QVector[4]
// Output: QVectorInverse[4]

void QInverse(double QVector[4],double QVectorInverse[4]) {
	QVectorInverse[0] = QVector[0];
	QVectorInverse[1] = -QVector[1];
	QVectorInverse[2] = -QVector[2];
	QVectorInverse[3] = -QVector[3];
}

// QNormalize function - Quaternion Normalization function
//
// Input:  QVector[4]
// Output: QNormVecotr[4]

void QNormalize(double QVector[4], double QNormVector[4]) {
	double Qzero = QVector[0];
	double Qone = QVector[1];
	double Qtwo = QVector[2];
	double Qthree = QVector[3];

	double QNorm = sqrt(pow(Qzero, 2) + pow(Qone, 2) + pow(Qtwo, 2) + pow(Qthree, 2));
	if (QNorm > 0.0) {
		QNormVector[0] = Qzero / QNorm;
		QNormVector[1] = Qone / QNorm;
		QNormVector[2] = Qtwo / QNorm;
		QNormVector[3] = Qthree / QNorm;
	}
	else {
		QNormVector[0] = QVector[0];
		QNormVector[1] = QVector[1];
		QNormVector[2] = QVector[2];
		QNormVector[3] = QVector[3];
	}
}

// QNormChk function - Quaternion Normalization Check
//
// Input:  QVector[4], Tolerance
// Output: QChkFlag

void QNormChk(double QVector[4],double Tolerance,int &QChkFlag) {

	double epsilon = Tolerance;

	double Qzero = QVector[0];
	double Qone = QVector[1];
	double Qtwo = QVector[2];
	double Qthree = QVector[3];

	double QNorm = sqrt(pow(Qzero, 2) + pow(Qone, 2) + pow(Qtwo, 2) + pow(Qthree, 2));

	if (abs(QNorm) < epsilon) {
		QChkFlag = 0;
	}
	else {
		QChkFlag = 1;
	}
}

// QProduct function - Performs Quaternion Product Operation
//
// Input:  QVector1[4], QVector2[4]
// Output: Q1Q2Vector[4]

void QProduct(double QVector1[4],double QVector2[4],double Q1Q2Vector[4]) {

	double Qzero = QVector1[0] * QVector2[0] - QVector1[1] * QVector2[1] - QVector1[2] * QVector2[2] - QVector1[3] * QVector2[3];
	double Qone = QVector1[0] * QVector2[1] + QVector1[1] * QVector2[0] + QVector1[2] * QVector2[3] - QVector1[3] * QVector2[2];
	double Qtwo = QVector1[0] * QVector2[2] + QVector1[2] * QVector2[0] + QVector1[3] * QVector2[1] - QVector1[1] * QVector2[3];
	double Qthree = QVector1[0] * QVector2[3] + QVector1[3] * QVector2[0] + QVector1[1] * QVector2[2] - QVector1[2] * QVector2[1];

	Q1Q2Vector[0] = Qzero;
	Q1Q2Vector[1] = Qone;
	Q1Q2Vector[2] = Qtwo;
	Q1Q2Vector[3] = Qthree;
}

// Quaternion2Euler function - Form Euler Angles (Roll,Pitch,and Heading (phi-theta-psi) from Quaternion Vector)
//
// Input: QVector[4]
// Output: NewPhi, NewTheta, NewPsi (Roll,Pitch,Hdg)

void Quaternion2Euler(double QVector[4], double& NewPhi, double& NewTheta, double& NewPsi) {

	double Qzero = QVector[0];
	double Qone = QVector[1];
	double Qtwo = QVector[2];
	double Qthree = QVector[3];

	NewPsi = atan2(2 * (Qone * Qtwo - Qzero * Qthree), (2 * pow(Qzero, 2) + 2 * pow(Qone, 2) - 1));
	NewTheta = asin(-2 * (Qone * Qthree + Qzero * Qtwo));
	NewPhi = atan2(-2 * (Qzero * Qone - Qtwo * Qthree), (2 * pow(Qzero, 2) + 2 * pow(Qthree, 2) - 1));
}

// QuaternionAngularRate function - Form Quaternion Propagation Solution from Angular Rates
//
// Input:  OmegaA, OmegaB, OmegaC - gyro (body x-y-z) rates
// Output: QVector_Rate[4]

void QuaternionAngularRate(double OmegaA, double OmegaB, double OmegaC, double QVector_Rate[4]) {
	
	double OmegaMagnitude = sqrt(pow(OmegaA, 2) + pow(OmegaB, 2) + pow(OmegaC, 2));
	double Qzero_Rate = cos(OmegaMagnitude / 2.0);
	double AngleRatio;

	if (OmegaMagnitude > 0.00) {
		AngleRatio = sin(OmegaMagnitude / 2.0) / OmegaMagnitude;
	}
	else {
		AngleRatio = 0;
	}

	double Qone_Rate = AngleRatio * OmegaA;
	double Qtwo_Rate = AngleRatio * OmegaB;
	double Qthree_Rate = AngleRatio * OmegaC;

	QVector_Rate[0] = Qzero_Rate;
	QVector_Rate[1] = Qone_Rate;
	QVector_Rate[2] = Qtwo_Rate;
	QVector_Rate[3] = Qthree_Rate;

}

// trnmt Function - Compute transition matrix, and input-noise-convolution matrix or integral of the transition matrix.
// 
// Syntax: phi = trnmt(f,t)         to get just the transition matrix.
//      or
//        [phi,q] = trnmt(f,t)      for transition matrix and its integral.
//      or
//        [phi,q] = trnmt(f,t,w)    for transition and integral of noise cov.
// 
//       Fmat_avg  = f
//       Filter_dt = t
//       Udriv_avg = w
//
// Input: f = system description matrix (n-by-n)
//        t  = time interval
//        w  = input-noise covariance matrix (n-by-n)
// Output: phi = transition matrix for 0 to t time interval   (n-by-n)
//             = expm(f*t)
//         q   = Integral of the transition matrix            (n-by-n)
//             or
//         q   = Integral of input-noise covariance matrix    (n-by-n)

void trnmt(double Fmat_avg[15][15], double Filter_dt, double Udriv_avg[15][15], double phi[15][15],double q[15][15]) {

}

// TotalAngularRate function

// input: Qvctr1, Qvctr2
// output: Qvctr

void TotalAngularRate(double Qvctr[4], double Qvctr1[4], double Qvctr2[4]) {
	Qvctr[0] = Qvctr1[0] - Qvctr2[0];
	Qvctr[1] = Qvctr1[1] - Qvctr2[1];
	Qvctr[2] = Qvctr1[2] - Qvctr2[2];
	Qvctr[3] = Qvctr1[3] - Qvctr2[3];
}

// -----------------------------------------------------------------------------

// Matrix functions

void cLL2Body(double cLL_to_body[3][3], double sr, double cr, double cp, double sp, double ch, double sh) {
	cLL_to_body[0][0] = cp * ch;
	cLL_to_body[0][1] = cp * sh;
	cLL_to_body[0][2] = -sp;

	cLL_to_body[1][0] = -cr * sh + sr * sp * ch;
	cLL_to_body[1][1] = cr * ch + sr * sp * sh;
	cLL_to_body[1][2] = sr * cp;

	cLL_to_body[2][0] = sr * sh + cr * sp * ch;
	cLL_to_body[2][1] = -sr * ch + cr * sp * sh;
	cLL_to_body[2][2] = cr * cp;
}

void TransposeMatrix(double cbody_to_LL[3][3], double cLL_to_body[3][3]) {
	cbody_to_LL[0][0] = cLL_to_body[0][0];
	cbody_to_LL[0][1] = cLL_to_body[1][0];
	cbody_to_LL[0][2] = cLL_to_body[2][0];

	cbody_to_LL[1][0] = cLL_to_body[0][1];
	cbody_to_LL[1][1] = cLL_to_body[1][1];
	cbody_to_LL[1][2] = cLL_to_body[2][1];

	cbody_to_LL[2][0] = cLL_to_body[0][2];
	cbody_to_LL[2][1] = cLL_to_body[1][2];
	cbody_to_LL[2][2] = cLL_to_body[2][2];
}

void findQVectorI(double QVectorI[4], double Vector_DC[3][3]) {
	QVectorI[0] = 0.5 * sqrt(Vector_DC[0][0] + Vector_DC[1][1] + Vector_DC[2][2] + 1);
	QVectorI[1] = (Vector_DC[1][2] - Vector_DC[2][1]) / (4 * QVectorI[0]);
	QVectorI[2] = (Vector_DC[2][0] - Vector_DC[0][2]) / (4 * QVectorI[0]);
	QVectorI[3] = (Vector_DC[0][1] - Vector_DC[1][0]) / (4 * QVectorI[0]);
}

void MatMultiply(double MatA[3][3], double MatB[3][3], double MatC[3][3])
// Function:  MatMultiply(A, B, C, l, m, n)
// Matrix multiply:  C = A * B,
// where A is 3x3, B is 3x3, and C is 3x3.

{
	int i, j, k;
	int l = 3; int m = 3; int n = 3;
	double sum;

	for (i = 0; i < l; i++) {
		for (j = 0; j < n; j++) {
			sum = 0.0;
			for (k = 0; k < m; k++) sum += MatA[i][k] * MatB[k][j];
			MatC[i][j] = sum;
		}
	}
}


