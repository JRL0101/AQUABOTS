#pragma once
// specification file
// declaration files for the function are placed here

#ifndef GLOBALROUTINECALL_H
#define GLOBALROUTINECALL_H
#define _CRT_SECURE_NO_WARNINGS


// Function declarations

// Axis Transformation Functions

void LLA2ECEF(double LLA[3], double ECEF[3]); // Latitude, Longitude, Altitude to ECEF cartesian

void ECEF2LLA(double ECEF[3], double LLA[3]); // ECEF cartesian to Latitude, longitude, and altitude

// Quaternion Functions

void QInverse(double QVector[4], double QVectorInverse[4]);                                      // Form the conjugate of Quaternion Vector

void QNormalize(double QVector[4], double QNormVector[4]);                                       // Quaternion Normalization function

void QNormChk(double QVector[4], double Tolerance, int& QChkFlag);                                // Quaternion Normalization Check

void QProduct(double QVector1[4], double QVector2[4], double Q1Q2Vector[4]);                     // Performs Quaternion Product Operation

void Quaternion2Euler(double QVector[4], double& NewPhi, double& NewTheta, double& NewPsi);         // Form Euler Angles (Roll,Pitch,and Heading (phi-theta-psi) 
                                                                                                 // from Quaternion Vector)

void QuaternionAngularRate(double OmegaA, double OmegaB, double OmegaC, double QVector_Rate[4]); // Form Quaternion Propagation Solution from Angular Rates

// Transition Matrix Function

void trnmt(double Fmat_avg[15][15],double Filter_dt,double Udriv_avg[15][15],double phi[15][15], double q[15][15]); // Compute transition matrix, and input-noise-convolution matrix
                                                                                                                    // or integral of the transition matrix.

// ---------------------------------------------------------------------------------

// Total Angular Rate function

void TotalAngularRate(double Qvctr[4], double Qvctr1[4], double Qvctr2[4]);

// Matrix functions

void cLL2Body(double cLL_to_body[3][3], double sr, double cr, double cp, double sp, double ch, double sh);
void TransposeMatrix(double cbody_to_LL[3][3], double cLL_to_body[3][3]); // Only for 3x3 matrices
void findQVectorI(double QVectorI[4], double Vector_DC[3][3]);
void MatMultiply(double MatA[3][3], double MatB[3][3], double MatC[3][3]);

// NAV strapdown variables

extern double Velocity_LL[3];
extern double Position_LL[3];
extern double PosN_delta, PosE_delta, PosD_delta;
extern double USE_GPS;
//extern double Xsys1[15];
extern double FilterUpdate;
extern double vel_randWalk;
extern double angle_randWalk;

//extern double H[15][15];


#endif //GLOBALROUTINECALL_H
