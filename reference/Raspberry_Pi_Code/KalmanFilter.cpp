/// KalmanFilter.cpp :

//					This routine will receive an array of data and provide the best estimates
//					of NTR error data back.  The whole value data position estimates for the
//					LHS position and uncertainty along with NTR ocean current data is also output.
//

#include "stdafx.h"
#include "KalmanFilter.h"
#include <memory.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <crtdbg.h>
#include "Utility2.h"
#include "KFTYPE.H"
#include "GlobalRoutineCall.h"
#include <iostream>
#define _CRT_SECURE_NO_WARNINGS
#pragma warning(disable : 4996)




/// Defining Variables for the Kalman Filter Application.
//double static pi = 3.1415926535897932384626433832795;
//double static nmile = 6076.115486;			// conversion to feet
//double static kn_ft = nmile / 3600.0;			// conversion to ft/sec from kts
//double static ae = 2.092564632545932e+007;	// Radius of earth in feet
//double static e2 = 6.6943799014e-003;		// Ellipsoidal model of eccentricity (WGS-84)

double ORE_mount[3]; // The array that holds the LHS Track Point Mounting Angles
double last_time = 0.0;

bool Startup = true;
bool Set_Depth_Range;

short mycase = 0;

//NStates = 15;
//max = 15;
//NMeas = 7;

double** _X, ** _F, ** _W, ** _I, ** _P, ** _Phi, ** _Q,
** _Temp1, ** _Temp2, ** _Temp3, ** _Temp4, ** _Temp5,
** _H, ** _R, ** _Z, ** _mq, ** _mcq, ** _KalmG;

double X[15][2], F[15][15], W[15][15], I[15][15],
P[15][15], Phi[15][15], Q[15][15],
Temp1[15][15], Temp2[15][15], Temp3[15][15],
Temp4[15][15], Temp5[15][15],
H[6][15], R[6][6], Z[6][6],
mq[15][6], mcq[6][6], KalmG[15][6];


double* Y, * temp_vect, * temp_vect2, * temp_vect3, * Z_vect, * Resid, * Ratio;

int* temp_ivect;//, temp_ivect[15];
//short *_mycase;

// ///////////////////////////////////////////////////////////////////////////



// MATLAB variables
double PosN_delta, PosE_delta, PosD_delta;
double USE_GPS = 0;
double Velocity_LL[3];
double Position_LL[3];
//double Xsys1[15];
double vel_randWalk;
double angle_randWalk;
double FilterUpdate;
KalmanInputType KInput;
KalmanOutputType KFoutput;



#pragma once

/*class KalmanFilter
{
public:
	KalmanFilter( KalmanInputType, KalmanOutputType );

	~KalmanFilter( KalmanInputType, KalmanOutputType );


};*/

//void casemeasure (KalmanInputType, double **, short *);
void free_arrays_proto();

void KalmanFilter(KalmanInputType KalmanInput, KalmanOutputType* _KFoutput, int FILTER_RUN)
{
	///	Local Variables for the filter
	//KalmanInputType KInput;
	//KalmanOutputType KFoutput;
	//unsigned char inbuf2[sizeof(KFoutput)];


	FILE* fp1;
	int i, j, k, n, rows, icheck, number_rows, jj, kk;

	double  crv, srv, cpv, spv, chv, shv, tau, North_offset_lat, East_offset_lon,
		R_merid, R_norm,
		Delta_lat, Delta_lon, Delta_N, Delta_E, Delta_D, Depth_LHS, Val, gooey;

	double  cr, sr, cp, sp, ch, sh, step, current_time,
		cphi, sphi, ctheta, stheta, cpsi, spsi,
		SRange, dt, nddt;

	double  Axx, Axy, Axz, Ayx, Ayy, Ayz, Azx, Azy, Azz, del_Axx[6], del_Axy[6],
		del_Ayx[6], del_Ayy[6], del_Azz[6], R_north, R_east, R_down, caz, saz,
		Filt_meas1, Filt_meas2, Filt_meas3;

	double  ratio_check, dummy;

	bool 	compute_filter = true;

	// Define variables to not get errors

	//i = 0; j = 0; k = 0; kk = 0; number_rows = 0; jj = 0;
	number_rows = 0; // DELETING THIS GIVES AN ERROR

	// MATLAB Local Variables

	int bad;
	double bad_fix_at_time;
	//PosN_delta = 0; PosE_delta = 0; PosD_delta = 0;
	//double FilterUpdate;

	int time_to_start_GPS = 69576; // GPS sigma reduced to 8 ft
	int time_bike_moved = 69483; // Zero velocity updates stopped

	// Errors for the 3020 IMU -----------------------------------------------------

	//double vel_randWalk = 0.2 / 60; // .078 m / s / rt - hr
	double accellSF = 0.003;
	double bias_accel = 6 * 9.8e-3; // six milli - g's
	double Accel_misalign = 5e-4;

	//double angle_randWalk = .22 * pi / (180. * 60); // siggn
	double bias_gyro = 20.5 * pi / (180. * 3600);
	double gyro_SF = 0.0005;
	double Gyro_misalign = 5e-4;
	// ----------------------------------------------------------------------------

	// buffer to hold incoming data structure message for compensation from filter as needed
	unsigned char inbuf[sizeof(KInput)];


	/// Starting the filter routine! *****************
		// getting the data structure loaded
	/*    memmove(inbuf2,_KFoutput,sizeof(KFoutput));
		memmove(&KFoutput,inbuf2,sizeof(KFoutput));
	*/
	memmove(inbuf, &KalmanInput, sizeof(KInput));
	memmove(&KInput, inbuf, sizeof(KInput));

	/// ALLOCATE MEMORY FOR ALL ARRAYS ... (ONCE)
	// .....................................................................
	/// The memory allocation for all arrays is done here - one time 
	/// upon startup of the parent routine. If reinitialization is 
	/// commanded, the arrays will simply be zeroed and/or placed in 
	/// the default startup state.
	/// The .INI file is also read in here upon startup of the filter.

	if (Startup)
	{
		/*
		if ((fp1=fopen("kalman_ore_data.txt","r"))==NULL)	// Changed to accept this new vector
		{
			printf( "I can't open the mounting angle data file! %s\n", strerror( errno ) );
			exit(1);
		}  //if..

		for (rows=0;rows<3;rows++)   // Reading the mounting angles for the ORE track point system
		{					  // on the LHS currently being used

			if ((icheck=feof(fp1))==0)      // Checking for end of file
			{
				fscanf(fp1," %lf\n",&gooey);
				ORE_mount[rows]=gooey;
			}
		}
		fclose(fp1);
		*/

		_X = convert_dmatrix(&X[0][0], 0, (NStates - 1), 0, 1);
		_F = convert_dmatrix(&F[0][0], 0, (NStates - 1), 0, (NStates - 1));
		_W = convert_dmatrix(&W[0][0], 0, (NStates - 1), 0, (NStates - 1));
		_I = convert_dmatrix(&I[0][0], 0, (NStates - 1), 0, (NStates - 1));
		_P = convert_dmatrix(&P[0][0], 0, (NStates - 1), 0, (NStates - 1));
		_Phi = convert_dmatrix(&Phi[0][0], 0, (NStates - 1), 0, (NStates - 1));
		_Q = convert_dmatrix(&Q[0][0], 0, (NStates - 1), 0, (NStates - 1));
		_Temp1 = convert_dmatrix(&Temp1[0][0], 0, (NStates - 1), 0, (NStates - 1));
		_Temp2 = convert_dmatrix(&Temp2[0][0], 0, (NStates - 1), 0, (NStates - 1));
		_Temp3 = convert_dmatrix(&Temp3[0][0], 0, (NStates - 1), 0, (NStates - 1));
		_Temp4 = convert_dmatrix(&Temp3[0][0], 0, (NStates - 1), 0, (NStates - 1));
		_Temp5 = convert_dmatrix(&Temp3[0][0], 0, (NStates - 1), 0, (NStates - 1));
		_H = convert_dmatrix(&H[0][0], 0, (NMeas - 1), 0, (NStates - 1));
		_R = convert_dmatrix(&R[0][0], 0, (NMeas - 1), 0, (NMeas - 1));
		_Z = convert_dmatrix(&Z[0][0], 0, (NMeas - 1), 0, (NMeas - 1));
		_mq = convert_dmatrix(&mq[0][0], 0, (NStates - 1), 0, (NMeas - 1));
		_mcq = convert_dmatrix(&mcq[0][0], 0, (NMeas - 1), 0, (NMeas - 1));
		_KalmG = convert_dmatrix(&KalmG[0][0], 0, (NStates - 1), 0, (NMeas - 1));

		//		_mycase = (short *)malloc( sizeof(short) );
		//		_mycase = &mycase;

		Y = dvector(0, (NMeas - 1));
		temp_vect = dvector(0, (NStates - 1));
		temp_vect2 = dvector(0, (NStates - 1));
		temp_vect3 = dvector(0, (NStates - 1));
		Z_vect = dvector(0, (NMeas - 1));
		Resid = dvector(0, (NMeas - 1));
		Ratio = dvector(0, (NMeas - 1));

		temp_ivect = ivector(0, (NStates - 1));

		// Function call to the LHS NAV Class to initialize it .....

	}  // end if startup...

	if ((KalmanInput.mode == 2) || (Startup)) //SET ALL ELEMENTS TO ZERO AND INITIALIZE PROPERLY
	{

		dmzero(_X, NStates, 2);
		dmzero(_F, NStates, NStates);
		dmzero(_W, NStates, NStates);
		dmzero(_I, NStates, NStates);
		dmzero(_P, NStates, NStates);
		dmzero(_Phi, NStates, NStates);
		dmzero(_Q, NStates, NStates);
		dmzero(_Temp1, NStates, NStates);
		dmzero(_Temp2, NStates, NStates);
		dmzero(_Temp3, NStates, NStates);
		dmzero(_H, NMeas, NStates);
		dmzero(_R, NMeas, NMeas);
		dmzero(_Z, NMeas, NMeas);
		dmzero(_mq, NStates, NMeas);
		dmzero(_mcq, NMeas, NMeas);
		dmzero(_KalmG, NStates, NMeas);

		vezero(Y, NMeas);
		vezero(temp_vect, NStates);
		vezero(temp_vect2, NStates);
		vezero(temp_vect3, NStates);
		vezero(Z_vect, NMeas);
		vezero(Resid, NMeas);
		vezero(Ratio, NMeas);

		iezero(temp_ivect, NStates);
		// Now Initialize...................

		/// THESE LINES INITIALIZE THE DESCRIPTION MATRIX F
		for (int i = 0; i < 15; ++i) {			// Initializes every element to zero in Fmat
			for (int j = 0; j < 15; ++j) {
				F[i][j] = 0.0;
			}
		}

		/// THESE LINES INITIALIZE THE DRIVING NOISE MATRIX W
		for (int i = 0; i < 15; ++i) {			// Initializes every element to zero in Wmat
			for (int j = 0; j < 15; ++j) {
				W[i][j] = 0.0;
			}
		}

		W[3][3] = pow(vel_randWalk, 2);
		W[4][4] = W[3][3];
		W[5][5] = W[3][3];
		W[6][6] = pow(angle_randWalk, 2);
		W[7][7] = W[6][6];
		W[8][8] = W[6][6];

		/// THESE LINES INITIALIZE THE COVARIANCE MATRIX P
		P[0][0] = pow(5, 2);				// Initial Covariance Matrix  pos error is 5 meters (N & E)
		P[1][1] = P[0][0];
		P[2][2] = pow(2.0, 2);			// initial depth error is 1 meter
		P[3][3] = pow(3.0, 2);			// initial vel error is 3 m/sec (N & E)
		P[4][4] = P[3][3];
		P[5][5] = pow(1.5, 2);				// Vert Velocity ~ 1.5 m/s
		P[6][6] = pow((5 * pi / 180), 2);		// initial angle errors are 1 deg for roll & pitch
		P[7][7] = P[6][6];
		P[8][8] = pow((10 * pi / 180), 2);	// Heading error set to 5 degrees
		P[9][9] = pow(bias_accel, 2);        // MEMS accel bias
		P[10][10] = P[9][9];
		P[11][11] = P[9][9];
		P[12][12] = pow(bias_gyro, 2);     // MEMS gyro bias
		P[13][13] = P[12][12];
		P[14][14] = P[12][12];

	}  // end if ( (KalmanInput.mode == 2) || 

	if (Startup)
	{
		Startup = false;	// Signifying that the memory allocation is only done ONCE!

		return;				// Jumping out of the filter after initializing the memory and arrays
	}

	if (KalmanInput.mode == 3) //Free all memory allocated as program is terminating
	{
		// Free-up the arrays on the heap:
		free_arrays_proto();
		return;
	}

	/// Running Mode.... 

	if (KalmanInput.mode == 0) // Running but not launched and Kalman filter is not engaged
	{
		last_time = KalmanInput.time;
		KFoutput.time = last_time;
		KFoutput.filter_mode = KalmanInput.mode;

		// MATLAB KFoutput

		KFoutput.delta_Pn = 0;
		KFoutput.delta_Pn = 0;
		KFoutput.delta_Pe = 0;
		KFoutput.delta_Pd = 0;
		KFoutput.delta_Vn = 0;
		KFoutput.delta_Ve = 0;
		KFoutput.delta_Vd = 0;
		KFoutput.delta_tiltN = 0;
		KFoutput.delta_tiltE = 0;
		KFoutput.delta_tiltD = 0;
		KFoutput.delta_ABx = 0;
		KFoutput.delta_ABy = 0;
		KFoutput.delta_ABz = 0;
		KFoutput.delta_GBx = 0;
		KFoutput.delta_GBy = 0;
		KFoutput.delta_GBz = 0;
		KFoutput.covPn = 0;
		KFoutput.covPe = 0;
		KFoutput.covVn = 0;
		KFoutput.covVe = 0;
		KFoutput.covPd = 0;
		KFoutput.covPsiD = 0;
	}

	//---------------------------------------------------------------------
	/// *** START KALMAN FILTER CODE HERE ***
	if (KalmanInput.mode == 1)		// We got the Launch signal
	{
		//---------------------------------------------------------------------

		// PART I - COMPENSATE DATA & UPDATE DESCRIPTION MATRIX (F) & DRIVING NOISE MATRIX (W)

		//----------------------------------------------------------------------------------
		current_time = KalmanInput.time;


		// PART II - CALCULATE TRANSITION MATRIX AND PROCESS NOISE COVARIANCE

		if (last_time > 0.0)
			dt = current_time - last_time;
		else
			dt = 1.0;
		last_time = current_time;

		// Create identity matrix I

		dmeye(_I, max, max);

		// Solve for Phi and Q

		nddt = 4;
		step = dt / 16.0;
		dmscal(_W, _Q, max, max, step);
		// Function:  dmscal(A, B, m, n, c)
		// Multiply matrix A by scalar double c:  B = A * c
		// where arrays A and B are m-by-n.

		dmscal(_F, _Temp1, max, max, step);
		// Function:  dmscal(A, B, m, n, c)
		// Multiply matrix A by scalar double c:  B = A * c
		// where arrays A and B are m-by-n.

		dmapb(_I, _Temp1, _Phi, max, max);  // Phi = _I + _F * step
		// Function:  dmapb(A, b, C, m, n)
		// Matrix add (plus):  C= A + B,
		// where arrays A, b, and C are m-by-n.

		for (i = 1; i <= nddt; i++)
		{
			dmab(_Phi, _Q, _Temp1, max, max, max);  // _Temp1 = _Phi * _Q
		// Function:  dmab(A, B, C, l, m, n)
		// Matrix multiply:  C = A * B,
		// where A is l-by-m, B is m-by-n, and C is l-by-n.

			dmabt(_Temp1, _Phi, _Temp2, max, max, max);  // _Temp2 = _Phi*_Q * _Phi^T
		// Function:  dmabt(A, B, C, l, m, n)
		// Matrix multiply:  C = A * B^T,  (B^T signifies b transposed)
		// where A is l-by-m, B^T is m-by-n (or B is n-by-m), and C is l-by-n.

			dmcopy(_Q, _Temp1, max, max);  // _Temp1 = _Q
		// Function:  dmcopy(A, B, m, n)
		// Copy m-by-n matrix A into m-by-n matrix B.

			dmapb(_Temp1, _Temp2, _Q, max, max);  // _Q = _Q + _Phi*_Q*_Phi^T
		// Function:  dmapb(A, b, C, m, n)
		// Matrix add (plus):  C= A + B,
		// where arrays A, b, and C are m-by-n.

			dmcopy(_Phi, _Temp1, max, max);  // _Temp1 = _Phi
		// Function:  dmcopy(A, B, m, n)
		// Copy m-by-n matrix A into m-by-n matrix B.

			dmab(_Phi, _Temp1, _Temp2, max, max, max);  // _Temp2 = _Phi * _Phi
		// Function:  dmab(A, B, C, l, m, n)
		// Matrix multiply:  C = A * B,
		// where A is l-by-m, B is m-by-n, and C is l-by-n.

			dmcopy(_Temp2, _Phi, max, max);  // _Phi = _Phi * _Phi
		// Function:  dmcopy(A, B, m, n)
		// Copy m-by-n matrix A into m-by-n matrix B.

		} // for (i=1...

		/// PART III - PROPAGATE STATE VECTOR AND COVARIANCE MATRICES

		// Propagate state matrix

		dmab(_Phi, _X, _Temp1, max, max, 2);  // _Temp1 = _Phi * _X
		// Function:  dmab(A, B, C, l, m, n)
		// Matrix multiply:  C = A * B,
		// where A is l-by-m, B is m-by-n, and C is l-by-n.

		dmcopy(_Temp1, _X, max, 2);   // _X = _Phi * _X
		// Function:  dmcopy(A, B, m, n)
		// Copy m-by-n matrix A into m-by-n matrix B.

		/// *** Propagate covariance matrix

		dmab(_Phi, _P, _Temp1, max, max, max);  // _Temp1 = _Phi * _P
		// Function:  dmab(A, B, C, l, m, n)
		// Matrix multiply:  C = A * B,
		// where A is l-by-m, B is m-by-n, and C is l-by-n.

		dmabt(_Temp1, _Phi, _Temp2, max, max, max);  // _Temp2 = _Phi * _P * _Phi^T
		// Function:  dmabt(A, B, C, l, m, n)
		// Matrix multiply:  C = A * B^T,  (B^T signifies b transposed)
		// where A is l-by-m, B^T is m-by-n (or B is n-by-m), and C is l-by-n.

		dmapb(_Temp2, _Q, _P, max, max);  // _P = _Phi*_P*_Phi^T + _Q
		// Function:  dmapb(A, b, C, m, n)
		// Matrix add (plus):  C= A + B,
		// where arrays A, b, and C are m-by-n.


		/// PART IV -- FORM THE MEASUREMENT EQUATIONS:

		//casemeasure(KalmanInput,_R,&mycase);
		number_rows = 0;
		dmzero(_H, NMeas, NStates);		// Zeros out the H matrix
		vezero(Y, NMeas);


		// REWRITE PER MATLAB ------------------------------------------------------------------------------------------------------
		//-------------------------------------------------------------------------------------------------------------------------
		//If filter run is true do the belo (all of it) to generate a message
		if (FILTER_RUN > 0.5) {
			if (USE_GPS < 0.5) {
				H[5][14] = 0;
				R[5][5] = 0;
				Y[5] = 0;
				number_rows = 6;

				Y[3] = Velocity_LL[0]; // Bringing in zero velocity data
				Y[4] = Velocity_LL[1];
				Y[5] = Velocity_LL[2];

				H[3][3] = 1;
				H[4][4] = 1;
				H[5][5] = 1;

				R[0][0] = pow(.7, 2); // Small stationary measurement covariance error
				R[1][1] = pow(.7, 2);
				R[2][2] = pow(.7, 2);
				R[3][3] = pow(0.05, 2);
				R[4][4] = pow(0.05, 2);
				R[5][5] = pow(0.05, 2);

			}
			else if (USE_GPS > 0.5) {
				H[2][14] = 0;
				R[2][2] = 0;
				Y[2] = 0;
				number_rows = 3;
			}
			Y[0] = PosN_delta;   // default position data is used
			Y[1] = PosE_delta;
			Y[2] = PosD_delta;

			H[0][0] = 1;
			H[1][1] = 1;
			H[2][2] = 1;
			if (USE_GPS > 0.5) {
				R[0][0] = pow(3., 2);
				R[1][1] = pow(3., 2);
				R[2][2] = pow(5., 2);
			}

			//}
			//}  //if (mycase > 1)
				// Kalman Gain Calculation
		/// INNOVATION CHECK $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
			j = number_rows;
			jj = max;
			kk = max;
			dmab(_H, _P, _Temp1, j, jj, kk);  // _Temp1=_H*_P
			// where _H is j-by-jj, _P is jj-by-kk, and _Temp1 is j-by-kk.

			j = number_rows;
			jj = max;
			kk = number_rows;
			dmabt(_Temp1, _H, _Temp2, j, jj, kk); //_Temp2 = _Temp1 * _H^T,  (^T signifies transpose) 
			// where _Temp1 is j-by-jj, _H^T is jj-by-kk (or _H is kk-by-jj), and _Temp2 is j-by-kk.
			// Function:  dmabt(A, B, C, l, m, n)
			// Matrix multiply:  C = A * B^T,  (B^T signifies b transposed)
			// where A is l-by-m, B^T is m-by-n (or B is n-by-m), and C is l-by-n.

			j = number_rows;
			dmapb(_Temp2, _R, _Z, j, j);
			// Function:  dmapb(A, b, C, m, n)
			// Matrix add (plus):  C= A + B,
			// where arrays A, b, and C are m-by-n.

			diagonal(_Z, Z_vect, j);
			// Function:  diagonal(A, B, m)
			// Place diagonal of square matrix A into vector B
			// where array A is m-by-m and array B is m-by-1

			for (i = 0; i < j; i++) {
				Z_vect[i] = sqrt(Z_vect[i]);
			}

			for (i = 0; i < max; i++) {
				temp_vect[i] = X[i][0];
			}

			j = number_rows;
			dmav(_H, temp_vect, temp_vect2, j, max);
			// Function:  dmav(A, v, u, m, n)
			// Matrix-vector multiply:  u = A * v,
			// where A is m-by-n matrix, v is n-by-1 vector, and u is m-by-1 vector.

			j = number_rows;
			vecsub(Y, temp_vect2, Resid, j);
			// Function:  vecsub(A, B, C, m)
			// Vector minus (subtract):  C = A - B,
			// where arrays A, B, and C are m-by-1.

			vecdiv(Resid, Z_vect, Ratio, j);
			// Function:  vecdiv(A, B, C, m)
			// Vector divide:  C = A / B,
			// where arrays A, B, and C are m-by-1.

			ratio_check = 4.5;

			bad = 0;
			/*
			double H_tran[15][6];
			double tempMat[15][6];

			// Z = H*P*H'+R ------ Matrix multiplication for Z(6x6)

			for (int i = 0; i < 6; i++) {
				for (int j = 0; j < 15; j++) {
					H_tran[j][i] = H[i][j];
				}
			}

			for (int i = 0; i < 15; ++i) {
				for (int j = 0; j < 6; ++j) {
					for (int k = 0; k < 15; ++k) {
						tempMat[i][j] += P[i][k] * H_tran[k][j];
					}
				}
			}

			for (int i = 0; i < 6; ++i) {
				for (int j = 0; j < 6; ++j) {
					for (int k = 0; k < 15; ++k) {
						Z[i][j] += H[i][k] * tempMat[k][j];
					}
				}
			}

			for (int i = 0; i < 6; i++) {
				for (int j = 0; j < 6; j++) {
					Z[i][j] = Z[i][j] + R[i][j];
				}
			}
			*/
			// -----------------------------------
			// Z_vect = sqrt(diagonal(Z))


			for (i = 0; i < number_rows; i++) {
				//Ratio[i] = Resid[i] / Z_vect[i];    // Computing Ratio for innovation check

				if (USE_GPS > 0.5) {
					ratio_check = 15;
				}
				else {
					ratio_check = 4.5;
				}
				if (Ratio[i] > ratio_check) {
					bad_fix_at_time = current_time;
					bad = 1;
					printf("Innovation sequence fail: Bad fix at time %f\n", current_time);
					printf(" -Failure at measurement row = %d\n", i);
					//std::cout << "Press Enter to continue..."; // C++ equivalent to MATLAB's "pause"
					//std::cin.get(); // waits for Enter key
					system("pause"); // Only works on Windows not Linux OS
				}
			}
			// if(Kinput.mode = 1)

			   //

			if (bad < 1) {
				//                                            REWRITE ABOVE PER MATLAB
				//------------------------------------------------------------------------------------------------------
				// -------------------------------------------------------Keep the below
				// 
			// Kalman Gain Calculation ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

				// Since we could have restructured the H and R matrices above, we must now start
				// all calculations anew if number_rows > 0

				if (number_rows > 0)
				{
					// mq=P*H^T
					j = max;
					jj = max;
					kk = number_rows;
					dmabt(_P, _H, _mq, j, jj, kk);
					// Function:  dmabt(A, B, C, l, m, n)
					// Matrix multiply:  C = A * B^T,  (B^T signifies b transposed)
					// where A is l-by-m, B^T is m-by-n (or B is n-by-m), and C is l-by-n.


					//Temp1=H*mq ...=H*P*H^T
					j = number_rows;
					jj = max;
					kk = j;
					dmab(_H, _mq, _Temp1, j, jj, kk);  // _Temp1=_H*_mq
					// where _H is j-by-jj, _mq is jj-by-j, and _Temp1 is j-by-j.


					//Temp2=H*P*H^T + R
					j = number_rows;
					dmapb(_Temp1, _R, _Temp2, j, j);
					// Function:  dmapb(A, b, C, m, n)
					// Matrix add (plus):  C= A + B,
					// where arrays A, b, and C are m-by-n.

					dmzero(_Temp1, NStates, NStates);
					vezero(temp_vect, NStates);
					vezero(temp_vect2, NStates);
					iezero(temp_ivect, NStates);

					// mcq = inverse of (H*P*H^T + R) or Inv(Temp2)
					j = number_rows;
					k = dmaib(_Temp2, _I, _mcq, _Temp1, temp_vect, temp_vect2, temp_ivect, j, j);
					if (k > 0)
					{
						printf("Singular Matrix!\n");
						compute_filter = false;
					} // Must return without any Kalman updates, if inverse has problems...
					// Function:  dmaib(A, B, X, Temp, temp_vect, temp_vect, temp_ivect, m, n)
					// Given matrices A and B, evaluate X = A\B without doing an explicit
					// inversion of the A matrix.  (I.e., it is more efficient than doing
					// a matrix inversion followed by a matrix multiply.  However, if the
					// inverse of A is required, then set B equal to the identity matrix.)
					//   A is m-by-m
					//   B is m-by-n
					//   X is m-by-n
					// The temporary (scratch) real vectors and the one integer vector are of length m.
					// If the A matrix is not singular the return value is 0; otherwise it is 1
					// and the "solution" for x is not valid.

					if (compute_filter)
					{
						// KalmG = P*H^T * inv(H*P*H^T + R)
						j = max;
						jj = number_rows;
						kk = number_rows;
						dmab(_mq, _mcq, _KalmG, j, jj, kk);
						// Function:  dmab(A, B, C, l, m, n)
						// Matrix multiply:  C = A * B,
						// where A is l-by-m, B is m-by-n, and C is l-by-n.
					//-------------------------------------------------------------------------------
					// Now to UPDATE the Matrices by the Kalman Gain

					// Update the Covariance Matrix

					// KalmG*H  ----> Temp1
						j = max;
						jj = number_rows;
						kk = max;
						dmab(_KalmG, _H, _Temp1, j, jj, kk);
						// Function:  dmab(A, B, C, l, m, n)
						// Matrix multiply:  C = A * B,
						// where A is l-by-m, B is m-by-n, and C is l-by-n.

					// (I - KalmG*H)  ------> Temp2
						j = max;
						jj = max;
						dmamb(_I, _Temp1, _Temp2, j, jj);
						// Function:  dmamb(A, B, C, m, n)
						// Matrix difference (minus):  C = A - B,
						// where arrays A, B, and C are m-by-n.

					// P * (I - KalmG*H)^T   ----------> Temp1
						j = max;
						jj = max;
						kk = max;
						dmabt(_P, _Temp2, _Temp1, j, jj, kk);
						// Function:  dmabt(A, B, C, l, m, n)
						// Matrix multiply:  C = A * B^T,  (B^T signifies b transposed)
						// where A is l-by-m, B^T is m-by-n (or B is n-by-m), and C is l-by-n.

					// (I - KalmG*H) * P * (I - KalmG*H)^T  ----> Temp3
									// Temp3 = Temp2 * Temp1
						j = max;
						jj = max;
						kk = max;
						dmab(_Temp2, _Temp1, _Temp3, j, jj, kk);
						// Function:  dmab(A, B, C, l, m, n)
						// Matrix multiply:  C = A * B,
						// where A is l-by-m, B is m-by-n, and C is l-by-n.

					// R*KalmG^T     ---------> Temp1
						j = number_rows;
						jj = number_rows;
						kk = max;
						dmabt(_R, _KalmG, _Temp1, j, jj, kk);
						// Function:  dmabt(A, B, C, l, m, n)
						// Matrix multiply:  C = A * B^T,  (B^T signifies b transposed)
						// where A is l-by-m, B^T is m-by-n (or B is n-by-m), and C is l-by-n.

					// KalmG * R*KalmG^T    ------> Temp2
									// Temp2 = KalmG * Temp1
						j = max;
						jj = number_rows;
						kk = max;
						dmab(_KalmG, _Temp1, _Temp2, j, jj, kk);
						// Function:  dmab(A, B, C, l, m, n)
						// Matrix multiply:  C = A * B,
						// where A is l-by-m, B is m-by-n, and C is l-by-n.

					//P = (I - KalmG*H) * P * (I - KalmG*H)^T + KalmG * R*KalmG^T
					//P = Temp3 + Temp2
						dmapb(_Temp3, _Temp2, _P, max, max);
						// Function:  dmapb(A, b, C, m, n)
						// Matrix add (plus):  C= A + B,
						// where arrays A, b, and C are m-by-n.

					// Update the State Matrix

						for (i = 0; i < max; i++) temp_vect[i] = X[i][0];  // temp_vect=X (1st column)

						// temp_vect2 = H * X
						j = number_rows;
						dmav(_H, temp_vect, temp_vect2, j, max);
						// Function:  dmav(A, v, u, m, n)
						// Matrix-vector multiply:  u = A * v,
						// where A is m-by-n matrix, v is n-by-1 vector, and u is m-by-1 vector.

						// Resid = Y - H*X
						j = number_rows;
						vecsub(Y, temp_vect2, Resid, j);
						// Function:  vecsub(A, B, C, m)
						// Vector minus (subtract):  C = A - B,
						// where arrays A, B, and C are m-by-1.

					// KalmG*Resid    --------> temp_vect
						j = max;
						jj = number_rows;
						dmav(_KalmG, Resid, temp_vect, j, jj);
						// Function:  dmav(A, v, u, m, n)
						// Matrix-vector multiply:  u = A * v,
						// where A is m-by-n matrix, v is n-by-1 vector, and u is m-by-1 vector.

					// X(1st col) = KalmG*Resid
						for (i = 0; i < max; i++) X[i][0] = X[i][0] + temp_vect[i];

						// Accumulate the State Vector
						for (i = 0; i < max; i++) X[i][1] = X[i][0] + X[i][1];
						for (i = 0; i < max; i++) X[i][0] = 0.0;
					} //if (compute_filter)

				} // if (number_rows...
			} //if bad <1
		} // if FilterRun
//
	// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

	//--------------------------------------------------------------------- USE THE FILTER OUTPUT AS THE MATLAB CODE DOES
	// *** OUTPUT DATA ***
	//---------------------------------------------------------------------
			//end of Filkter run check

			KFoutput.time = current_time;

			KFoutput.delta_Pn = X[0][1]; //Xsys1[0] = X[0][1]
			X[0][1] = 0;
			KFoutput.delta_Pe = X[1][1]; //X[1][1]
			X[1][1] = 0;
			KFoutput.delta_Pd = X[2][1]; //X[2][1] ... etc
			X[2][1] = 0;
			KFoutput.delta_Vn = X[3][1];
			X[3][1] = 0;
			KFoutput.delta_Ve = X[4][1];
			X[4][1] = 0;
			KFoutput.delta_Vd = X[5][1];
			X[5][1] = 0;

			KFoutput.delta_tiltN = X[6][1];
			X[6][1] = 0;
			KFoutput.delta_tiltE = X[7][1];
			X[7][1] = 0;
			KFoutput.delta_tiltD = X[8][1];
			X[8][1] = 0;

			KFoutput.delta_ABx = X[9][1];
			X[9][1] = 0;
			KFoutput.delta_ABy = X[10][1];
			X[10][1] = 0;
			KFoutput.delta_ABz = X[11][1];
			X[11][1] = 0;
			KFoutput.delta_GBx = X[12][1];
			X[12][1] = 0;
			KFoutput.delta_GBy = X[13][1];
			X[13][1] = 0;
			KFoutput.delta_GBz = X[14][1];
			X[14][1] = 0;

		//} // bad < 1 Good fix

		//MEASUREMENT = 0;
		KalmanInput.GPS_valid = 0;
		KalmanInput.VertVelocity_valid = 0;
		KalmanInput.LevelVelocity_valid = 0;

		//} //  if (KInput.mode = 1)
	}

//
}
//*****************************************************************************
// This function is called at exit time to free-up the long double pointers:
//*****************************************************************************
void free_arrays_proto()
{

    free_convert_dmatrix(_X, 0);		
    free_convert_dmatrix(_F, 0);		
    free_convert_dmatrix(_W, 0);		
    free_convert_dmatrix(_I, 0);		
    free_convert_dmatrix(_P, 0);		
    free_convert_dmatrix(_Phi, 0);		
    free_convert_dmatrix(_Q, 0);		
    free_convert_dmatrix(_Temp1, 0);	
    free_convert_dmatrix(_Temp2, 0);	
    free_convert_dmatrix(_Temp3, 0);	
    free_convert_dmatrix(_Temp4, 0);	
    free_convert_dmatrix(_Temp5, 0);	
    free_convert_dmatrix(_H, 0);		
    free_convert_dmatrix(_R, 0);
    free_convert_dmatrix(_Z, 0);
    free_convert_dmatrix(_mq, 0);		
    free_convert_dmatrix(_mcq, 0);
    free_convert_dmatrix(_KalmG, 0);	

	free_dvector(Y, 0);
	free_dvector(temp_vect, 0);
	free_dvector(temp_vect2, 0);
	free_dvector(temp_vect3, 0);
	free_dvector(Z_vect, 0);
	free_dvector(Resid, 0);
	free_dvector(Ratio, 0);

    free_ivector(temp_ivect, 0);

}
//----------------- end of free_arrays_proto ----------------------------------

