#define _CRT_SECURE_NO_WARNINGS
/// Defining Variables for the Kalman Filter Application.
double static pi=3.1415926535897932384626433832795;
double static nmile=6076.115486;			// conversion to feet
double static kn_ft=nmile/3600.0;			// conversion to ft/sec from kts
double static ae = 2.092564632545932e+007;	// Radius of earth in feet
double static e2 = 6.6943799014e-003;		// Ellipsoidal model of eccentricity (WGS-84)

//extern double ORE_mount[3]; // The array that holds the LHS Track Point Mounting Angles
extern double last_time;

//extern bool Startup;
//extern bool Set_Depth_Range;

extern short mycase;

int static NStates = 15;
int static Max = 15;
int static NMeas = 6;

extern double **_X, **_F, **_W, **_I, **_P, **_Phi, **_Q,
	**_Temp1, **_Temp2, **_Temp3, **_Temp4, **_Temp5, 
	**_H, **_R, **_Z, **_mq, **_mcq, **_KalmG;

extern double X[15][2],F[15][15], W[15][15], I[15][15],
	P[15][15], Phi[15][15], Q[15][15], 
	Temp1[15][15], Temp2[15][15],Temp3[15][15],
	Temp4[15][15],Temp5[15][15],
	H[6][15], R[6][6], Z[6][6],
	mq[15][6], mcq[6][6], KalmG[15][6];

extern double *Y, *temp_vect, *temp_vect2, *temp_vect3, *Z_vect, *Resid, *Ratio;

//double temp_vect[15], temp_vect2[15], temp_vect3[15], Z_vect[7]; Resid[7], Ratio[7];

extern int *temp_ivect;//, temp_ivect[15];
//short *_mycase;

// ///////////////////////////////////////////////////////////////////////////
/*
extern double
HBeta,				// Helo velocity error North & East Markov correlation coefficient
Hsigma,				// Helo velocity error rms 

LBdown,			// LHS down velocity error Markov correlation coeffficient
LBdown_sigma,			// LHS down velocity error 

LRP_beta,			// LHS roll & pitch Markov correlation coefficient
LRP_sigma,	// LHS roll and pitch rms 

LH_beta,		// LHS heading error Markov coefficient
LH_sigma,		// LHS heading error rms 

Vhdg_beta,     // NTR heading error Markov coefficient
Vhdg_sigma,
Vpitch_beta,		// NTR heading error Markov coefficient
Vpitch_sigma,	// NTR pitch rms = 0.5 degrees
Bvb,			// NTR Buoyancy error Markov coefficient
sigma_vb,		        // NTR Buoyancy error rms = 0.6 ft/sec below
KVB,					// Bringing in constant of integrated bouyancy velocity error
Boc,			// NTR ocean current error Markov correlation coefficient (initial value only as it will change with vehicle flight)
Voc_sigma,		// NTR ocean current error rms = 2 knots
base_tau,			// Setting up for Vehicle dynamic ocean current formulation underway
base_sigma,		// Setting up for  "      "
Beta_sfB,		// NTR Shaft speed bias error correlation coefficient
sfB_sigma,			// NTR shaft speed bias rms = 0.1 ft/sec  
shaftSF_sigma,		// NTR shaft speed scale factor error = 5%

Baz ,				// ORE Sonar Az error correlation - small correlation time for whi
sigma_Az;	// ORE Sonar Az error rms - will be dynamically updated

extern double initial_depth, Drange, Percent_depth, Last_Lat, Last_Lon, Base_Lat, Base_Lon,
		R_norm_V, R_merid_V;

/// We now define and initialize the LHS dead reckon variables
extern double  dt_helo_vel;
extern double	NHelo, EHelo, NLhs1, NLhs2, ELhs1, ELhs2, Base_Depth;
*/

extern double R_norm_V, R_merid_V;


#pragma once

/*class KalmanFilter
{
public:
	KalmanFilter( KalmanInputType, KalmanOutputType );

	~KalmanFilter( KalmanInputType, KalmanOutputType );


};*/

