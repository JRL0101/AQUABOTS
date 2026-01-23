// Header file for the Kalman Filter Data Structures

//-----------------------------------------------------------------------------
// Structures:

//#ifndef KFTYPE_H
//#define KFTYPE_H
#pragma once

///		Kalman Filter Input Data Structure

struct KalmanInputType {
	double time;								// Seconds
	int mode;									// 0, 1, or 2
	

	// MATLAB input

	double GPS_lat;
	double GPS_lon;
	double GPS_alt;
	double GPS_time;

	int GPS_go;

	int GPS_valid;
	int VertVelocity_valid;
	int LevelVelocity_valid;
};


struct KalmanOutputType{

	double time;								// seconds
	short filter_mode;							// 0, 1, 2, or 3

	// MATLAB output

	double delta_Pn;
	double delta_Pe;
	double delta_Pd;
	double delta_Vn;
	double delta_Ve;
	double delta_Vd;
	double delta_tiltN;
	double delta_tiltE;
	double delta_tiltD;
	double delta_ABx;
	double delta_ABy;
	double delta_ABz;
	double delta_GBx;
	double delta_GBy;
	double delta_GBz;
	double covPn;
	double covPe;
	double covVn;
	double covVe;
	double covPd;
	double covPsiD;


};

extern KalmanInputType KInput;
extern KalmanOutputType KFoutput;

void KalmanFilter(KalmanInputType KalmanInput, KalmanOutputType* _KFoutput, int Filter_Run);

//#endif // KFTYPE_H