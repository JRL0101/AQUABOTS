// Implementation file
// function definitions are placed here

//#include "Guidance.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#define PI 3.14159265358979323846
#define _CRT_SECURE_NO_WARNINGS

using namespace std;

void guidance_angle(double pos_cur[3], double vehicle_v[3], double pos_target[3],double &Theta) {

	// Pure-Pursuit Method
	// Heads to the desired position set by the USER but doesn't go along a desired path
	// Takes the angle between the unit vectors (directions) of the vehicle's velocity and heading vector

	// Inputs:
	// 
	//	pos_cur		: [X_c,Y_c,Z_c] initial position (m)
	//  vehicle_v	: [V_x,V_y,V_z] current velocity vector (m/s)
	//	pos_target	: [X_t,Y_t,Z_t] target position (Given from the GUI) (m)

	// Output:
	//
	// Theta		: Angle to deflect rudder to align the direction of the vehicle toward the desired destination (Deg)

	// NOTE:
	// USING only 1st and 2nd elements of input vectors.
	// Simplified down to a 2D problem for an UAMV only riding along the surface of the water. Thus, change in altitude in negligible;


	// Initial Local Variables

	double hdg_vector[2] = { 0 };
	double v_norm = 0;
	double hdg_norm = 0;

	double v_unit[2] = { 0 };
	double hdg_unit[2] = { 0 };

	double dot_val = 0;
	double angle = 0;
	double cross_val = 0;

	// Tolerance

	double tol = 1e-6;

	// Step 1: Heading vector from current to target
	hdg_vector[0] = pos_target[0] - pos_cur[0];
	hdg_vector[1] = pos_target[1] - pos_cur[0];

	// Step 2: Normalize both heading and velocity vectors

	v_norm = sqrt(pow(vehicle_v[0], 2) + pow(vehicle_v[1], 2));
	hdg_norm = sqrt(pow(hdg_vector[0], 2) + pow(hdg_vector[1], 2));

	if (v_norm < tol || hdg_norm < tol) { // Avoid division by zero or undefined angles
		Theta = 0;
	}
	else {

		v_unit[0] = vehicle_v[0] / v_norm;
		v_unit[1] = vehicle_v[1] / v_norm;

		hdg_unit[0] = hdg_vector[0] / hdg_norm;
		hdg_unit[1] = hdg_vector[1] / hdg_norm;

		// Step 3: Compute angle between vectors using dot product

		dot_val = v_unit[0] * hdg_unit[0] + v_unit[1] * hdg_unit[1];
		dot_val = max(min(dot_val, 1.0),-1.0); // Clamp to valid acos range
		angle = acos(dot_val);                 // Always in range [0,pi]  // CHECK 

		// Step 4: Determine sign of rotation using 2D scalar cross product

		cross_val = v_unit[0] * hdg_unit[1] - v_unit[1] * hdg_unit[0];

		// Step 5: Apply sign to angle

		if (cross_val < 0) {
			Theta = angle * (-1.0); // Turn Right
		}
		else {
			Theta = angle; // Turn Left
		}
	}

}
