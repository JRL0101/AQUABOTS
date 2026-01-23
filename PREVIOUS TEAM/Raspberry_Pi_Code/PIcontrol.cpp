// Implementation file
// function definitions are placed here

#include "PIcontrol.h"
#include <cmath>
#include <iostream>
#define PI 3.14159265358979323846
#define _CRT_SECURE_NO_WARNINGS

void PIcontroller(double Theta,double &Control, double &integral) {

	// Input from Guidance: 
	// Theta

	// Output from PIcontroller: 
	// Control

	// Local variables inside the controller --------------

	// Gains (Kp, Ki)

	double Kp = 1;  // Proportional Gain - changes the amount of oscillations
	double Ki = .1e-4; // Integral Gain  - compensates the error remaining after the proportional gain

	// Time step - constant determined by system's processing

	double dt = 0.1;  // Change to match system's time step

	// ----------------------------------------------------
	// -------------- REAL TIME CONTROLLER ----------------
	// ----------------------------------------------------

	integral += Theta * dt;

	// Control output
	Control = Kp * Theta + Ki * integral;

	// END
}


