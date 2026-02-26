#pragma once

namespace control {
// Simulation hooks for SITL
// Returns rudder deflection angle in radians
double simulate_guidance(double pos_cur[3], double vehicle_v[3], double pos_target[3]);
// Returns control output from PI controller
double simulate_pi(double theta);
}
