#include "control.h"
#include "Guidance.h"
#include "PIcontrol.h"

namespace control {

double simulate_guidance(double pos_cur[3], double vehicle_v[3], double pos_target[3]) {
    double theta = 0.0;
    guidance_angle(pos_cur, vehicle_v, pos_target, theta);
    return theta;
}

double simulate_pi(double theta) {
    double control = 0.0;
    double integral = 0.0;
    PIcontroller(theta, control, integral);
    return control;
}

} // namespace control
