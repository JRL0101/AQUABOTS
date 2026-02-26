#include <gtest/gtest.h>
#include "control.h"

TEST(GuidanceTest, StraightAhead) {
    double pos_cur[3] = {0,0,0};
    double vel[3] = {1,0,0};
    double target[3] = {10,0,0};
    double theta = control::simulate_guidance(pos_cur, vel, target);
    EXPECT_NEAR(theta, 0.0, 1e-6);
}

TEST(PIControlTest, ZeroError) {
    double out = control::simulate_pi(0.0);
    EXPECT_NEAR(out, 0.0, 1e-6);
}
