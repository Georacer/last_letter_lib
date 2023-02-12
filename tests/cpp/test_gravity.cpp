#include <iostream>
#include <gtest/gtest.h>
#include "Eigen/Eigen"

#include "last_letter_lib/gravity.hpp"
#include "last_letter_lib/math_utils.hpp"

using namespace std;
using namespace Eigen;

using namespace last_letter_lib::math_utils;
using namespace last_letter_lib;

TEST(TestGravity, TestGravity1)
{
    double g{9.81};
    Vector3d euler(M_PI / 4, M_PI / 4, 0);
    // Vector3d euler(M_PI/2, 0, 0);
    // Vector3d euler(0, M_PI/2, 0);
    // Vector3d euler(0, 0, M_PI/2);

    // cout << "Starting with Euler angles (RPY):\n"
    //  << euler * 180 / M_PI << endl;
    // cout << "Desired transformations are from Earth-frame to Body-frame, i.e. v_b = R*v_e" << endl;

    // cout << "Generating a quaternion q_eb from said Euler angles\n";
    Quaterniond orientation_eb = euler2quat(euler);

    Gravity gravityModel;
    Vector3d force = gravityModel.getForce(orientation_eb, g, 1);
    Vector3d torque = gravityModel.getTorque(orientation_eb, g, 1);

    EXPECT_NEAR(force(0), -g * sqrt(2) / 2, 1e-3);
    EXPECT_NEAR(force(1), g / 2, 1e-3);
    EXPECT_NEAR(force(2), g / 2, 1e-3);
    EXPECT_EQ(force.norm(), g);
    EXPECT_EQ(torque.norm(), 0);
}
