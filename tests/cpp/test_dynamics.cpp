#include <iostream>
#include <gtest/gtest.h>

#include "yaml-cpp/yaml.h"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/dynamics.hpp"

#include "test_utils.hpp"

using namespace std;
using namespace Eigen;

using namespace last_letter_lib::programming_utils;
using namespace last_letter_lib::uav_utils;
using namespace last_letter_lib;

TEST(TestDynamics, TestDynamics1)
{
    // Read parameter files
    auto config = load_config_aircraft("skywalker_2013");
    SimState_t state = build_aircraft_state_from_config(config);

    // Create input data
    std::vector<double> tempVec;

    double mass = config.filter("inertial").get<double>("m");
    double j_x, j_y, j_z, j_xz;
    j_x = config.filter("inertial").get<double>("j_x");
    j_y = config.filter("inertial").get<double>("j_y");
    j_z = config.filter("inertial").get<double>("j_z");
    j_xz = config.filter("inertial").get<double>("j_xz");
    std::vector<double> J = {
        j_x, 0, -j_xz,
        0, j_y, 0,
        -j_xz, 0, j_z};
    Inertial inertial(mass, J);

    Input_t input;
    input.value[0] = 0.1;
    input.value[1] = 0.1;
    input.value[2] = 0.5;
    input.value[3] = 0.0;

    EnvironmentModel environmentModel = EnvironmentModel(config.filter("env"), config.filter("world"));
    environmentModel.calcEnvironment(state);
    Environment_t environment = environmentModel.environment;

    // Create LinkStateMap
    LinkStateMap_t linkStateMap;
    linkStateMap["body_frd"] = state;
    linkStateMap["thruster_0"] = state;

    // Create dynamics object
    Dynamics dynamics(config.filter("world"), config.filter("aero"), config.filter("prop"), config.filter("ground"));
    dynamics.setInput(input);
    LinkWrenchMap_t linkWrenchMap = dynamics.calcWrench(linkStateMap, inertial, environment);
    Eigen::Vector3d force, torque;

    force = linkWrenchMap["body_frd"].force;
    torque = linkWrenchMap["body_frd"].torque;
    EXPECT_NEAR(force(2), -17, 1);
    EXPECT_NEAR(torque(2), 7.5, 1);

    force = linkWrenchMap["thruster_0"].force;
    torque = linkWrenchMap["thruster_0"].torque;
    EXPECT_NEAR(force(0), 25, 1);
    EXPECT_LE(torque(0), 0);
}
