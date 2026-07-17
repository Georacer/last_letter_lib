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

    Input input;
    input.value[0] = 0.1;
    input.value[1] = 0.1;
    input.value[2] = 0.5;
    input.value[3] = 0.0;

    EnvironmentModel environmentModel = EnvironmentModel();
    environmentModel.initialize(config.filter("env"));
    environmentModel.calcEnvironment(state);
    Environment_t environment = environmentModel.environment;

    // Create dynamics object
    Dynamics dynamics(config.filter("world"), config.filter("aero"), config.filter("prop"), config.filter("ground"));
    dynamics.setInput(input);
    dynamics.update_local_state(state, environment);
    dynamics.calc_model(state);

    EXPECT_NEAR(dynamics.wrench_sum.wrenchAero.force(2), -19, 1);
    EXPECT_NEAR(dynamics.wrench_sum.wrenchAero.torque(2), 7.5, 1);

    EXPECT_NEAR(dynamics.wrench_sum.wrenchProp.force(0), 25, 1);
    EXPECT_LE(dynamics.wrench_sum.wrenchProp.torque(0), 0);
}
