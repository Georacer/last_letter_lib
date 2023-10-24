#include <iostream>
#include <gtest/gtest.h>

#include "yaml-cpp/yaml.h"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/propulsion/propulsion.hpp"
#include "last_letter_lib/environment.hpp"

#include "test_utils.hpp"

using namespace std;
using namespace Eigen;

using namespace last_letter_lib;
using namespace last_letter_lib::programming_utils;
using namespace last_letter_lib::math_utils;
using namespace last_letter_lib::propulsion;

TEST(TestPropulsion, TestPropulsion1)
{
    ParameterManager config = load_config_aircraft("skywalker_2013");
    Propulsion *motor1 = buildPropulsion(config.filter("prop/motor1/"), config.filter("world"));
    SimState_t states = build_aircraft_state_from_config(config);

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
    input.value[2] = 0.5;

    EnvironmentModel environmentModel = EnvironmentModel();
    environmentModel.initialize(config.filter("env"));
    environmentModel.calcEnvironment(states);

    motor1->setInput(input);
    motor1->stepEngine(states, inertial, environmentModel.environment); // perform one step in the propdynamics
    EXPECT_GT(motor1->omega, 0);
    EXPECT_GT(motor1->wrenchProp.force(0), 0);
    EXPECT_EQ(motor1->wrenchProp.force(1), 0);
    EXPECT_EQ(motor1->wrenchProp.force(2), 0);
    EXPECT_LT(motor1->wrenchProp.torque(0), 0);
    EXPECT_EQ(motor1->wrenchProp.torque(1), 0);
    EXPECT_EQ(motor1->wrenchProp.torque(2), 0);
}
