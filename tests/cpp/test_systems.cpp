#include "yaml-cpp/yaml.h"
#include <fstream>
#include <gtest/gtest.h>
#include <iostream>
#include <string>
#include <stdexcept>

// #include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/uav_model.hpp"
#include "last_letter_lib/systems.hpp"

// #include "test_utils.hpp"

using namespace std;

// using namespace last_letter_lib::programming_utils;
using namespace last_letter_lib::uav_utils;
using namespace last_letter_lib::systems;
using namespace last_letter_lib;

class ComponentTest : public ::testing::Test
{
protected:
    void SetUp() override{};

    Component c{"comp_name"};
};

TEST_F(ComponentTest, TestComponent1)
{
    ASSERT_EQ(c.get_param<double>("inertial/mass"), 0.0);
}

TEST_F(ComponentTest, TestComponent2)
{
    c.set_param("inertial/mass", 1.0);
    ASSERT_EQ(c.get_param<double>("inertial/mass"), 1.0);
}

TEST_F(ComponentTest, TestComponent3)
{
    c.set_param("inertial/mass", 1.0);
    c.update_parameters();
    ASSERT_EQ(c.inertial.mass, 1.0);
}

TEST_F(ComponentTest, TestComponent4)
{
    c.set_param("inertial/tensor/j_xx", 1.0);
    c.set_param("inertial/tensor/j_yy", 1.0);
    c.set_param("inertial/tensor/j_zz", 1.0);
    c.update_parameters();
    ASSERT_EQ(c.inertial.tensor, Eigen::MatrixXd::Identity(3, 3));
}
