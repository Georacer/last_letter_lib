#include "yaml-cpp/yaml.h"
#include <fstream>
#include <gtest/gtest.h>
#include <iostream>
#include <string>
#include <stdexcept>

#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/uav_model.hpp"

#include "test_utils.hpp"

using namespace std;

using namespace last_letter_lib::programming_utils;
using namespace last_letter_lib;

TEST(ProgUtilsTest, TestYaml1)
{

    std::string param_path = "tests/cpp/";
    std::string filename = "test_params.yaml";
    std::string full_filename = param_path + filename;
    YAML::Node config = YAML::LoadFile(full_filename);
    EXPECT_EQ(config["world"]["simRate"].as<int>(), 500);

    // Test basic parameter access
    std::string timeControls = config["world"]["timeControls"].as<std::string>();
    std::string integratorTypeStr =
        config["world"]["integratorType"].as<std::string>();
    EXPECT_EQ(integratorTypeStr, "0");

    config["lastLogin"] = 5.0;

    // // Test alternate accessor
    // int integratorType;
    // getParameter(config, "world/integratorType", integratorType);
    // EXPECT_EQ(integratorType, 0);

    // std::vector<int> intVector;
    // getParameterList(config, "anIntSequence", intVector);
    // string vectorString = vectorToString(intVector);
    // EXPECT_EQ(vectorString, "0 1 2 ");

    // std::vector<double> doubleVector;
    // getParameterList(config, "aDoubleSequence", doubleVector);
    // vectorString = vectorToString2(doubleVector);
    // EXPECT_EQ(vectorString, "0, 1.1, 2.2, ");
}

TEST(ProgUtilsTest, GeneralTest)
{

    // Read parameter files
    ParameterManager configs = load_config_aircraft("skywalker_2013");

    // Test filter config
    // YAML::Node newConfig = filterConfig(configs.prop, "motor1/");
    // EXPECT_EQ(newConfig["motorType"].as<int>(), 1);

    // Test parameter randomization
    // YAML::Node testConfig = load_config_simple();
    // std::vector<string> paramNames;
    // double doubleValue;
    // getParameter(testConfig, "scalarParameter", doubleValue);
    // EXPECT_EQ(doubleValue, 1);
    // paramNames.push_back("scalarParameter");
    // YAML::Node randomizedConfig = randomizeConfig(testConfig, paramNames, 0.1);
    // getParameter(randomizedConfig, "scalarParameter", doubleValue);
    // EXPECT_NE(doubleValue, 1);

    // Test config struct randomizing
    // ConfigsStruct_t newConfigStruct = load_config_aircraft("skywalker_2013", true);
    // EXPECT_NE(newConfigStruct.aero["airfoil1/c_l_pn"].as<double>(), -1);

    // Test parameter loading from aircraft models folder in user home.
    ParameterManager loadedConfig = loadModelConfig("skywalker_2013");
}

class A
{
public:
    A(string name) : parameters(name)
    {
    }
    ParameterManager parameters;
};

class B
{
public:
    B(string name) : parameters(name)
    {
    }
    void setChild(shared_ptr<A> child_p)
    {
        child_ptr = child_p;
        parameters.register_child_mngr(child_ptr->parameters);
    }
    ParameterManager parameters;
    shared_ptr<A> child_ptr;
};

class ParameterManagerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        child = make_shared<A>("child");
        child->parameters.set("param_1", 1, false);
        child->parameters.set("param_2/param_3", 3, false);

        parent = make_shared<B>("parent");
        parent->setChild(child);
        parent->parameters.set("param_4", 4, false);
        parent->parameters.set("param_5/param_6", 6, false);
    }

    shared_ptr<A> child;
    shared_ptr<B> parent;
};

TEST_F(ParameterManagerTest, TestAccessParent1)
{
    int r = parent->parameters.get<int>("param_4");
    ASSERT_EQ(r, 4);
}

TEST_F(ParameterManagerTest, TestAccessParent2)
{
    int r = parent->parameters.get<int>("param_5/param_6");
    ASSERT_EQ(r, 6);
}

TEST_F(ParameterManagerTest, TestAccessChild1)
{
    int r = child->parameters.get<int>("param_1");
    ASSERT_EQ(r, 1);
}

TEST_F(ParameterManagerTest, TestAccessChild2)
{
    int r = child->parameters.get<int>("param_2/param_3");
    ASSERT_EQ(r, 3);
}

TEST_F(ParameterManagerTest, TestAccessChild3)
{
    int r = parent->parameters.get<int>("child/param_2/param_3");
    ASSERT_EQ(r, 3);
}

TEST_F(ParameterManagerTest, TestExist1)
{
    ASSERT_EQ(child->parameters.exists("param_1"), true);
}

TEST_F(ParameterManagerTest, TestExist2)
{
    ASSERT_EQ(child->parameters.exists("not_existing"), false);
}

TEST_F(ParameterManagerTest, TestInvalidGet)
{
    ASSERT_THROW(child->parameters.get<int>("not_existing"), std::invalid_argument);
}

TEST_F(ParameterManagerTest, TestFilter)
{
    ParameterManager node = parent->parameters.filter("child");
    ASSERT_EQ(node.get<int>("param_1"), 1);
}
