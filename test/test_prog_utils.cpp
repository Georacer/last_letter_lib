#include <iostream>
#include <fstream>
#include <string>
#include "yaml-cpp/yaml.h"
#include "prog_utils.hpp"
#include "uav_utils.hpp"

std::string uav_name = "skywalker_2013";

int main(int, char * argv[]){

    // Read parameter files
    string paramDir = "../test/parameters/";
    string uav_name = argv[1];
    cout << "Building uav model for UAV: " << uav_name << endl;
    string prop_filename = "propulsion.yaml";
    string aero_filename = "aerodynamics.yaml";
    string ground_filename = "ground.yaml";
    string inertial_filename = "inertial.yaml";
    string init_filename = "init.yaml";
    string world_filename = "world.yaml";
    string environment_filename = "environment.yaml";
    string randomizer_filename = "randomizer.yaml";

    string fullWorldFilename = paramDir + world_filename;
    string fullEnvironmentFilename = paramDir + environment_filename;
    string fullPropFilename = paramDir+uav_name+"/"+prop_filename;
    string fullAeroFilename = paramDir+uav_name+"/"+aero_filename;
    string fullGroundFilename = paramDir+uav_name+"/"+ground_filename;
    string fullInertialFilename = paramDir+uav_name+"/"+inertial_filename;
    string fullInitFilename = paramDir+uav_name+"/"+init_filename;
    string fullRandomizerFilename = paramDir+uav_name+"/"+randomizer_filename;

    std::cout << "Reading parameter files" << endl;
    ConfigsStruct_t configs;
    configs.world = YAML::LoadFile(fullWorldFilename);
    configs.env = YAML::LoadFile(fullEnvironmentFilename);
    configs.prop = YAML::LoadFile(fullPropFilename);
    configs.aero = YAML::LoadFile(fullAeroFilename);
    configs.ground = YAML::LoadFile(fullGroundFilename);
    configs.inertial = YAML::LoadFile(fullInertialFilename);
    configs.init = YAML::LoadFile(fullInitFilename);
    YAML::Node randomizerConfig = YAML::LoadFile(fullRandomizerFilename);


    // Test filter config
    YAML::Node newConfig = filterConfig(configs.prop, "motor1/");
    cout << "Filtered config:" << endl;
    cout << newConfig << endl;

    // Test parameter randomization
    cout << "Testing parameter randomization" << endl;
    YAML::Node testConfig = YAML::LoadFile("../test/test_params.yaml");
    std::vector<string> paramNames;
    double doubleValue;
    getParameter(testConfig, "/scalarParameter", doubleValue);
    cout << "Original parameter value: " << doubleValue << endl;
    paramNames.push_back("/scalarParameter");
    YAML::Node randomizedConfig = randomizeConfig(testConfig, paramNames, 0.1);
    getParameter(randomizedConfig, "/scalarParameter", doubleValue);
    cout << "Randomized value: " << doubleValue << endl;

    // Test config struct randomizing
    cout << "Testing config struct randomizing" << endl;
    ConfigsStruct_t newConfigStruct = randomizeConfigsStruct(configs, randomizerConfig);

    // Test parameter loading
    ConfigsStruct_t loadedConfig = loadModel(uav_name);
}