// Cannot implement without working, independent kinematics.

#include <iostream>

#include "yaml-cpp/yaml.h"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/uav_model.hpp"

using namespace std;
using namespace Eigen;

int main(int /* argc */, char *argv[])
{
    // Read parameter files
    string paramDir = "last_letter_lib/test/parameters/";
    string uav_name = argv[1];
    cout << "Building uav model for UAV: " << uav_name << endl;
    string prop_filename = "propulsion.yaml";
    string aero_filename = "aerodynamics.yaml";
    string ground_filename = "ground.yaml";
    string inertial_filename = "inertial.yaml";
    string init_filename = "init.yaml";
    string world_filename = "world.yaml";
    string environment_filename = "environment.yaml";

    string fullWorldFilename = paramDir + world_filename;
    string fullEnvironmentFilename = paramDir + environment_filename;
    string fullPropFilename = paramDir + uav_name + "/" + prop_filename;
    string fullAeroFilename = paramDir + uav_name + "/" + aero_filename;
    string fullGroundFilename = paramDir + uav_name + "/" + ground_filename;
    string fullInertialFilename = paramDir + uav_name + "/" + inertial_filename;
    string fullInitFilename = paramDir + uav_name + "/" + init_filename;

    cout << "Reading parameter files" << endl;
    ConfigsStruct_t configs;
    configs.world = YAML::LoadFile(fullWorldFilename);
    configs.env = YAML::LoadFile(fullEnvironmentFilename);
    configs.prop = YAML::LoadFile(fullPropFilename);
    configs.aero = YAML::LoadFile(fullAeroFilename);
    configs.ground = YAML::LoadFile(fullGroundFilename);
    configs.inertial = YAML::LoadFile(fullInertialFilename);
    configs.init = YAML::LoadFile(fullInitFilename);

    // Create model
    UavModel uavModel(configs);
    SimState_t state = uavModel.state;

    // Edit motor parameters
    cout << endl;
    string paramName = "motor1/s_prop";
    cout << "Original parameter value: " << uavModel.configs.prop[paramName] << endl;
    cout << "Setting " << paramName << " to 0.0" << endl;
    uavModel.configs.prop[paramName] = 0.0;
    cout << "Reading back the parameter: " << uavModel.configs.prop[paramName] << endl;

    uavModel.updateConfigAll();

    // Set input
    Input_t input;
    input.value[0] = 0.1;
    input.value[1] = 0.1;
    input.value[2] = 0.5;
    uavModel.setInput(input);

    // Step simulation
    uavModel.step();
    SimState_t newState = uavModel.state;

    // Print results
    cout << "Aileron input\t: " << input.value[0] << endl;
    cout << "Elevator input\t: " << input.value[1] << endl;
    cout << "Throttle input\t: " << input.value[2] << endl;
    cout << "Rudder input\t: " << input.value[3] << endl;
    cout << endl;

    cout << "\nInitial State:\n"
         << endl;
    cout << "Position\n"
         << state.pose.position << endl;
    cout << "Coordinates\n"
         << state.geoid.latitude << "\n"
         << state.geoid.longitude << "\n"
         << state.geoid.altitude << endl;
    cout << "Orientation\n"
         << state.pose.orientation.vec() << "\n"
         << state.pose.orientation.w() << endl;
    cout << "Linear Velocity\n"
         << state.velocity.linear << endl;
    cout << "Angular Velocity\n"
         << state.velocity.angular << endl;
    cout << "Linear Acceleration\n"
         << state.acceleration.linear << endl;
    cout << "Angular Acceleration\n"
         << state.acceleration.angular << endl;

    cout << "\nPropagated State:\n"
         << endl;
    cout << "Coordinates\n"
         << newState.geoid.latitude << "\n"
         << newState.geoid.longitude << "\n"
         << newState.geoid.altitude << endl;
    cout << "Position\n"
         << newState.pose.position << endl;
    cout << "Orientation\n"
         << newState.pose.orientation.vec() << "\n"
         << newState.pose.orientation.w() << endl;
    cout << "Linear Velocity\n"
         << newState.velocity.linear << endl;
    cout << "Angular Velocity\n"
         << newState.velocity.angular << endl;
    cout << "Linear Acceleration\n"
         << newState.acceleration.linear << endl;
    cout << "Angular Acceleration\n"
         << newState.acceleration.angular << endl;
}
