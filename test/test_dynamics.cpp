#include <iostream>

#include "yaml-cpp/yaml.h"
#include "prog_utils.hpp"
#include "uav_utils.hpp"
#include "dynamics.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char * argv[])
{
    // Read parameter files
    string paramDir = "../test/parameters/";
    string uav_name = argv[1];
    cout << "Building dynamics for UAV: " << uav_name << endl;
    string prop_filename = "propulsion.yaml";
    string aero_filename = "aerodynamics.yaml";
    string ground_filename = "ground.yaml";
    string inertial_filename = "inertial.yaml";
    string init_filename = "init.yaml";
    string world_filename = "world.yaml";
    string environment_filename = "environment.yaml";

    string fullWorldFilename = paramDir + world_filename;
    string fullEnvironmentFilename = paramDir + environment_filename;
    string fullPropFilename = paramDir+uav_name+"/"+prop_filename;
    string fullAeroFilename = paramDir+uav_name+"/"+aero_filename;
    string fullGroundFilename = paramDir+uav_name+"/"+ground_filename;
    string fullInertialFilename = paramDir+uav_name+"/"+inertial_filename;
    string fullInitFilename = paramDir+uav_name+"/"+init_filename;

    std::cout << "Reading parameter files" << endl;
    YAML::Node worldConfig = YAML::LoadFile(fullWorldFilename);
    YAML::Node environmentConfig = YAML::LoadFile(fullEnvironmentFilename);
    YAML::Node aeroConfig = YAML::LoadFile(fullAeroFilename);
    YAML::Node propConfig = YAML::LoadFile(fullPropFilename);
    YAML::Node groundConfig = YAML::LoadFile(fullGroundFilename);
    YAML::Node inertialConfig = YAML::LoadFile(fullInertialFilename);
    YAML::Node initConfig = YAML::LoadFile(fullInitFilename);
    
    // Create input data
    std::cout << "Creating input structures" << endl;
    std::vector<double> tempVec;

    SimState_t states;
    getParameterList(initConfig, "velLin", tempVec);
    states.velocity.linear = Eigen::Vector3d(tempVec.data());
    tempVec.clear();
    getParameterList(initConfig, "position", tempVec);
    states.pose.position = Eigen::Vector3d(tempVec.data());
    states.geoid.altitude = -states.pose.position.z();
	// Read initial orientation quaternion
    std::vector<double> doubleVect;
	getParameterList(initConfig, "orientation", doubleVect);
	states.pose.orientation = Quaterniond(doubleVect[3], doubleVect[0], doubleVect[1], doubleVect[2]);

    Inertial_t inertial;
    getParameter(inertialConfig, "m", inertial.mass);
    double j_x, j_y, j_z, j_xz;
    getParameter(inertialConfig, "j_x", j_x);
    getParameter(inertialConfig, "j_y", j_y);
    getParameter(inertialConfig, "j_z", j_z);
    getParameter(inertialConfig, "j_xz", j_xz);
    inertial.J << j_x, 0, -j_xz,
                    0, j_y, 0,
                    -j_xz, 0, j_z;

    Input_t input;
    input.value[0] = 0.1;
    input.value[1] = 0.1;
    input.value[2] = 0.5;

    EnvironmentModel environmentModel = EnvironmentModel(environmentConfig, worldConfig);
    environmentModel.calcEnvironment(states);
    Environment_t environment = environmentModel.environment;

    // Create dynamics object
    std::cout << "Creating dynamics object" << endl;
    Dynamics dynamics(worldConfig, aeroConfig, propConfig, groundConfig);
    dynamics.setInput(input);
    dynamics.calcWrench(states, inertial, environment);
    Eigen::Vector3d force, torque;
    force = dynamics.getForce();
    torque = dynamics.getTorque();

    // Print results
    cout << "Aileron input set to: " << input.value[0] << endl;
    cout << "Motor input set to: " << input.value[2] << endl;
    cout << "Body-frame wind:\n" << environment.wind << endl;
    cout << "Total force:\n" << force << endl;
    cout << "Total torque:\n" << torque << endl;
}
