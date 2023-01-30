#include <iostream>

#include "yaml-cpp/yaml.h"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/propulsion/propulsion.hpp"

using namespace std;
using namespace Eigen;

int main(int /* argc */, char * argv[])
{
    string paramDir = "last_letter_lib/test/parameters/";
    string uav_name = argv[1];
    cout << "Building propulsion for UAV: " << uav_name << endl;
    string prop_filename = "propulsion.yaml";
    string init_filename = "init.yaml";
    string world_filename = "world.yaml";
    string inertial_filename = "inertial.yaml";

    string fullWorldFilename = paramDir + "world.yaml";
    string fullEnvironmentFilename = paramDir + "environment.yaml";
    string fullPropFilename = paramDir+uav_name+"/"+prop_filename;
    string fullInertialFilename = paramDir+uav_name+"/"+inertial_filename;
    string fullInitFilename = paramDir+uav_name+"/"+init_filename;

    YAML::Node worldConfig = YAML::LoadFile(fullWorldFilename);
    YAML::Node environmentConfig = YAML::LoadFile(fullEnvironmentFilename);
    YAML::Node propConfig = YAML::LoadFile(fullPropFilename);
    YAML::Node propConfig1 = filterConfig(propConfig, "motor1/");
    YAML::Node inertialConfig = YAML::LoadFile(fullInertialFilename);
    YAML::Node initConfig = YAML::LoadFile(fullInitFilename);

    Propulsion * motor1 = buildPropulsion(propConfig1, worldConfig);

    SimState_t states;
    std::vector<double> doubleVect;
    getParameterList(initConfig, "velLin", doubleVect);
    states.velocity.linear = Eigen::Vector3d(doubleVect.data());
    doubleVect.clear();
    getParameterList(initConfig, "position", doubleVect);
    states.pose.position = Eigen::Vector3d(doubleVect.data());
    states.geoid.altitude = -states.pose.position.z();
	// Read initial orientation quaternion
	doubleVect.clear();
	getParameterList(initConfig, "orientation", doubleVect);
	states.pose.orientation = Quaterniond(doubleVect[3], doubleVect[0], doubleVect[1], doubleVect[2]);
    Eigen::Transform<double, 3, Eigen::Affine> t;
    t = states.pose.orientation;
    cout << "Earth to Body frame rotation:\n" << t.matrix() << endl;

    Inertial_t inertial;
    getParameter(inertialConfig, "m", inertial.mass);
    double j_x, j_y, j_z, j_xz;
    getParameter(inertialConfig, "j_x", j_x);
    getParameter(inertialConfig, "j_y", j_y);
    getParameter(inertialConfig, "j_z", j_z);
    getParameter(inertialConfig, "j_xz", j_xz);
    inertial.J << j_x, 0, -j_xz,
                    0, j_y, 0,
                    -j_xz, 0, j_x;

    Input_t input;
    input.value[2] = 0.5;

    EnvironmentModel environmentModel = EnvironmentModel(environmentConfig, worldConfig);
    environmentModel.calcEnvironment(states);

    motor1->setInput(input);
    motor1->stepEngine(states, inertial, environmentModel.environment); // perform one step in the propdynamics
    cout << "Motor input set to: " << motor1->inputMotor << endl;
    cout << "Body-frame wind:\n" << environmentModel.environment.wind << endl;
    cout << "Propeller-frame normal wind: " << motor1->normalWind << endl;
    cout << "Propeller omega: " << motor1->omega << endl;
    cout << "Propulsion force:\n" << motor1->wrenchProp.force << endl;
    cout << "Propulsion torque:\n" << motor1->wrenchProp.torque << endl;
}
