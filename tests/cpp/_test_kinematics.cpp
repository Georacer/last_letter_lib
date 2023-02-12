#include <iostream>

#include "yaml-cpp/yaml.h"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/kinematics.hpp"

using namespace std;
using namespace Eigen;

int main(int /* argc */, char * argv[])
{
    // Read parameter files
    string paramDir = "last_letter_lib/test/parameters/";
    string uav_name = argv[1];
    cout << "Building kinematics for UAV: " << uav_name << endl;
    string inertial_filename = "inertial.yaml";
    string init_filename = "init.yaml";
    string world_filename = "world.yaml";

    string fullWorldFilename = paramDir + world_filename;
    string fullInertialFilename = paramDir+uav_name+"/"+inertial_filename;
    string fullInitFilename = paramDir+uav_name+"/"+init_filename;

    std::cout << "Reading parameter files" << endl;
    YAML::Node worldConfig = YAML::LoadFile(fullWorldFilename);
    YAML::Node inertialConfig = YAML::LoadFile(fullInertialFilename);
    YAML::Node initConfig = YAML::LoadFile(fullInitFilename);

    // Create input data
    std::cout << "Creating input structures" << endl;

    std::vector<double> tempVec;
    SimState_t state;
    getParameterList(initConfig, "velLin", tempVec);
    state.velocity.linear = Eigen::Vector3d(tempVec.data());
    tempVec.clear();
    getParameterList(initConfig, "position", tempVec);
    state.pose.position = Eigen::Vector3d(tempVec.data());
    state.geoid.altitude = -state.pose.position.z();
	// Read initial orientation quaternion
    std::vector<double> doubleVect;
	getParameterList(initConfig, "orientation", doubleVect);
	state.pose.orientation = Quaterniond(doubleVect[3], doubleVect[0], doubleVect[1], doubleVect[2]);

    Eigen::Vector3d inpForce(1, 0.1, 2);
    Eigen::Vector3d inpTorque(0.2, 0.1, 0.05);
    Wrench_t inpWrench;
    inpWrench.force = inpForce;
    inpWrench.torque = inpTorque;

    // Create kinematics object
    SimState_t newState;
    Kinematics kinematics(inertialConfig, worldConfig);
	newState = kinematics.propagateState(state, inpWrench); // Use this method to calculate state integral


    // Print results
    cout << "Input force:\n" << inpForce << endl;
    cout << "Input torque:\n" << inpTorque << endl;

    cout << "Initial State:\n" << endl;
    cout << "Position\n" << state.pose.position << endl;
    cout << "Orientation\n" << state.pose.orientation.vec() << "\n" << state.pose.orientation.w() << endl;
    cout << "Linear Velocity\n" << state.velocity.linear << endl;
    cout << "Angular Velocity\n" << state.velocity.angular << endl;
    cout << "Linear Acceleration\n" << state.acceleration.linear << endl;
    cout << "Angular Acceleration\n" << state.acceleration.angular << endl;

    cout << "Propagated State:\n" << endl;
    cout << "Position\n" << newState.pose.position << endl;
    cout << "Orientation\n" << newState.pose.orientation.vec() << "\n" << newState.pose.orientation.w() << endl;
    cout << "Linear Velocity\n" << newState.velocity.linear << endl;
    cout << "Angular Velocity\n" << newState.velocity.angular << endl;
    cout << "Linear Acceleration\n" << newState.acceleration.linear << endl;
    cout << "Angular Acceleration\n" << newState.acceleration.angular << endl;
}
