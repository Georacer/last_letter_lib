#include <iostream>

#include "yaml-cpp/yaml.h"
#include "prog_utils.hpp"
#include "uav_utils.hpp"
#include "aerodynamics.hpp"

using namespace std;
using namespace Eigen;

int main(int, char **)
{
    string paramDir = "../test/parameters/";
    string uav_name = "skywalker_2013/";
    string aero_filename = "aerodynamics.yaml";
    string init_filename = "init.yaml";
    string world_filename = "world.yaml";
    string inertial_filename = "inertial.yaml";

    string fullWorldFilename = paramDir + "world.yaml";
    string fullEnvironmentFilename = paramDir + "environment.yaml";
    string fullAeroFilename = paramDir+uav_name+aero_filename;
    string fullInertialFilename = paramDir+uav_name+"/"+inertial_filename;
    string fullInitFilename = paramDir+uav_name+"/"+init_filename;

    YAML::Node worldConfig = YAML::LoadFile(fullWorldFilename);
    YAML::Node environmentConfig = YAML::LoadFile(fullEnvironmentFilename);
    YAML::Node aeroConfig = YAML::LoadFile(fullAeroFilename);
    YAML::Node aeroConfig1 = filterConfig(aeroConfig, "airfoil1/");
    YAML::Node inertialConfig = YAML::LoadFile(fullInertialFilename);
    YAML::Node initConfig = YAML::LoadFile(fullInitFilename);
    
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

    EnvironmentModel environmentModel = EnvironmentModel(environmentConfig, worldConfig);
    environmentModel.calcEnvironment(states);

    Input_t input;
    input.value[0] = 0.1;
    input.value[1] = 0.1;
    input.value[2] = 0.5;

    Aerodynamics * airfoil1 = buildAerodynamics(aeroConfig1);
    airfoil1->setInput(input, aeroConfig1);

    airfoil1->stepDynamics(states, inertial, environmentModel.environment); // perform one step in the aerodynamics
    cout << "Body-frame wind:\n" << environmentModel.environment.wind << endl;
    cout << "Body-frame relative airdata:\n" << airfoil1->airspeed << "\n" << airfoil1->alpha << "\n" << airfoil1->beta << endl;
    cout << "Aerodynamic force:\n" << airfoil1->wrenchAero.force << endl;
    cout << "Aerodynamic torque:\n" << airfoil1->wrenchAero.torque << endl;

}
