#include <iostream>

#include "yaml-cpp/yaml.h"

#include "prog_utils.hpp"
#include "uav_utils.hpp"
#include "ground_reaction.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char * argv[])
{
    string paramDir = "../test/parameters/";
    string uav_name = argv[1];
    cout << "Building ground reactions for UAV: " << uav_name << endl;
    string ground_filename = "contactPts.yaml";
    string world_filename = "world.yaml";

    string fullWorldFilename = paramDir + "world.yaml";
    string fullGroundFilename = paramDir+uav_name+"/"+ground_filename;

    YAML::Node worldConfig = filterConfig(YAML::LoadFile(fullWorldFilename), "/world/");
    YAML::Node groundConfig = YAML::LoadFile(fullGroundFilename);
    YAML::Node groundConfig1 = filterConfig(groundConfig, "airframe/");

    GroundReaction * airframe = buildGroundReaction(groundConfig1, worldConfig);
    
    SimState_t states;
    states.velocity.linear = Eigen::Vector3d(0,0,0);
    states.pose.position = Eigen::Vector3d(0,0,-0.1);
    states.geoid.altitude = states.pose.position.z();

    WrenchSum_t wrenchSum;

    Vector3d groundForce = airframe->getForce(states, wrenchSum);
    Vector3d groundTorque = airframe->getTorque(states, wrenchSum);
    cout << "Ground force:\n" << groundForce << endl;
    cout << "Ground torque:\n" << groundTorque << endl;
}
