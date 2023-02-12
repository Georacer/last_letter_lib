#include <iostream>

#include "yaml-cpp/yaml.h"

#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/ground_reaction/ground_reaction.hpp"

using namespace std;
using namespace Eigen;

int main(int /* argc */, char * argv[])
{
    string paramDir = "last_letter_lib/test/parameters/";
    string uav_name = argv[1];
    cout << "Building ground reactions for UAV: " << uav_name << endl;
    string ground_filename = "ground.yaml";
    string world_filename = "world.yaml";

    string fullWorldFilename = paramDir + "world.yaml";
    string fullGroundFilename = paramDir+uav_name+"/"+ground_filename;

    YAML::Node worldConfig = YAML::LoadFile(fullWorldFilename);
    YAML::Node groundConfig = YAML::LoadFile(fullGroundFilename);

    GroundReaction * airframe = buildGroundReaction(groundConfig, worldConfig);

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
