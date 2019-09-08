#include <Eigen/Eigen>
#include "environment.hpp"
#include "uav_utils.hpp"
#include "prog_utils.hpp"

using namespace std;

int main(int, char ** ) {

    std::string param_path = "../test/parameters/";
    std::string fullWorldFilename = param_path + "world.yaml";
    std::string fullEnvironmentFilename = param_path + "environment.yaml";
    YAML::Node worldConfig = filterConfig(YAML::LoadFile(fullWorldFilename), "/world/");
    YAML::Node environmentConfig = filterConfig(YAML::LoadFile(fullEnvironmentFilename), "/environment/");

    EnvironmentModel environment_obj = EnvironmentModel(environmentConfig, worldConfig);
    SimState_t states;
    states.pose.position = Eigen::Vector3d(0,0,-10);
    states.geoid.altitude = -states.pose.position.z();

    environment_obj.calcEnvironment(states);
    auto result = environment_obj.environment;
    cout << "Resulting environment data" << endl;
    cout << "wind:\n" << result.wind << endl;
    cout << "density: " << result.density << endl;
    cout << "pressure: " << result.pressure << endl;
    cout << "temperature: " << result.temperature << endl;
    cout << "gravity: " << result.gravity << endl;

}