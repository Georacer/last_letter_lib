#include <Eigen/Eigen>
#include "last_letter_lib/environment.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/prog_utils.hpp"

using namespace std;

int main(int, char ** ) {

    std::string param_path = "last_letter_lib/test/parameters/";
    std::string fullWorldFilename = param_path + "world.yaml";
    std::string fullEnvironmentFilename = param_path + "environment.yaml";
    YAML::Node worldConfig = YAML::LoadFile(fullWorldFilename);
    YAML::Node environmentConfig = YAML::LoadFile(fullEnvironmentFilename);

    EnvironmentModel environment_obj = EnvironmentModel(environmentConfig, worldConfig);
    SimState_t states;
    states.pose.position = Eigen::Vector3d(0,0,-10);
    states.geoid.altitude = -states.pose.position.z();
    states.velocity.linear = Eigen::Vector3d(10, 0, 1);

    environment_obj.calcEnvironment(states);
    auto result = environment_obj.environment;

    Eigen::Vector3d airdata = getAirData(states.velocity.linear - result.wind);

    cout << "Body-velocity:\n" << states.velocity.linear << endl;
    cout << "***Resulting environment data" << endl;
    cout << "wind:\n" << result.wind << endl;
    cout << "Airspeed: " << airdata.x() << endl;
    cout << "Alpha (deg): " << airdata.y()*180/M_PI << endl;
    cout << "Beta (deg): " << airdata.z()*180/M_PI << endl;
    cout << "density: " << result.density << endl;
    cout << "pressure: " << result.pressure << endl;
    cout << "temperature: " << result.temperature << endl;
    cout << "gravity: " << result.gravity << endl;

}
