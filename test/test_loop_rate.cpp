#include <iostream>
#include <time.h>

#include "uav_model.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char * argv[])
{
    ConfigsStruct_t configs;
    configs = loadModelConfig(argv[1]);
    UavModel uav(configs);

    SimState_t state;
    state = uav.state;

    uint32_t loopNum=10000;
    time_t t_before, t_after;
    double seconds;

    cout << "Testing simulation loop rate" << endl;

    t_before = time(NULL);
    for (uint32_t i=0; i<loopNum; i++)
    {
        uav.step();
    }
    t_after = time(NULL);

    seconds = difftime(t_after, t_before);

    SimState_t newState;
    newState = uav.state;

    cout << "Initial State:\n" << endl;
    cout << "Position\n" << state.pose.position << endl;
    cout << "Coordinates\n" << state.geoid.latitude << "\n" << state.geoid.longitude << "\n" << state.geoid.altitude <<  endl;
    cout << "Orientation\n" << state.pose.orientation.vec() << "\n" << state.pose.orientation.w() << endl;
    cout << "Linear Velocity\n" << state.velocity.linear << endl;
    cout << "Angular Velocity\n" << state.velocity.angular << endl;
    cout << "Linear Acceleration\n" << state.acceleration.linear << endl;
    cout << "Angular Acceleration\n" << state.acceleration.angular << endl;

    cout << "Propagated State:\n" << endl;
    cout << "Position\n" << newState.pose.position << endl;
    cout << "Coordinates\n" << newState.geoid.latitude << "\n" << newState.geoid.longitude << "\n" << newState.geoid.altitude << endl;
    cout << "Orientation\n" << newState.pose.orientation.vec() << "\n" << newState.pose.orientation.w() << endl;
    cout << "Linear Velocity\n" << newState.velocity.linear << endl;
    cout << "Angular Velocity\n" << newState.velocity.angular << endl;
    cout << "Linear Acceleration\n" << newState.acceleration.linear << endl;
    cout << "Angular Acceleration\n" << newState.acceleration.angular << endl;

    cout << seconds << " second(s) elapsed for " << loopNum << " steps" << endl;

}
