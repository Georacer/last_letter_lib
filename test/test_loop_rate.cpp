#include <iostream>
#include <time.h>
#include <chrono>

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
    double t_steps, t_derivs;

    cout << "Testing simulation loop rate" << endl;

    // Calculate step time requirements 
    auto t_start = chrono::steady_clock::now();
    for (uint32_t i=0; i<loopNum; i++)
    {
        uav.step();
    }
    auto t_end = chrono::steady_clock::now();
    t_steps = (double)chrono::duration_cast<chrono::milliseconds>(t_end-t_start).count()/1000;

    SimState_t newState;
    newState = uav.state;

    // Calculate dynamics calculation time requirements 
    t_start = chrono::steady_clock::now();
    for (uint32_t i=0; i<loopNum; i++)
    {
        uav.dynamics.calcWrench(uav.state, uav.kinematics.inertial, uav.environmentModel.environment);
    }
    t_end = chrono::steady_clock::now();
    t_derivs = (double)chrono::duration_cast<chrono::milliseconds>(t_end-t_start).count()/1000;

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

    cout << t_steps << " second(s) elapsed for " << loopNum << " simulation steps" << endl;
    cout << t_derivs << " second(s) elapsed for " << loopNum << " model derivative calculations" << endl;

}
