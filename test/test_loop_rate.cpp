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

    cout << seconds << " second(s) elapsed for " << loopNum << " steps" << endl;

}
