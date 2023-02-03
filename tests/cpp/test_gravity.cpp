#include <iostream>
#include "last_letter_lib/gravity.hpp"
#include "Eigen/Eigen"

using namespace std;
using namespace Eigen;

int main(int, char **)
{
    Vector3d euler(M_PI/4, M_PI/4, 0);
    // Vector3d euler(M_PI/2, 0, 0);
    // Vector3d euler(0, M_PI/2, 0);
    // Vector3d euler(0, 0, M_PI/2);

    cout << "Starting with Euler angles (RPY):\n" << euler*180/M_PI << endl;
    cout << "Desired transformations are from Earth-frame to Body-frame, i.e. v_b = R*v_e" << endl;

    cout << "Generating a quaternion q_eb from said Euler angles\n";
    Quaterniond orientation_eb = euler2quat(euler);

    Gravity gravityModel;
    Vector3d force = gravityModel.getForce(orientation_eb, 9.81, 1);
    Vector3d torque = gravityModel.getTorque(orientation_eb, 9.81, 1);

    cout << "Body gravity force: \n" << force << endl;
    cout << "Gravity norm: " << force.norm() << endl;
    cout << "Body gravity torque:\n" << torque << endl;
}
