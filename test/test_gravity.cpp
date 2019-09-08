#include <iostream>
#include "gravity.hpp"
#include "Eigen/Eigen"

using namespace std;
using namespace Eigen;

int main(int, char **)
{
    Gravity gravityModel;

    Vector3d euler(M_PI/4, M_PI/4, 0);
    // Vector3d euler(M_PI/2, 0, 0);
    // Vector3d euler(0, M_PI/2, 0);
    // Vector3d euler(0, 0, M_PI/2);

    Quaterniond orientation_eb = AngleAxis<double>(-euler.x(), Vector3d::UnitX())
                                * AngleAxis<double>(-euler.y(), Vector3d::UnitY())
                                * AngleAxis<double>(-euler.z(), Vector3d::UnitZ());

    Eigen::Transform<double, 3, Eigen::Affine> t1;
    t1 = orientation_eb;

    Vector3d force = gravityModel.getForce(orientation_eb, 9.81, 1);
    Vector3d torque = gravityModel.getTorque(orientation_eb, 9.81, 1);

    // cout << "Desired rotation quaternion:\n"<< q.w() << "\n" << q.vec() << endl;
    cout << "Example of Eigen Y-rotation by 90 degrees:\n" << AngleAxis<double>(M_PI/2, Vector3d::UnitY()).matrix() << endl;
    cout << "This standard Eigen rotation is wrong, has the inverse polarity." << endl;
    cout << "Correct Earth-to-Body rotation quaternion:\n" << orientation_eb.w() << "\n" << orientation_eb.vec() << endl;
    cout << "Quaternion norm: " << orientation_eb.norm() << endl;
    cout << "Equivalent rotation matrix:\n" << t1.matrix() << endl;
    cout << "Body gravity force: \n" << force << endl;
    cout << "Gravity norm: " << force.norm() << endl;
    cout << "Body gravity torque:\n" << torque << endl;
}