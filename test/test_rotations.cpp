#include <iostream>
#include "gravity.hpp"
#include "Eigen/Eigen"

using namespace std;
using namespace Eigen;

int main(int, char **)
{
    cout << "Example of Eigen Y-rotation by 90 degrees:\n" << AngleAxis<double>(M_PI/2, Vector3d::UnitY()).matrix() << endl;
    cout << "This standard Eigen rotation is wrong, has the inverse polarity." << endl;

    Vector3d euler(M_PI/4, M_PI/4, 0);
    // Vector3d euler(M_PI/2, 0, 0);
    // Vector3d euler(0, M_PI/2, 0);
    // Vector3d euler(0, 0, M_PI/2);

    cout << "Starting with Euler angles (RPY):\n" << euler*180/M_PI << endl;
    cout << "Desired transformations are from Earth-frame to Body-frame, i.e. v_b = R*v_e" << endl;

    Eigen::Vector3d unitVectX = Eigen::Vector3d::UnitX();
    cout << "Starting with a unit vector in the Inertial frame, pointing to X-direction (unitVectX)\n";
    cout << unitVectX << endl;

    cout << "Generating a quaternion q_eb from said Euler angles\n";
    Quaterniond q_eb = euler2quat(euler);
    cout << q_eb.w() << "\n" << q_eb.vec() << endl;
    cout << "Quaternion norm: " << q_eb.norm() << endl;

    cout << "Going back to Euler angles:\n";
    cout << quat2euler(q_eb.conjugate())*180/M_PI << endl;

    cout << "Multiplying q_eb * unitVectX:\n";
    cout << q_eb*unitVectX << endl;
    // cout << "This should be equal to (0.07071, 0.5, 0.5)\n";

    cout << "Generating a transformation object with q_eb:\n";
    Eigen::Transform<double, 3, Eigen::Affine> t_eb;
    t_eb = q_eb;
    cout << t_eb.matrix() << endl;
    cout << "Rotating t_eb * unitVectX:\n";
    cout << t_eb * unitVectX << endl;
    cout << "Should obtain the same result as before.\n";

    cout << "Defining q_01 as a 90-deg rotation by X0 and then q_12 as a 90-deg rotation by Y1.\n";
    cout << "The point v2 =R(v1), v1=(1, 0, 0) is:\n";
    Eigen::Quaterniond q_01, q_12;
    q_01 = euler2quat(Eigen::Vector3d(M_PI/2, 0, 0));
    q_12 = euler2quat(Eigen::Vector3d(0, M_PI/2, 0));
    cout << q_12*q_01*Eigen::Vector3d(1, 0, 0) << endl;
    cout << "The required transformation ACCORDING TO MY LIBRARY is v2 = q_12*q_01*v1\n";

    cout << "Testing combined translations, rotations:\n";

    cout << "By default, Eigen performs pre-multiplied transformations in the inerti axes and gives the coordinates in the inertial frame:\n";
    cout << "The same transformation can be though of as post-multiplications in the intermediate axes.\n";
    Eigen::Transform<double, 3, Eigen::Affine> t;
    t = Eigen::Translation<double, 3>(0, 0, 1) * Eigen::AngleAxis<double>(M_PI/2, Eigen::Vector3d::UnitY());
    cout << "T(0, 0, 1)*R(90,Y) * (0, 0, 0) =\n";
    cout << t * Eigen::Vector3d::Zero() << endl;

    cout << "Similarly...\n";
    t = Eigen::Translation<double, 3>(1, 0, 0) * Eigen::AngleAxis<double>(M_PI/2, Eigen::Vector3d::UnitY()) * Eigen::Translation<double, 3>(0, 0, 1);
    cout << "T(1, 0, 0)*R(90,Y)*T(0, 0, 1) * (0, 0, 0) =\n";
    cout << t * Eigen::Vector3d::Zero() << endl;

}