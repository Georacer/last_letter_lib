#include <stdexcept>
#include <Eigen/Eigen>

#include <math_utils.hpp>
#include <uav_utils.hpp>

using namespace std;
using Eigen::Vector3d;
using Eigen::Quaterniond;

class Gravity
{
	public:
	Gravity();
	~Gravity();
	double g;
	Wrench_t wrenchGrav;
	Vector3d getForce(Quaterniond orientation, double g, double mass);
	Vector3d getTorque(Quaterniond orientation, double g, double mass);
};