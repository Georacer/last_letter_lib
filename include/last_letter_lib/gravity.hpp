#include <stdexcept>
#include <Eigen/Eigen>

#include <last_letter_lib/math_utils.hpp>
#include <last_letter_lib/uav_utils.hpp>

using namespace std;
using Eigen::Quaterniond;
using Eigen::Vector3d;

namespace last_letter_lib
{
	class Gravity
	{
	public:
		Gravity();
		~Gravity();
		double g;
		uav_utils::Wrench_t wrenchGrav;
		Vector3d getForce(Quaterniond orientation, double g, double mass);
		Vector3d getTorque(Quaterniond orientation, double g, double mass);
	};
} // namespace last_letter_lib
