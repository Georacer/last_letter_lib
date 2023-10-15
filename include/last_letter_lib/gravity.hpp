#include <stdexcept>
#include <Eigen/Eigen>

#include <last_letter_lib/math_utils.hpp>
#include <last_letter_lib/uav_utils.hpp>

using namespace std;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using last_letter_lib::uav_utils::SimState_t;
using last_letter_lib::uav_utils::Geoid;
using last_letter_lib::math_utils::Inertial;

#define E2_EARTH Geoid::WGS84_e2
#define RP_EARTH Geoid::WGS84_Ra * (1.0 - E2_EARTH)

#define grav_const 3.986004418e14
#define grav_temp (2.0 / Geoid::WGS84_Ra) * (1.0 + Geoid::EARTH_flattening + (Geoid::EARTH_Omega * Geoid::EARTH_Omega) * (Geoid::WGS84_Ra * Geoid::WGS84_Ra) * RP_EARTH / grav_const)


namespace last_letter_lib
{
	class GravityModel
	{
	public:
        virtual void calcGravity(const SimState_t states, const Inertial /*inertial*/)=0; // perform the gravity calculations
		Vector3d getForce(const SimState_t states, const Inertial inertial);
		Vector3d getTorque(const SimState_t /*states*/, const Inertial /*inertial*/) {return Vector3d(0,0,0);}
		void setG(double v) {_g = v;}
	private:
		double _g;
	};

	class GravitySimple : public GravityModel
	{
	public:
		GravitySimple();
		~GravitySimple();
        void calcGravity(const SimState_t /*states*/, const Inertial /*inertial*/) {}; // perform the gravity calculations
	};

	class GravityClassic : public GravityModel
	{
	public:
		GravityClassic() = default;
		~GravityClassic() = default;
        void calcGravity(const SimState_t states, const Inertial /*inertial*/); // perform the gravity calculations
	};
} // namespace last_letter_lib
