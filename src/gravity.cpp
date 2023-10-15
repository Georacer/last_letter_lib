#include "last_letter_lib/gravity.hpp"

namespace last_letter_lib
{

	//////////////////////////
	// Define Gravity class
	//////////////////////////

	Vector3d GravityModel::getForce(const SimState_t states, const Inertial inertial) {
		UnitQuaternion orientation_eb = states.pose.orientation.conjugate();
		if (math_utils::isnan(orientation_eb))
		{
			throw runtime_error("gravity.cpp: NaN member in orientation quaternion");
		}

		auto gravVect = Vector3d(0, 0, inertial.mass * _g);
		Vector3d force = orientation_eb * gravVect;
		if (math_utils::isnan(force))
		{
			throw runtime_error("gravity.cpp: NaN member in force vector");
		}

		return force;
		}

	// Class constructor
	GravitySimple::GravitySimple()
	{
		setG(9.81);
	}

	// Class destructor
	GravitySimple::~GravitySimple()
	{
	}


	///////////////////
	// Calculate gravity
	void GravityClassic::calcGravity(const SimState_t states, const Inertial /*inertial*/)
	{
		double slat2 = pow(sin(M_PI / 180 * states.geoid.latitude), 2);
		double Re2 = pow(Geoid::WGS84_Ra, 2);

		double grav0 = uav_utils::Geoid::EARTH_grav * (1.0 + 0.00193185138639 * slat2) / sqrt(1.0 - 0.00669437999013 * slat2);
		double gravity = grav0 * (1.0 - grav_temp * states.geoid.altitude + 3.0 * (pow(states.geoid.altitude, 2) / Re2));
		if (isnan(gravity))
		{
			throw runtime_error("environment.cpp: gravity is NaN");
		}
		setG(gravity);
	}
} // namespace last_letter_lib
