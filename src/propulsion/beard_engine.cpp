////////////////////////////////////////
// Engine model found in R. Beard's book
////////////////////////////////////////

#include "last_letter_lib/propulsion/propulsion.hpp"

namespace last_letter_lib
{
namespace propulsion
{

// Constructor
EngBeard::EngBeard(string name)
    : Propulsion(name)
{
	std::cout << "Building new Beard Engine" << std::endl;
	omega = 0; // Initialize engine rotational speed
}

// Destructor
EngBeard::~EngBeard()
{
}

void EngBeard::update_parameters()
{
	Propulsion::update_parameters();

	s_prop = get_param<double>("s_prop");
	c_prop = get_param<double>("c_prop");
	k_motor = get_param<double>("k_motor");
	k_t_p = get_param<double>("k_t_p");
	k_omega = get_param<double>("k_omega");
}

// Update motor rotational speed and other states for each timestep
void EngBeard::updateRadPS(SimState_t /* states */, Inertial /* inertial */, Environment_t /*environment*/)
{
	omega = rotationDir * inputMotor * k_omega;
}

// Calculate propulsion forces
void EngBeard::getForce(SimState_t /* states */, Inertial /* inertial */, Environment_t environment)
{
	rho = environment.density;
	double x, y, z;
	x = 1.0 / 2.0 * rho * s_prop * c_prop * (pow(inputMotor * k_motor, 2) - pow(normalWind, 2));
	// cap x above zero
	x = std::max(0.0, x);
	y = 0;
	z = 0;
	wrenchProp.force = Vector3d(x, y, z);
	if (!wrenchProp.force.allFinite())
	{
		throw runtime_error("propulsion.cpp/EngBeard: State NaN in wrenchProp.force");
	}
}

// Calculate propulsion torques
void EngBeard::getTorque(SimState_t /* states */, Inertial /* inertial */, Environment_t /* environment */)
{
	double x, y, z;
	x = -rotationDir * k_t_p * pow(omega, 2);
	y = 0;
	z = 0;
	wrenchProp.torque = Vector3d(x, y, z);
	if (!wrenchProp.torque.allFinite())
	{
		throw runtime_error("propulsion.cpp/EngBeard: State NaN in wrenchProp.torque");
	}
}
} // namespace propulsion
} // namespace last_letter_lib
