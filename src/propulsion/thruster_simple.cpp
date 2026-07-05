////////////////////////
// Simple thruster model
////////////////////////

#include "last_letter_lib/propulsion/propulsion.hpp"

namespace last_letter_lib {
namespace propulsion {

// Constructor
ThrusterSimple::ThrusterSimple(string name_p)
    : Thruster(name_p)
{
	omega = 0.0;
}

void ThrusterSimple::calc_wrench(SimState_t /*states*/, Inertial /*inertial*/, Environment_t /*environment*/)
{
    wrenchProp.force.x() = (thrustMax - thrustMin)*inputMotor + thrustMin;
    wrenchProp.torque.x() = -torque_sign()*((torqueMax - torqueMin)*inputMotor + torqueMin);
}

} // namespace propulsion
} // namespace last_letter_lib
