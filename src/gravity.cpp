#include "gravity.hpp"

//////////////////////////
// Define Gravity class
//////////////////////////

// Class constructor
Gravity::Gravity()
{
	wrenchGrav.force = Vector3d::Zero();
	wrenchGrav.torque = Vector3d::Zero();
}

// Class destructor
Gravity::~Gravity()
{
}

// Force calculation function
Vector3d Gravity::getForce(Quaterniond orientation_eb, double g, double mass)
{
	if (isnan(orientation_eb)) {throw runtime_error("gravity.cpp: NaN member in orientation quaternion");}

	Vector3d gravVect;
	gravVect = Vector3d(0, 0, mass*g);

	wrenchGrav.force = orientation_eb*gravVect;
	if (isnan(wrenchGrav.force)) {throw runtime_error("gravity.cpp: NaN member in force vector");}

	return wrenchGrav.force;
}

// Torque calculation function
Vector3d Gravity::getTorque(Quaterniond orientation_eb, double g, double mass)
{
	// Gravity does not generate torque around the CG
	return wrenchGrav.torque;
}