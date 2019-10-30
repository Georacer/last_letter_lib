#include "kinematics.hpp"

//////////////////////////
// Define Kinematics class
//////////////////////////

///////////////////
//Class Constructor
Kinematics::Kinematics(YAML::Node inertialConfig, YAML::Node worldConfig)
{
	readParametersInertial(inertialConfig);
	readParametersWorld(worldConfig);

	// Build the integrator object
	integrator = buildIntegrator(worldConfig);
}

//////////////////
//Class Destructor
Kinematics::~Kinematics()
{
	delete integrator;
}

////////////////////////////////////////
// Read and register world parameters
void Kinematics::readParametersWorld(YAML::Node worldConfig)
{
	getParameter(worldConfig, "deltaT", dt);
}

////////////////////////////////////////
// Read and register inertial parameters
void Kinematics::readParametersInertial(YAML::Node inertialConfig)
{
	// Calculate inertia matrix...
    getParameter(inertialConfig, "m", inertial.mass);
    double j_x, j_y, j_z, j_xz;
    getParameter(inertialConfig, "j_x", j_x);
    getParameter(inertialConfig, "j_y", j_y);
    getParameter(inertialConfig, "j_z", j_z);
    getParameter(inertialConfig, "j_xz", j_xz);
    inertial.J << j_x, 0, -j_xz,
                    0, j_y, 0,
                    -j_xz, 0, j_z;

	int res = is_pos_def(inertial.J.data());
	if (!(res==0)) {
		switch (res) {
			case -1:
				throw runtime_error("Matrix of inertia is singular");
				break;
			case -2:
				throw runtime_error("Matrix of inertia is not positive definite");
				break;
			default:
				break;
		}

	}

	// ... and its inverse
	inertial.Jinv = inertial.J.inverse();
}

///////////////////////////////
//State derivatives calculation
void Kinematics::calcDerivatives(SimState_t states, Wrench_t inpWrench)
{
	// variable declaration
	// create position derivatives from earth velocity
	stateDot.posDot = states.pose.orientation * states.velocity.linear;
	if (!stateDot.posDot.allFinite()) {throw runtime_error("NaN member in position derivative vector");}

	// create body velocity derivatives from acceleration, angular rotation and body velocity
	Vector3d linearAcc = (1.0/inertial.mass)*inpWrench.force;
	if (!linearAcc.allFinite()) {throw runtime_error("NaN member in linear acceleration vector");}
	Vector3d corriolisAcc = -states.velocity.angular.cross(states.velocity.linear);
	if (!corriolisAcc.allFinite()) {throw runtime_error("NaN member in corriolis acceleration vector");}
	stateDot.speedDot = linearAcc + corriolisAcc;

	// create angular derivatives quaternion from angular rates
	// ATTENTION! This quaternion derivative equation refers to the body-to-earth quaternion
	double x, y, z, w;
	x = states.velocity.angular.x()*0.5*dt;
	y = states.velocity.angular.y()*0.5*dt;
	z = states.velocity.angular.z()*0.5*dt;
	w = 1.0;
	stateDot.quatDot = Quaterniond(w, x, y, z);
	if (!myisfinite(stateDot.quatDot)) {throw runtime_error("NaN member in quaternion derivative vector");}

	// create angular rate derivatives from torque
	stateDot.rateDot = inertial.Jinv * (inpWrench.torque - states.velocity.angular.cross(inertial.J*states.velocity.angular));
	if (!stateDot.rateDot.allFinite()) {throw runtime_error("NaN member in angular velocity derivative vector");}

	stateDot.coordDot.x() = 180.0/M_PI*asin(stateDot.posDot.x() / WGS84_RM(states.geoid.latitude));
	stateDot.coordDot.y() = 180.0/M_PI*asin(stateDot.posDot.y() / WGS84_RN(states.geoid.longitude));
	stateDot.coordDot.z() = - stateDot.posDot.z();
}

//////////////////////////
// Propagate the UAV state
SimState_t Kinematics::propagateState(SimState_t states, Wrench_t inpWrench)
{
	calcDerivatives(states, inpWrench);
	return integrator->propagation(states, stateDot);
}


//////////////////////////
// Define Integrator class
//////////////////////////

Integrator::Integrator(YAML::Node worldConfig)
{
	getParameter(worldConfig, "deltaT", dt);
}

Integrator::~Integrator()
{
}

//////////////////////////
// Define ForwardEuler class
//////////////////////////

//Class Constructor
ForwardEuler::ForwardEuler(YAML::Node worldConfig) : Integrator(worldConfig)
{
}

//Propagation of the states
SimState_t ForwardEuler::propagation(SimState_t states, Derivatives_t derivatives)
{
	SimState_t newStates;

	// Propagate the NED coordinates from earth velocity
	newStates.pose.position = states.pose.position + derivatives.posDot*dt;
	if (!newStates.pose.position.allFinite()) {throw runtime_error("NaN member in position vector");}

	// Propagate orientation quaternion from angular derivatives quaternion
	Quaterniond tempQuat = states.pose.orientation*derivatives.quatDot;
	// Quaterniond tempQuat = states.pose.orientation.conjugate()*derivatives.quatDot;
	newStates.pose.orientation = tempQuat.normalized();
	if (!myisfinite(newStates.pose.orientation)) {throw runtime_error("NaN member in orientation quaternion");}

	// Propagate body velocity from body velocity derivatives
	newStates.velocity.linear = states.velocity.linear + derivatives.speedDot*dt;
	if (!newStates.velocity.linear.allFinite()) {throw runtime_error("NaN member in linear velocity vector");}

	// Propagate angular velocity from angular derivatives
	newStates.velocity.angular = states.velocity.angular + derivatives.rateDot*dt;
	if (!newStates.velocity.angular.allFinite()) {throw runtime_error("NaN member in angular velocity vector");}

	// Set linear acceleration from the speed derivatives
	newStates.acceleration.linear = derivatives.speedDot;

	// Set angular acceleration from the angular rate derivatives
	newStates.acceleration.angular = derivatives.rateDot;

	//Update Geoid stuff using the NED coordinates
	newStates.geoid.latitude = states.geoid.latitude + derivatives.coordDot.x() * dt;
	newStates.geoid.longitude = states.geoid.longitude + derivatives.coordDot.y() * dt;
	newStates.geoid.altitude = states.geoid.altitude + derivatives.coordDot.z() * dt;
	newStates.geoid.velocity[0] = derivatives.posDot.x();
	newStates.geoid.velocity[1] = derivatives.posDot.y();
	newStates.geoid.velocity[2] = -derivatives.posDot.z(); // Upwards velocity

	return newStates;
}

//Build integrator model
Integrator * buildIntegrator(YAML::Node worldConfig)
{
	int integratorType;
	getParameter(worldConfig, "integratorType", integratorType);
	std::cout<< "building integrator class... ";
	switch (integratorType)
	{
	case 0:
		std::cout << "selecting Forward Euler" << std::endl;
		return new ForwardEuler(worldConfig);
	default:
		throw runtime_error("Invalid integrator type!");
		break;
	}
}