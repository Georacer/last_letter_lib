////////////////////////////////////////
// Engine model found in R. Beard's book
////////////////////////////////////////

// Constructor
EngBeard::EngBeard(YAML::Node propConfig, YAML::Node worldConfig):Propulsion(propConfig, worldConfig)
{
	std::cout << "reading parameters for new Beard engine" << std::endl;
	omega = 0; // Initialize engine rotational speed
	// Read engine data from parameter server
	getParameter(propConfig, "s_prop", s_prop);
	getParameter(propConfig, "c_prop", c_prop);
	getParameter(propConfig, "k_motor", k_motor);
	getParameter(propConfig, "k_t_p", k_t_p);
	getParameter(propConfig, "k_omega", k_omega);
}

// Destructor
EngBeard::~EngBeard()
{
}

// Update motor rotational speed and other states for each timestep
void EngBeard::updateRadPS(SimState_t states, Inertial_t inertial, Environment_t environment)
{
	rho = environment.density;
	airspeed = normalWind; // Read vehicle airspeed
	if (!std::isfinite(airspeed)) {throw runtime_error("propulsion.cpp: EngBeard airspeed is not finite");}
	omega = rotationDir * inputMotor * k_omega;
	// parentObj->states.rotorspeed[0]=std::fabs(omega); // Write engine speed to states message
}

// Calculate propulsion forces
void EngBeard::getForce(SimState_t states, Inertial_t inertial, Environment_t environment)
{
	double x, y, z;
	x = 1.0/2.0*rho*s_prop*c_prop*(pow(inputMotor * k_motor,2)-pow(airspeed,2));
	y = 0;
	z = 0;
	wrenchProp.force = Vector3d(x, y, z);
	if (!wrenchProp.force.allFinite()) {throw runtime_error("propulsion.cpp/EngBeard: State NaN in wrenchProp.force");}
}

// Calculate propulsion torques
void EngBeard::getTorque(SimState_t states, Inertial_t inertial, Environment_t environment)
{
	double x, y, z;
	x = -rotationDir * k_t_p*pow(omega,2);
	y = 0;
	z = 0;
	wrenchProp.torque = Vector3d(x, y, z);
	if (!wrenchProp.torque.allFinite()) {throw runtime_error("propulsion.cpp/EngBeard: State NaN in wrenchProp.torque");}
}