////////////////////////////////////////
// Engine model found in R. Beard's book
////////////////////////////////////////

// Constructor
EngBeard::EngBeard(ParameterManager propConfig, ParameterManager worldConfig) : Propulsion(propConfig, worldConfig)
{
	std::cout << "Building new Beard Engine" << std::endl;
	omega = 0; // Initialize engine rotational speed
	readParametersProp(propConfig);
}

// Destructor
EngBeard::~EngBeard()
{
}

void EngBeard::readParametersProp(ParameterManager config)
{
	Propulsion::readParametersProp(config);

	s_prop = config.get<double>("s_prop");
	c_prop = config.get<double>("c_prop");
	k_motor = config.get<double>("k_motor");
	k_t_p = config.get<double>("k_t_p");
	k_omega = config.get<double>("k_omega");
}

// Update motor rotational speed and other states for each timestep
void EngBeard::updateRadPS(SimState_t /* states */, Inertial_t /* inertial */, Environment_t /*environment*/)
{
	omega = rotationDir * inputMotor * k_omega;
	// parentObj->states.rotorspeed[0]=std::fabs(omega); // Write engine speed to states message
}

// Calculate propulsion forces
void EngBeard::getForce(SimState_t /* states */, Inertial_t /* inertial */, Environment_t environment)
{
	rho = environment.density;
	double x, y, z;
	x = 1.0 / 2.0 * rho * s_prop * c_prop * (pow(inputMotor * k_motor, 2) - pow(normalWind, 2));
	// cap x above zero
	x = std::max(0.0, x);
	y = 0;
	z = 0;
	// std::cout << "Thrust force calculation:\n"
	// 		  << "rho" << rho << "\n"
	// 		  << "s_prop" << s_prop << "\n"
	// 		  << "c_prop" << c_prop << "\n"
	// 		  << "inputMotor" << inputMotor << "\n"
	// 		  << "k_motor" << k_motor << "\n"
	// 		  << "airspeed" << normalWind << "\n"
	// 		  << std::endl;
	wrenchProp.force = Vector3d(x, y, z);
	if (!wrenchProp.force.allFinite())
	{
		throw runtime_error("propulsion.cpp/EngBeard: State NaN in wrenchProp.force");
	}
}

// Calculate propulsion torques
void EngBeard::getTorque(SimState_t /* states */, Inertial_t /* inertial */, Environment_t /* environment */)
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
