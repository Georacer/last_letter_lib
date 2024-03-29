////////////////////////////////////////
// Engine model found in R. Beard's book
////////////////////////////////////////

// Constructor
EngOmegaControl::EngOmegaControl(ParameterManager propConfig, ParameterManager worldConfig):Propulsion(propConfig, worldConfig)
{
	std::cout << "Building new Controlled-Omega Engine" << std::endl;;
	omega = 0; // Initialize engine rotational speed
	readParametersProp(propConfig);
}

// Destructor
EngOmegaControl::~EngOmegaControl()
{
}

void EngOmegaControl::readParametersProp(ParameterManager config)
{
	Propulsion::readParametersProp(config);

	prop_diam = config.get<double>("propDiam");
	omega_max = config.get<double>("omega_max");
	// Create propeller thrust polynomial
	ParameterManager thrustPolyConfig = config.filter("thrust_poly/");
	thrust_poly =  buildPolynomial(thrustPolyConfig);
	// Create propeller power polynomial
	ParameterManager powerPolyConfig = config.filter("power_poly/");
	power_poly =  buildPolynomial(powerPolyConfig);
}

// Update motor rotational speed and other states for each timestep
void EngOmegaControl::updateRadPS(SimState_t /* states */, Inertial_t /* inertial */, Environment_t environment)
{
	omega = inputMotor*omega_max; // Direct omega setting from user
	rho = environment.density;
	double eps = 1e-4;

	double advRatio = normalWind / (std::fabs(omega)/2.0/M_PI + eps) / prop_diam; // Convert advance ratio to dimensionless units, not 1/rad
	torque = power_poly->evaluate(advRatio) /2/M_PI * rho * pow(std::fabs(omega)/2.0/M_PI,2) * pow(prop_diam,5);
	thrust = thrust_poly->evaluate(advRatio) * rho * pow(std::fabs(omega)/2.0/M_PI,2) * pow(prop_diam,4);
}

// Calculate propulsion forces
void EngOmegaControl::getForce(SimState_t /* states */, Inertial_t /* inertial */, Environment_t /* environment */)
{
	double x, y, z;
	x = thrust;
	y = 0;
	z = 0;
	wrenchProp.force = Vector3d(x, y, z);
	if (!wrenchProp.force.allFinite()) {throw runtime_error("propulsion.cpp/EngOmegaControl: State NaN in wrenchProp.force");}
}

// Calculate propulsion torques
void EngOmegaControl::getTorque(SimState_t /* states */, Inertial_t /* inertial */, Environment_t /* environment */)
{
	double x, y, z;
	x = -rotationDir * torque;
	y = 0;
	z = 0;
	wrenchProp.torque = Vector3d(x, y, z);
	if (!wrenchProp.torque.allFinite()) {throw runtime_error("propulsion.cpp/EngOmegaControl: State NaN in wrenchProp.torque");}
}
