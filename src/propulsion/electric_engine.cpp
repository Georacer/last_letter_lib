////////////////////////
// Electric engine model
////////////////////////

using namespace std;

// Constructor
ElectricEng::ElectricEng(ParameterManager propConfig, ParameterManager worldConfig):Propulsion(propConfig, worldConfig)
{
	readParametersProp(propConfig);
	// TODO: Make sure to add this to the ROS wrapper
	// sprintf(paramMsg, "propulsion%i", id);
	// ros::NodeHandle n;
	// pub = n.advertise<last_letter_msgs::ElectricEng>(paramMsg, 1000); //propulsion data publisher
	omega = omegaMin; // Initialize engine rotational speed
}

// Destructor
ElectricEng::~ElectricEng()
{
	delete npPoly;
	delete propPowerPoly;
}

void ElectricEng::readParametersProp(ParameterManager config)
{
	Propulsion::readParametersProp(config);

	vector<double> doubleVect;
	propDiam = config.get<double>("propDiam");
	engInertia = config.get<double>("engInertia");
	Kv = config.get<double>("Kv");
	Rm = config.get<double>("Rm");
	Rs = config.get<double>("Rs");
	Cells = config.get<double>("Cells");
	I0 = config.get<double>("I0");
	doubleVect = config.get<vector<double>>("RadPSLimits");
	omegaMin = doubleVect[0];
	omegaMax = doubleVect[1];

	// Create propeller efficiency polynomial
	ParameterManager nCoeffPolyConfig = config.filter("nCoeffPoly/");
	npPoly =  buildPolynomial(nCoeffPolyConfig);
	// Create propeller power polynomial
	ParameterManager propPowerPolyConfig = config.filter("propPowerPoly/");
	propPowerPoly =  buildPolynomial(propPowerPolyConfig);
}

// Update motor rotational speed and calculate thrust
void ElectricEng::updateRadPS(SimState_t /* states */, Inertial_t inertial, Environment_t environment)
{
	rho = environment.density;

	double Ei = std::fabs(omega)/2/M_PI/Kv;
	double Im = (Cells*4.0*inputMotor - Ei)/(Rs*inputMotor + Rm);
	double engPower = Ei*(Im - I0);

	double advRatio = normalWind / (std::fabs(omega)/2.0/M_PI + 0.001) /propDiam; // Convert advance ratio to dimensionless units, not 1/rad
	double propPower = propPowerPoly->evaluate(advRatio) * rho * pow(std::fabs(omega)/2.0/M_PI,3) * pow(propDiam,5);
	double npCoeff = npPoly->evaluate(advRatio);

	double forceX, forceY, forceZ;
	double torqueX, torqueY, torqueZ;

	forceX = propPower*std::fabs(npCoeff/(normalWind+1.0e-10)); // Added epsilon for numerical stability
	forceY = 0.0;
	forceZ = 0.0;
	double fadeFactor = (exp(-normalWind*3/12));
	double staticThrust = 0.9*fadeFactor*pow(M_PI/2.0*propDiam*propDiam*rho*engPower*engPower,1.0/3); //static thrust fades at 5% at 12m/s
	forceX += staticThrust;

	// Constrain propeller force to [0,+5] times the aircraft weight
	forceX = std::max(std::min(forceX, 5.0*inertial.mass*9.81), 0.0*inertial.mass*9.81);
	torqueX = propPower / omega;
	torqueY = 0.0;
	torqueZ = 0.0;

	if (inputMotor < 0.01) {
		forceX = 0;
		torqueX = 0;
	} // To avoid aircraft rolling and turning on the ground while throttle is off
	wrenchProp.force = Vector3d(forceX, forceY, forceZ);
	wrenchProp.torque = Vector3d(torqueX, torqueY, torqueZ);
	double deltaT = (engPower - propPower)/(std::fabs(omega) + 0.001);
	double omegaDot = 1/engInertia*deltaT;
	omega += rotationDir * omegaDot * dt;
	omega = rotationDir * std::max(std::min(std::fabs(omega), omegaMax), omegaMin); // Constrain omega to working range


	// TODO: Decide where this should be output
	// parentObj->states.rotorspeed[0]=std::fabs(omega); // Write engine speed to states message

}

void ElectricEng::getForce(SimState_t /* states */, Inertial_t /* inertial */, Environment_t /* environment */)
{
	if (!wrenchProp.force.allFinite()) {throw runtime_error("propulsion.cpp: State NaN in wrenchProp.force");}
}

void ElectricEng::getTorque(SimState_t /* states */, Inertial_t /* inertial */, Environment_t /* environment */)
{
	if (!wrenchProp.torque.allFinite()) {throw runtime_error("propulsion.cpp: State NaN in wrenchProp.torque");}
}


// Constructor
ElectricEng2::ElectricEng2(ParameterManager propConfig, ParameterManager worldConfig):Propulsion(propConfig, worldConfig)
{
	readParametersProp(propConfig);
	// TODO: Make sure to add this to the ROS wrapper
	// sprintf(paramMsg, "propulsion%i", id);
	// ros::NodeHandle n;
	// pub = n.advertise<last_letter_msgs::ElectricEng2>(paramMsg, 1000); //propulsion data publisher
	omega = omegaMin; // Initialize engine rotational speed
}

// Destructor
ElectricEng2::~ElectricEng2()
{
	delete propThrustPoly;
	delete propPowerPoly;
}

void ElectricEng2::readParametersProp(ParameterManager config)
{
	Propulsion::readParametersProp(config);

	vector<double> doubleVect;
	propDiam = config.get<double>("propDiam");
	engInertia = config.get<double>("engInertia");
	Kv = config.get<double>("Kv");
	Rm = config.get<double>("Rm");
	Rs = config.get<double>("Rs");
	Cells = config.get<double>("Cells");
	I0 = config.get<double>("I0");
	doubleVect = config.get<vector<double>>("RadPSLimits");
	momentumDragCoeff = config.get<double>("momentumDragCoeff");
	propThrustMultiplier = config.get<double>("propThrustMultiplier");

	omegaMin = doubleVect[0];
	omegaMax = doubleVect[1];

	// Create propeller efficiency polynomial
	ParameterManager thrustCoeffPolyConfig = config.filter("propThrustPoly/");
	propThrustPoly =  buildPolynomial(thrustCoeffPolyConfig);
	// Create propeller power polynomial
	ParameterManager propPowerPolyConfig = config.filter("propPowerPoly/");
	propPowerPoly =  buildPolynomial(propPowerPolyConfig);
}

// Update motor rotational speed and calculate thrust
void ElectricEng2::updateRadPS(SimState_t /* states */, Inertial_t inertial, Environment_t environment)
{
	rho = environment.density;

	double Ei = std::fabs(omega)/2/M_PI/Kv;
	double Im = (Cells*4.0*inputMotor - Ei)/(Rs*inputMotor + Rm);
	double engPower = Ei*(Im - I0);

	double advRatio = normalWind / (std::fabs(omega)/2.0/M_PI + 0.001) /propDiam; // Convert advance ratio to dimensionless units, not 1/rad
	double propPower = propPowerPoly->evaluate(advRatio) * rho * pow(std::fabs(omega)/2.0/M_PI,3) * pow(propDiam,5);
	double propThrust = propThrustPoly->evaluate(advRatio) * rho * pow(std::fabs(omega)/2.0/M_PI,2) * pow(propDiam,4);
	// Constrain propeller force to [0,+5] times the aircraft weight
	propThrust = std::max(std::min(propThrust, 5.0*inertial.mass*9.81), 0.0*inertial.mass*9.81);

	double forceX, forceY, forceZ;
	double torqueX, torqueY, torqueZ;

	forceX = propThrust * propThrustMultiplier;
	forceY = 0.0;
	// Generate momentum drag along the wind direction
	Vector3d sidewind = relativeWind;
	sidewind.x() = 0; // Null out the axial component
	Vector3d momentumDrag = - forceX * sidewind * momentumDragCoeff;
	forceY += momentumDrag.y();
	forceZ += momentumDrag.z();
	// std::cout <<
	// "\nsidewind:\n" << sidewind <<
	// "\nraw force:" <<
	// "\nx: " << forceX <<
	// "\ny: " << forceY <<
	// "\nz: " << forceZ <<
	// std::endl;

	torqueX = propPower / omega;
	torqueY = 0.0;
	torqueZ = 0.0;

	if (inputMotor < 0.01) {
		forceX = 0;
		forceY = 0;
		forceZ = 0;
		torqueX = 0;
	} // To avoid aircraft rolling and turning on the ground while throttle is off
	wrenchProp.force = Vector3d(forceX, forceY, forceZ);
	wrenchProp.torque = Vector3d(torqueX, torqueY, torqueZ);
	double deltaT = (engPower - propPower)/(std::fabs(omega) + 0.001);
	double omegaDot = 1/engInertia*deltaT;
	omega += rotationDir * omegaDot * dt;
	omega = rotationDir * std::max(std::min(std::fabs(omega), omegaMax), omegaMin); // Constrain omega to working range
	// std::cout <<
	// "\nforce:\n" << wrenchProp.force <<
	// "\nomegadot: " << omegaDot <<
	// "\nomega: " << omega << std::endl;
}

void ElectricEng2::getForce(SimState_t /* states */, Inertial_t /* inertial */, Environment_t /* environment */)
{
	if (!wrenchProp.force.allFinite()) {throw runtime_error("propulsion.cpp: State NaN in wrenchProp.force");}
}

void ElectricEng2::getTorque(SimState_t /* states */, Inertial_t /* inertial */, Environment_t /* environment */)
{
	if (!wrenchProp.torque.allFinite()) {throw runtime_error("propulsion.cpp: State NaN in wrenchProp.torque");}
}
