///////////////////////////////////////////////////
// Piston engine model (based upon Zanzoterra 305i)
///////////////////////////////////////////////////

using namespace std;

// Constructor
PistonEng::PistonEng(YAML::Node propConfig, YAML::Node worldConfig):Propulsion(propConfig, worldConfig)
{
	readParametersProp(propConfig);
	omega = omegaMin; // Initialize engine rotational speed
}

// Destructor
PistonEng::~PistonEng()
{
	delete npPoly;
	// delete engPowerPoly; //@TODO examine if I need to uncomment this
	delete propPowerPoly;
}

void PistonEng::readParametersProp(YAML::Node config)
{
	Propulsion::readParametersProp(config);

	vector<double> doubleVect;
	getParameter(config, "propDiam", propDiam);
	getParameter(config, "engInertia", engInertia);
	getParameter(config, "RadPSLimits", doubleVect);
	// // Initialize RadPS limits
	omegaMin = doubleVect[0];
	omegaMax = doubleVect[1];

	// Create engine power polynomial
	YAML::Node engPowerPolyConfig = filterConfig(config, "engPowerPoly");
	engPowerPoly =  buildPolynomial(engPowerPolyConfig);
	// Create propeller efficiency polynomial
	YAML::Node engCoeffPolyConfig = filterConfig(config, "engCoeffPoly");
	npPoly =  buildPolynomial(engCoeffPolyConfig);
	// Create propeller power polynomial
	YAML::Node propPowerPolyConfig = filterConfig(config, "propPowerPoly");
	propPowerPoly =  buildPolynomial(propPowerPolyConfig);
}

// Update motor rotational speed and calculate thrust
void PistonEng::updateRadPS(SimState_t /* states */, Inertial_t inertial, Environment_t environment)
{
	rho = environment.density; // Read current air density

	// Calculate current engine power
	double powerHP = engPowerPoly->evaluate(omega/2.0/M_PI*60);
	double engPower = inputMotor * powerHP * 745.7;

	double forceX, forceY, forceZ;
	double torqueX, torqueY, torqueZ;

	double advRatio = normalWind/ (omega/2.0/M_PI) /propDiam; // Convert advance ratio to dimensionless units, not 1/rad
	advRatio = std::max(advRatio, 0.0); // Force advance ratio above zero, in lack of a better propeller model
	double propPower = propPowerPoly->evaluate(advRatio) * rho * pow(omega/2.0/M_PI,3) * pow(propDiam,5);
	double npCoeff = npPoly->evaluate(advRatio);
	forceX = propPower*std::fabs(npCoeff/(normalWind+1.0e-10)); // Added epsilon for numerical stability

	double fadeFactor = (exp(-normalWind*3/12));
	double staticThrust = 0.9*fadeFactor*pow(M_PI/2.0*propDiam*propDiam*rho*engPower*engPower,1.0/3); //static thrust fades at 5% at 12m/s
	forceX += staticThrust;

	// Constrain propeller force to +-2 times the aircraft weight
	forceX = std::max(std::min(forceX, 2.0*inertial.mass*9.81), -2.0*inertial.mass*9.81);
	torqueX = propPower / omega;
	if (inputMotor < 0.01) {
		forceX = 0;
		torqueX = 0;
	} // To avoid aircraft rolling and turning on the ground while throttle is off
	torqueY = 0.0;
	torqueZ = 0.0;

	wrenchProp.torque = Vector3d(forceX, forceY, forceZ);
	wrenchProp.torque = Vector3d(torqueX, torqueY, torqueZ);

	double deltaT = (engPower - propPower)/omega;
	double omegaDot = 1/engInertia*deltaT;
	omega += rotationDir * omegaDot * dt;
	omega = rotationDir * std::max(std::min(std::fabs(omega), omegaMax), omegaMin); // Constrain omega to working range

	//TODO: Decide where this should be output
	// parentObj->states.rotorspeed[0]=std::fabs(omega); // Write engine speed to states message

}

void PistonEng::getForce(SimState_t /* states */, Inertial_t /* inertial */, Environment_t /* environment */)
{
	if (!wrenchProp.force.allFinite()) {throw runtime_error("propulsion.cpp: State NaN in wrenchProp.force");}
}

void PistonEng::getTorque(SimState_t /* states */, Inertial_t /* inertial */, Environment_t /* environment */)
{
	if (!wrenchProp.torque.allFinite()) {throw runtime_error("propulsion.cpp: State NaN in wrenchProp.torque");}
}
