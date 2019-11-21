////////////////////////
// Electric engine model
////////////////////////

using namespace std;

// Constructor
ElectricEng::ElectricEng(YAML::Node propConfig, YAML::Node worldConfig):Propulsion(propConfig, worldConfig)
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

void ElectricEng::readParametersProp(YAML::Node config)
{
	Propulsion::readParametersProp(config);

	vector<double> doubleVect;
	getParameter(config, "propDiam", propDiam);
	getParameter(config, "engInertia", engInertia);
	getParameter(config, "Kv", Kv);
	getParameter(config, "Rm", Rm);
	getParameter(config, "Rs", Rs);
	getParameter(config, "Cells", Cells);
	getParameter(config, "I0", I0);
	getParameterList(config, "RadPSLimits", doubleVect);
	omegaMin = doubleVect[0];
	omegaMax = doubleVect[1];

	// Create propeller efficiency polynomial
	YAML::Node nCoeffPolyConfig = filterConfig(config, "nCoeffPoly/");
	npPoly =  buildPolynomial(nCoeffPolyConfig);
	// Create propeller power polynomial
	YAML::Node propPowerPolyConfig = filterConfig(config, "propPowerPoly/");
	propPowerPoly =  buildPolynomial(propPowerPolyConfig);
}

// Update motor rotational speed and calculate thrust
void ElectricEng::updateRadPS(SimState_t states, Inertial_t inertial, Environment_t environment)
{
	rho = environment.density;

	double Ei = std::fabs(omega)/2/M_PI/Kv;
	double Im = (Cells*4.0*inputMotor - Ei)/(Rs*inputMotor + Rm);
	double engPower = Ei*(Im - I0);

	double advRatio = normalWind / (std::fabs(omega)/2.0/M_PI) /propDiam; // Convert advance ratio to dimensionless units, not 1/rad
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

	if (inputMotor < 0.01) {
		forceX = 0;
		torqueX = 0;
	} // To avoid aircraft rolling and turning on the ground while throttle is off
	wrenchProp.force = Vector3d(forceX, forceY, forceZ);
	wrenchProp.torque = Vector3d(torqueX, torqueY, torqueZ);
	double deltaT = (engPower - propPower)/std::fabs(omega);
	double omegaDot = 1/engInertia*deltaT;
	omega += rotationDir * omegaDot * dt;
	omega = rotationDir * std::max(std::min(std::fabs(omega), omegaMax), omegaMin); // Constrain omega to working range


	// TODO: Decide where this should be output
	// parentObj->states.rotorspeed[0]=std::fabs(omega); // Write engine speed to states message

	// message.header.stamp = ros::Time::now();
	// message.powerEng = propPower;
	// message.omega = omega;
	// message.throttle = inputMotor*100.0;
	// message.powerProp = propPower;
	// message.thrust = wrenchProp.force.x;
	// message.torque = wrenchProp.torque.x;
	// message.advRatio = advRatio;
	// message.airspeed = normalWind;
	// message.ncoeff = npCoeff;
	// pub.publish(message);

}

void ElectricEng::getForce(SimState_t states, Inertial_t inertial, Environment_t environment)
{
	if (!wrenchProp.force.allFinite()) {throw runtime_error("propulsion.cpp: State NaN in wrenchProp.force");}
}

void ElectricEng::getTorque(SimState_t states, Inertial_t inertial, Environment_t environment)
{
	if (!wrenchProp.torque.allFinite()) {throw runtime_error("propulsion.cpp: State NaN in wrenchProp.torque");}
}