#include "dynamics.hpp"

//////////////////////////
// Define Dynamics class
//////////////////////////

	///////////////////
	//Class Constructor
	Dynamics::Dynamics(YAML::Node p_worldConfig, YAML::Node p_aeroConfig, YAML::Node p_propConfig, YAML::Node p_groundConfig)
	{
		// Store the configuration nodes
		worldConfig = p_worldConfig;
		groundConfig = p_groundConfig;
		// Create and initialize aerodynamic objects array
		getParameter(p_aeroConfig, "nWings", nWings);
		aerodynamics = new Aerodynamics*[nWings];
		for (int i=0; i<nWings; i++) {
			YAML::Node aeroConfig = filterConfig(p_aeroConfig, "airfoil"+std::to_string(i+1)+"/");
			aeroConfigVec.push_back(aeroConfig);
			aerodynamics[i] = buildAerodynamics(aeroConfigVec[i]); // Create a new aerodynamics object, id's are 1-indexed
		}

		// Create and initialize gravity object
		gravity = new Gravity();

		// Create and initialize motor objects array
		getParameter(p_propConfig, "nMotors", nMotors);
		propulsion = new Propulsion*[nMotors];
		propConfigVec.reserve(nMotors);	
		for (int i=0; i<nMotors; i++) {
			YAML::Node propConfig = filterConfig(p_propConfig, "motor"+std::to_string(i+1)+"/");
			propConfigVec.push_back(propConfig); 
			propulsion[i] = buildPropulsion(propConfigVec[i], worldConfig); // Create a new propulsion object, id's are 1-indexed
		}

		// Create and initialize ground reactions object
		groundReaction = buildGroundReaction(groundConfig, worldConfig);
	}

	//Class Destructor
	Dynamics::~Dynamics()
	{
		for (int i=0; i<nWings; i++){
			delete aerodynamics[i]; // delete all aerodynamics objects
		}
		delete gravity;
		for (int i=0; i<nMotors; i++){
			delete propulsion[i]; // delete all propulsion objects
		}
		delete propulsion; // Must also separately delete the array of object pointers
		delete groundReaction;
	}

	// Order subsystems to store control input
	void Dynamics::setInput(Input_t input)
	{
		for (int i=0; i<nMotors; i++) {
			propulsion[i]->setInput(input, propConfigVec[i]);
		}
		for (int i=0; i<nWings; i++) {
			aerodynamics[i]->setInput(input, aeroConfigVec[i]);
		}
		groundReaction->setInput(input, groundConfig);
	}

	// Order subsystems to store control input, passed as PWM micorseconds
	void Dynamics::setInputPwm(InputPwm_t input)
	{
		for (int i=0; i<nMotors; i++) {
			propulsion[i]->setInputPwm(input, propConfigVec[i]);
		}
		for (int i=0; i<nWings; i++) {
			aerodynamics[i]->setInputPwm(input, aeroConfigVec[i]);
		}
		groundReaction->setInputPwm(input, groundConfig);
	}

	// Calculate the forces and torques for each Wrench_t source
	void Dynamics::calcWrench(SimState_t states, Inertial_t inertial, Environment_t environment)
	{
		Vector3d tempVect;

		// Call gravity calculation routines
		forceGrav = gravity->getForce(states.pose.orientation, environment.gravity, inertial.mass);
		if (!forceGrav.allFinite()) {throw runtime_error("dynamicsLib.cpp: NaN member in gravity force vector");}

		torqueGrav = gravity->getTorque(states.pose.orientation, environment.gravity, inertial.mass);
		if (!torqueGrav.allFinite()) {throw runtime_error("dynamicsLib.cpp: NaN member in gravity torque vector");}

		// Call  motors routines

		// Execute one step in the motor dynamics
		for (int i=0; i<nMotors; i++) {
			propulsion[i]->stepEngine(states, inertial, environment);
		}

		forceProp.setZero();	
		for (int i=0; i<nMotors; i++){
			if (!propulsion[i]->wrenchProp.force.allFinite())
			{
				throw runtime_error("dynamicsLib.cpp: NaN member in propulsion"+std::to_string(i+1)+" force vector");
			}
			forceProp += propulsion[i]->wrenchProp.force;
		}

		torqueProp.setZero();
		for (int i=0; i<nMotors; i++){
			if (!propulsion[i]->wrenchProp.torque.allFinite())
			{
				throw runtime_error("dynamicsLib.cpp: NaN member in propulsion"+std::to_string(i+1)+" torque vector");
			}
			torqueProp += propulsion[i]->wrenchProp.torque;
		}

		// Call  aerodynamics routines

		// Execute one step in the aerodynamics
		for (int i=0; i<nWings; i++) {
			aerodynamics[i]->stepDynamics(states, inertial, environment);
		}

		forceAero.setZero();
		for (int i=0; i<nWings; i++){
			if (!aerodynamics[i]->wrenchAero.force.allFinite())
			{
				Vector3d tempVect(aerodynamics[i]->wrenchAero.force);
				cout << "Resulting aerodynamics force:\n" << tempVect << endl;
				throw runtime_error("dynamicsLib.cpp: NaN member in airfoil"+std::to_string(i+1)+" force vector");
			}
			forceAero += aerodynamics[i]->wrenchAero.force;
		}

		torqueAero.setZero();
		for (int i=0; i<nWings; i++){
			if (!aerodynamics[i]->wrenchAero.torque.allFinite())
			{
				throw runtime_error("dynamicsLib.cpp: NaN member in airfoil"+std::to_string(i+1)+" torque vector");
			}
			torqueAero += aerodynamics[i]->wrenchAero.torque;
		}

		// Call ground reactions routines - MUST BE CALLED LAST!!!
		// This is needed for some ground reactions models, which are designed to counter the remaining force sum
		WrenchSum_t wrenchSum;
		wrenchSum.wrenchAero.force = forceAero;
		wrenchSum.wrenchAero.torque = torqueAero;
		wrenchSum.wrenchProp.force = forceProp;
		wrenchSum.wrenchProp.torque = torqueProp;
		wrenchSum.wrenchGrav.force = forceGrav;
		wrenchSum.wrenchGrav.torque = torqueGrav;
		forceGround = groundReaction->getForce(states, wrenchSum);
		if (!forceGround.allFinite()) {throw runtime_error("dynamicsLib.cpp: NaN member in groundReaction force vector");}

		torqueGround = groundReaction->getTorque(states, wrenchSum);
		if (!torqueGround.allFinite()) {throw runtime_error("dynamicsLib.cpp: NaN member in groundReaction torque vector");}

	}

	// Collect forces from underlying models
	Vector3d Dynamics::getForce()
	{
		Vector3d accumulator;

		accumulator = forceGrav;
		accumulator = accumulator + forceProp;
		accumulator = accumulator + forceAero;
		accumulator = accumulator + forceGround;

		return accumulator;
	}

	// Collect torques from underlying models
	Vector3d Dynamics::getTorque()
	{
		Vector3d accumulator;

		accumulator = torqueGrav;
		accumulator = accumulator + torqueProp;
		accumulator = accumulator + torqueAero;
		accumulator = accumulator + torqueGround;

		return accumulator;
	}

