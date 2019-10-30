// Core class definitions

#include "uav_model.hpp"

using namespace std;

//////////////////////////
// Define UavModel class
//////////////////////////

///////////////////
//Class Constructor
UavModel::UavModel (ConfigsStruct_t p_configs) :
	kinematics(p_configs.inertial, p_configs.world),
	dynamics(p_configs.world, p_configs.aero, p_configs.prop, p_configs.ground),
	environmentModel(p_configs.env, p_configs.world),
	airdata()
{
	configs = p_configs;

	readParametersWorld(configs.world);
	readParametersInit(configs.init);

	init();
}

//Initialize states
void UavModel::init()
{
	// Read initial NED coordinates
	state.pose.position = initPosition_;

	// Read initial orientation quaternion
	state.pose.orientation = initOrientation_;

	// Read initial velocity
	state.velocity.linear = initVelLinear_;

	// Read initial angular velocity
	state.velocity.angular = initVelAngular_;

	// Initialize rotorspeed vector
	fill(state.rotorspeed.begin(), state.rotorspeed.end(), 0.01);
	// And propulsion model omega
	for (int i=0; i<dynamics.nMotors; i++) {
		dynamics.propulsion[i]->omega = 0.01; // A small non-zero value
	}

	// Initialize WGS coordinates
	state.geoid.latitude = initCoordinates_(0);
	state.geoid.longitude = initCoordinates_(1);
	state.geoid.altitude = initCoordinates_(2);

	chanReset = initChanReset_;

	// Initialize input
	setInput(initCtrlInput_);
}

// Pick and update all sub-models which read from each one
void UavModel::updateConfigWorld()
{
	readParametersWorld(configs.world);
	kinematics.readParametersWorld(configs.world);	
	dynamics.readParametersWorld(configs.world);
	environmentModel.readParametersWorld(configs.world);
}

void UavModel::updateConfigEnvironment()
{
	environmentModel.readParametersEnvironment(configs.env);
}

void UavModel::updateConfigInit()
{
	readParametersInit(configs.init);
}

void UavModel::updateConfigInertial()
{
	kinematics.readParametersInertial(configs.inertial);
}

void UavModel::updateConfigAero(){
	dynamics.readParametersAerodynamics(configs.aero);
}

void UavModel::updateConfigProp()
{
	dynamics.readParametersProp(configs.prop);
}

void UavModel::updateConfigGround()
{
	dynamics.readParametersGround(configs.ground);
}

void UavModel::updateConfigAll()
{
	updateConfigWorld();
	updateConfigEnvironment();
	updateConfigInit();
	updateConfigInertial();
	updateConfigAero();
	updateConfigProp();
	updateConfigGround();
}


void UavModel::readParametersWorld(YAML::Node worldConfig)
{
	getParameter(configs.world, "deltaT", dt);
}

void UavModel::readParametersInit(YAML::Node initConfig)
{
	// Read initial NED coordinates
	vector<double> doubleVect;
	getParameterList(initConfig, "position", doubleVect);
	initPosition_ = Vector3d(doubleVect.data());

	// Read initial orientation quaternion
	doubleVect.clear();
	getParameterList(initConfig, "orientation", doubleVect);
	initOrientation_ = Quaterniond(doubleVect[3], doubleVect[0], doubleVect[1], doubleVect[2]);

	// Read initial velocity
	doubleVect.clear();
	getParameterList(initConfig, "velLin", doubleVect);
	initVelLinear_ = Vector3d(doubleVect.data());

	// Read initial angular velocity
	doubleVect.clear();
	getParameterList(initConfig, "velAng", doubleVect);
	initVelAngular_ = Vector3d(doubleVect.data());

	// Initialize WGS coordinates
	doubleVect.clear();
	getParameterList(initConfig, "coordinates", doubleVect);
	initCoordinates_(0) = doubleVect[0];
	initCoordinates_(1) = doubleVect[1];
	initCoordinates_(2) = doubleVect[2] - initPosition_.z(); // Read ground geoid altitude and raise the WGS coordinate by the NED altitude

	if (!getParameter(initConfig, "chanReset", initChanReset_, false)) {cout << "No RESET channel selected" << endl; initChanReset_=-1;}

	// Initialize input
	doubleVect.clear();
	if (getParameterList(initConfig, "ctrlInput", doubleVect))
	{
		std::copy(doubleVect.begin(), doubleVect.end(), initCtrlInput_.value.begin());
	}
}

///////////////////////////////////////
//Make one step of the plane simulation
void UavModel::step(void)
{
	// Perform step actions serially

	// TODO: read the updated propeller velocities and store them (if needed)

	airdata.calcAirData(state.velocity.linear, environmentModel.environment.wind);

	environmentModel.calcEnvironment(state);
	dynamics.calcWrench(state, kinematics.inertial, environmentModel.environment);
	Wrench_t inpWrench;
	inpWrench.force = dynamics.getForce();
	inpWrench.torque = dynamics.getTorque();

	SimState_t newState;
	newState = kinematics.propagateState(state, inpWrench);

	// Check new state for non-finite values
	Vector3d tempVect;
	Quaterniond tempQuat;
	if (!newState.pose.position.allFinite())
	{
		tempVect = newState.pose.position;
		cout << "New position:\n" << tempVect << endl;
		throw runtime_error("uav_model.cpp: NaN member in new position");
	}
	if (!myisfinite(newState.pose.orientation))
	{
		tempQuat = newState.pose.orientation;
		cout << "New orientation:\n" << tempQuat.w() << "\n" << tempQuat.vec() << endl;
		throw runtime_error("uav_model.cpp: NaN member in new orientation");
	}
	if (!newState.velocity.linear.allFinite())
	{
		tempVect = newState.velocity.linear;
		cout << "New linear velocity:\n" << tempVect << endl;
		throw runtime_error("uav_model.cpp: NaN member in new linear velocity");
	}
	if (!newState.velocity.angular.allFinite())
	{
		tempVect = newState.velocity.angular;
		cout << "New angular velocity:\n" << tempVect << endl;
		throw runtime_error("uav_model.cpp: NaN member in new angular velocity");
	}

	for (int i=0; i<dynamics.nMotors; i++)
	{
		newState.rotorspeed[i] = dynamics.propulsion[i]->omega;
	}

	state = newState;
}

////////////////////////
// Set state explicitly
void UavModel::setState(const SimState_t p_state)
{
	state = p_state;
}

/////////////////////////////////////////////////
// Pass control inputs to sub-models
void UavModel::setInput(const Input_t p_input)
{
	input.value = p_input.value;
	dynamics.setInput(input);

	if (chanReset>-1) { // If a reset channel is set
		if (input.value[chanReset] > 0.6) { // Reset the simulation upon command
			init();
		}
	}
}

/////////////////////////////////////////////////
//convert uS PWM values to control surface inputs
void UavModel::setInputPwm(const InputPwm_t p_input)
{
	PwmInput.value = p_input.value;
	dynamics.setInputPwm(PwmInput);

	if (chanReset>-1) { // If a reset channel is set
		if (PwmInput.value[chanReset] > 1600) { // Reset the simulation upon PWM command
			init();
		}
	}
}

//////////////////
//Class destructor
UavModel::~UavModel ()
{
}
