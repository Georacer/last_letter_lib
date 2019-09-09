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
	getParameter(configs.world, "deltaT", dt);
	// TODO: Make sure to add this functionality to the ROS wrapper
	// tprev = ros::Time::now();
	// states.header.stamp = tprev;
	//Subscribe and advertize
	// subInp = n.subscribe("ctrlPWM",1,&UavModel::getInput, this); //model control input subscriber
	// subEnv = n.subscribe("environment",1,&UavModel::getEnvironment, this); //dynamic environment effects subscriber
	// pubState = n.advertise<last_letter_msgs::SimStates>("states",1000); //model states publisher
	// pubForce = n.advertise<geometry_msgs::Vector3>("forceInput",1000); // forces publisher
	// pubTorque = n.advertise<geometry_msgs::Vector3>("torqueInput",1000); // torques publisher
	// pubLinAcc = n.advertise<geometry_msgs::Vector3>("linearAcc",1000); // Body frame linear acceleration - no corriolis effect

	init(configs.init);
}

//Initialize states
void UavModel::init(YAML::Node initConfig)
{
	// Set states message frame name
	// TODO port this to ROS wrapper
	// states.header.frame_id = "bodyFrame";

	// Read initial NED coordinates
	vector<double> doubleVect;
	getParameterList(initConfig, "position", doubleVect);
	state.pose.position = Vector3d(doubleVect.data());

	// Read initial orientation quaternion
	doubleVect.clear();
	getParameterList(initConfig, "orientation", doubleVect);
	state.pose.orientation = Quaterniond(doubleVect[3], doubleVect[0], doubleVect[1], doubleVect[2]);

	// Read initial velocity
	doubleVect.clear();
	getParameterList(initConfig, "velLin", doubleVect);
	state.velocity.linear = Vector3d(doubleVect.data());

	// Read initial angular velocity
	doubleVect.clear();
	getParameterList(initConfig, "velAng", doubleVect);
	state.velocity.angular = Vector3d(doubleVect.data());

	// Initialize rotorspeed array
	state.rotorspeed.fill(0.0);
	// And propulsion model omega
	for (int i=0; i<dynamics.nMotors; i++) {
		dynamics.propulsion[i]->omega = 0.01; // A small non-zero value
	}

	// Initialize WGS coordinates
	doubleVect.clear();
	getParameterList(initConfig, "coordinates", doubleVect);
	state.geoid.latitude = doubleVect[0];
	state.geoid.longitude = doubleVect[1];
	state.geoid.altitude = doubleVect[2] - state.pose.position.z(); // Read ground geoid altitude and raise the WGS coordinate by the NED altitude

	// Initialize input
	input.value.fill(0.0);
	doubleVect.clear();
	if (getParameterList(initConfig, "ctrlInput", doubleVect))
	{
		std::copy(doubleVect.begin(), doubleVect.end(), input.value.begin());
	}
	setInput(input);

	// if(!ros::param::getCached("init/chanReset", chanReset)) {ROS_INFO("No RESET channel selected"); chanReset=-1;}
	if (!getParameter(initConfig, "chanReset", chanReset, false)) {cout << "No RESET channel selected" << endl; chanReset=-1;}
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

	state = newState;

	// TODO: Port these to ROS wrapper
	// tprev = ros::Time::now();
	// states.header.stamp = tprev;

	//publish results
	// pubState.publish(states);
	// pubForce.publish(kinematics.forceInput);
	// pubTorque.publish(kinematics.torqueInput);
	// pubLinAcc.publish(kinematics.linearAcc);
}

/////////////////////////////////////////////////
// Pass control inputs to sub-models
void UavModel::setInput(Input_t p_input)
{
	input = p_input;
	dynamics.setInput(input);

	if (chanReset>-1) { // If a reset channel is set
		if (input.value[chanReset] > 0.6) { // Reset the simulation upon command
			init(configs.init);
		}
	}
}

/////////////////////////////////////////////////
//convert uS PWM values to control surface inputs
void UavModel::setInputPwm(InputPwm_t p_input)
{
	PwmInput = p_input;
	dynamics.setInputPwm(PwmInput);

	if (chanReset>-1) { // If a reset channel is set
		if (PwmInput.value[chanReset] > 1600) { // Reset the simulation upon PWM command
			init(configs.init);
		}
	}
}

//////////////////
//Class destructor
UavModel::~UavModel ()
{
}
