// Core class definitions
#include <stdexcept>

#include "last_letter_lib/uav_model.hpp"
#include "last_letter_lib/prog_utils.hpp"

using namespace std;
using namespace last_letter_lib::programming_utils;

namespace last_letter_lib
{

	//////////////////////////
	// Define UavModel class
	//////////////////////////

	///////////////////
	// Class Constructor
	UavModel::UavModel(ParameterManager p_configs) : configs(p_configs),
													 kinematics(p_configs.filter("inertial"),
																p_configs.filter("world")),
													 dynamics(p_configs.filter("world"),
															  p_configs.filter("aero"),
															  p_configs.filter("prop"),
															  p_configs.filter("ground")),
													 environmentModel(p_configs.filter("env"), p_configs.filter("world"))
	{

		readParametersWorld(configs.filter("world"));
		readParametersInit(configs.filter("init"));

		// Add sensors to the UAV
		sensors.push_back(std::make_shared<Imu>());
		sensors.push_back(std::make_shared<Barometer>());
		sensors.push_back(std::make_shared<AirdataSensor>());
		sensors.push_back(std::make_shared<Gnss>());
		sensors.push_back(std::make_shared<MavlinkHilStateQuaternion>());
		for (auto sensor_ptr : sensors)
		{
			sensor_ptr->init(p_configs);
		}

		init();
	}

	// Initialize states
	void UavModel::init()
	{
		// TODO: All initialization state is not used, passed by Gazebo
		//  // Read initial NED coordinates
		//  state.pose.position = initPosition_;

		// // Read initial orientation quaternion
		// state.pose.orientation = initOrientation_;

		// // Read initial velocity
		// state.velocity.linear = initVelLinear_;

		// // Read initial angular velocity
		// state.velocity.angular = initVelAngular_;

		// // Initialize rotorspeed vector
		// fill(state.rotorspeed.begin(), state.rotorspeed.end(), 0.01);
		// And propulsion model omega
		for (int i = 0; i < dynamics.nMotors; i++)
		{
			dynamics.propulsionLinks[i]->propulsion->omega = 0.01; // A small non-zero value
		}

		// // Initialize WGS coordinates
		// state.geoid.latitude = initCoordinates_(0);
		// state.geoid.longitude = initCoordinates_(1);
		// state.geoid.altitude = initCoordinates_(2);

		chanReset = initChanReset_;

		// Initialize input
		setInput(initCtrlInput_);
	}

	// Pick and update all sub-models which read from each one
	void UavModel::updateConfigWorld()
	{
		readParametersWorld(configs.filter("world"));
		kinematics.readParametersWorld(configs.filter("world"));
		dynamics.readParametersWorld(configs.filter("world"));
		environmentModel.readParametersWorld(configs.filter("world"));
	}

	void UavModel::updateConfigEnvironment()
	{
		environmentModel.readParametersEnvironment(configs.filter("env"));
	}

	void UavModel::updateConfigInit()
	{
		readParametersInit(configs.filter("init"));
	}

	void UavModel::updateConfigInertial()
	{
		kinematics.readParametersInertial(configs.filter("inertial"));
	}

	void UavModel::updateConfigAero()
	{
		dynamics.readParametersAerodynamics(configs.filter("aero"));
	}

	void UavModel::updateConfigProp()
	{
		dynamics.readParametersProp(configs.filter("prop"));
	}

	void UavModel::updateConfigGround()
	{
		dynamics.readParametersGround(configs.filter("ground"));
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

	void UavModel::readParametersWorld(ParameterManager worldConfig)
	{
		dt = worldConfig.get<double>("deltaT");
	}

	void UavModel::readParametersInit(ParameterManager initConfig)
	{
		// Read initial NED coordinates
		vector<double> doubleVect;
		doubleVect = initConfig.get<vector<double>>("position");
		initPosition_ = Vector3d(doubleVect.data());

		// Read initial orientation quaternion
		doubleVect.clear();
		doubleVect = initConfig.get<vector<double>>("orientation");
		initOrientation_ = Quaterniond(doubleVect[3], doubleVect[0], doubleVect[1], doubleVect[2]);

		// Read initial velocity
		doubleVect.clear();
		doubleVect = initConfig.get<vector<double>>("velLin");
		initVelLinear_ = Vector3d(doubleVect.data());

		// Read initial angular velocity
		doubleVect.clear();
		doubleVect = initConfig.get<vector<double>>("velAng");
		initVelAngular_ = Vector3d(doubleVect.data());

		// Initialize WGS coordinates
		doubleVect.clear();
		doubleVect = initConfig.get<vector<double>>("coordinates");
		initCoordinates_(0) = doubleVect[0];
		initCoordinates_(1) = doubleVect[1];
		initCoordinates_(2) = doubleVect[2] - initPosition_.z(); // Read ground geoid altitude and raise the WGS coordinate by the NED altitude

		try
		{
			initChanReset_ = initConfig.get<int>("chanReset");
		}
		catch (const std::exception &)
		{
			initChanReset_ = -1;
		}

		// Initialize input
		try
		{
			initCtrlInput_.value = initConfig.get<vector<double>>("ctrlInput");
		}
		catch (const std::exception &)
		{
		}
	}

	///////////////////////////////////////
	// Make one step of the plane simulation
	void UavModel::step(LinkStateMap_t linkStates)
	{
		// Perform step actions serially

		setLinkStates(linkStates);

		// Calculate dynamics
		linkWrenches = dynamics.calcWrench(linkStates_, kinematics.inertial, environmentModel.environment);

		// SimState_t newState;
		// newState = kinematics.propagateState(state, inpWrench);

		// // Check new state for non-finite values
		// Vector3d tempVect;
		// Quaterniond tempQuat;
		// if (!newState.pose.position.allFinite())
		// {
		// 	tempVect = newState.pose.position;
		// 	cout << "New position:\n"
		// 		 << tempVect << endl;
		// 	throw runtime_error("uav_model.cpp: NaN member in new position");
		// }
		// if (!myisfinite(newState.pose.orientation))
		// {
		// 	tempQuat = newState.pose.orientation;
		// 	cout << "New orientation:\n"
		// 		 << tempQuat.w() << "\n"
		// 		 << tempQuat.vec() << endl;
		// 	throw runtime_error("uav_model.cpp: NaN member in new orientation");
		// }
		// if (!newState.velocity.linear.allFinite())
		// {
		// 	tempVect = newState.velocity.linear;
		// 	cout << "New linear velocity:\n"
		// 		 << tempVect << endl;
		// 	throw runtime_error("uav_model.cpp: NaN member in new linear velocity");
		// }
		// if (!newState.velocity.angular.allFinite())
		// {
		// 	tempVect = newState.velocity.angular;
		// 	cout << "New angular velocity:\n"
		// 		 << tempVect << endl;
		// 	throw runtime_error("uav_model.cpp: NaN member in new angular velocity");
		// }

		// for (int i = 0; i < dynamics.nMotors; i++)
		// {
		// 	newState.rotorspeed[i] = dynamics.propulsion[i]->omega;
		// }

		// state = newState;
	}

	/////////////////////////////////////////////////////////////////
	// Make one step of the plane simulation using the existing state
	void UavModel::step()
	{

		// Calculate dynamics
		linkWrenches = dynamics.calcWrench(linkStates_, kinematics.inertial, environmentModel.environment);
	}

	////////////////////////
	// Set state explicitly
	void UavModel::setLinkStates(LinkStateMap_t linkStates)
	{
		// TODO: read the updated propeller velocities and store them (if needed)
		linkStates_ = linkStates;

		SimState_t bodyLinkState = linkStates["body_frd"];
		// Build geoid coordinates
		// TODO: Make this propagate coordinates, instead of delegating it to the PX4 adapter
		bodyLinkState.geoid.altitude = -bodyLinkState.pose.position.z();

		// Re-calculate the environment based on the new state
		// This should only involve static calculations, not dynamic
		environmentModel.calcEnvironment(bodyLinkState);

		// Update sensors
		for (auto it = std::begin(sensors); it != std::end(sensors); ++it)
		{
			(*it)->update(bodyLinkState, environmentModel.environment);
		}
	}

	/////////////////////////////////////////////////
	// Pass control inputs to sub-models
	void UavModel::setInput(const Input_t p_input)
	{
		input.value = p_input.value;
		dynamics.setInput(input);

		if (chanReset > -1)
		{ // If a reset channel is set
			if (input.value[chanReset] > 0.6)
			{ // Reset the simulation upon command
				init();
			}
		}
	}

	/////////////////////////////////////////////////
	// convert uS PWM values to control surface inputs
	void UavModel::setInputPwm(const InputPwm_t p_input)
	{
		PwmInput.value = p_input.value;
		dynamics.setInputPwm(PwmInput);

		if (chanReset > -1)
		{ // If a reset channel is set
			if (PwmInput.value[chanReset] > 1600)
			{ // Reset the simulation upon PWM command
				init();
			}
		}
	}

	////////////////////////
	// Set a model parameter
	bool UavModel::set_parameter(programming_utils::ParamType_t paramType, std::string name, double value)
	{
		using namespace last_letter_lib::programming_utils;
		switch (paramType)
		{
		case PARAM_TYPE_WORLD:
			return (configs.filter("world")).set(name, value);
			break;
		case PARAM_TYPE_ENV:
			return (configs.filter("env")).set(name, value);
			break;
		case PARAM_TYPE_INIT:
			return (configs.filter("init")).set(name, value);
			break;
		case PARAM_TYPE_INERTIAL:
			return (configs.filter("inertial")).set(name, value);
			break;
		case PARAM_TYPE_AERO:
			return (configs.filter("aero")).set(name, value);
			break;
		case PARAM_TYPE_PROP:
			return (configs.filter("prop")).set(name, value);
			break;
		case PARAM_TYPE_GROUND:
			return (configs.filter("ground")).set(name, value);
			break;
		default:
			std::cerr << "Cannot handle this parameter type" << std::endl;
			return false;
		}
		return true;
	}

	//////////////////////////////
	// Update all model parameters
	void UavModel::update_model()
	{
		updateConfigAll();
	}

	//////////////////
	// Class destructor
	UavModel::~UavModel()
	{
	}

} // namespace last_letter_lib
