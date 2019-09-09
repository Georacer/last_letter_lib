#include "yaml-cpp/yaml.h"

#include "uav_utils.hpp"
#include "environment.hpp"
#include "dynamics.hpp"
#include "kinematics.hpp"

using namespace std;

using Eigen::Vector3d;
using Eigen::Quaterniond;

// Core class declarations


// Top UavModel object class
class UavModel
{
	public:
	///////////
	//Variables
	ConfigsStruct_t configs;
	SimState_t state; // main simulation states
	Input_t input; // Normalized input to the model
	InputPwm_t PwmInput; // PWM input to the model
	double dt; // simulation timestep in s
	int chanReset;

	/////////
	//Members
	Kinematics kinematics;
	Dynamics dynamics;
	EnvironmentModel environmentModel;
	Airdata airdata;

	///////////
	//Methods

	// Constructor
	UavModel (ConfigsStruct_t configs);

	// Initialize UavModel object
	void init(YAML::Node initConfig);
	void init();

	// Destructor
	~UavModel ();

	// Input callback
	/**
	 * setInput Read PWM input to the model and store its normalized values
	 * @param inputMsg Direct servo control commands
	 */
	void setInput(Input_t inputMsg);
	void setInputPwm(InputPwm_t inputMsg);

	// TODO: Implement this
	// void setConfig(new_config)

	// Perform simulation step
	void step(void);
};