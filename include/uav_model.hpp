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
	Airdata airdata; // TODO: Is this member used?

	private:
	// Initialization parameters
	Vector3d initPosition_, initVelLinear_, initVelAngular_, initCoordinates_;
	Quaterniond initOrientation_;
	int initChanReset_;
	Input_t initCtrlInput_;

	///////////
	//Methods
	public:

	// Constructor
	UavModel (ConfigsStruct_t configs);

	// Initialize UavModel object
	void init();

	// Destructor
	~UavModel ();

	void setState(const SimState_t p_state);
	void setInput(Input_t inputMsg);
	void setInputPwm(InputPwm_t inputMsg);
	bool set_parameter(ParamType_t paramType, std::string name, double value);
	void update_model();

	// Force all models to re-read configuration parameters
	void updateConfigWorld();
	void updateConfigEnvironment();
	void updateConfigInit();
	void updateConfigInertial();
	void updateConfigAero();
	void updateConfigProp();
	void updateConfigGround();
	void updateConfigAll();

	void readParametersWorld(YAML::Node worldConfig);
	void readParametersInit(YAML::Node initConfig);

	// Perform simulation step
	void step(void);
};