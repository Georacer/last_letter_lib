#include "yaml-cpp/yaml.h"

#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/environment.hpp"
#include "last_letter_lib/dynamics.hpp"
#include "last_letter_lib/kinematics.hpp"
#include "last_letter_lib/sensors.hpp"

using namespace std;
using namespace last_letter_lib::programming_utils;

using Eigen::Quaterniond;
using Eigen::Vector3d;

namespace last_letter_lib
{

	// Core class declarations

	// Top UavModel object class
	class UavModel
	{
	public:
		///////////
		// Variables
		ParameterManager configs;
		// SimState_t state;	 // main simulation states
		LinkWrenchMap_t linkWrenches;
		Input_t input;		 // Normalized input to the model
		InputPwm_t PwmInput; // PWM input to the model
		double dt;			 // simulation timestep in s
		int chanReset;

		/////////
		// Members
		Kinematics kinematics;
		Dynamics dynamics;
		EnvironmentModel environmentModel;
		std::vector<std::shared_ptr<Sensor>> sensors;

	private:
		// Initialization parameters
		Vector3d initPosition_, initVelLinear_, initVelAngular_, initCoordinates_;
		Quaterniond initOrientation_;
		int initChanReset_;
		Input_t initCtrlInput_;
		// Link state storage
		LinkStateMap_t linkStates_;

		///////////
		// Methods
	public:
		// Constructor
		UavModel(programming_utils::ParameterManager configs);

		// Initialize UavModel object
		void init();

		// Destructor
		~UavModel();

		void setLinkStates(LinkStateMap_t linkStates);			  // Set simulation state
		SimState_t getState() { return linkStates_["body_frd"]; } // Return the state of the Body Frame
		void setInput(Input_t inputMsg);
		void setInputPwm(InputPwm_t inputMsg);
		bool set_parameter(programming_utils::ParamType_t paramType, std::string name, double value);
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

		void readParametersWorld(ParameterManager worldConfig);
		void readParametersInit(ParameterManager initConfig);

		void step(LinkStateMap_t linkStates); // Perform simulation step
		void step();						  // Perform simulation step on stored link states
	};
} // namespace last_letter_lib
