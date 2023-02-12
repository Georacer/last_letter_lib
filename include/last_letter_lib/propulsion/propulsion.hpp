///////////////////////////////////////////
// Propulsion class related declarations //
///////////////////////////////////////////

#include <Eigen/Eigen>
#include "yaml-cpp/yaml.h"

#include "last_letter_lib/environment.hpp"
#include "last_letter_lib/math_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/prog_utils.hpp"

////////////////////////////////////////////
// Propulsion interface class declaration //
////////////////////////////////////////////

using Eigen::Vector3d;
using namespace last_letter_lib::uav_utils;
using last_letter_lib::math_utils::Polynomial;
using last_letter_lib::programming_utils::buildPolynomial;
using last_letter_lib::programming_utils::ParameterManager;

namespace last_letter_lib
{
	namespace propulsion
	{

		class Propulsion
		{
		public:
			///////////
			//Variables
			double dt;		   // Simulation time step
			double inputMotor; // control input (0-1)
			int chanMotor;
			double omega;		// motor angular speed in rad/s
			double rotationDir; // motor direction of rotation
			double theta;		// propeller angle in rads
			double normalWind;	// scalar wind normal to propeller disc
			Vector3d relativeWind;
			Wrench_t wrenchProp;

			///////////
			//Functions
			Propulsion(ParameterManager propConfig, ParameterManager worldConfig);
			virtual ~Propulsion();
			virtual void readParametersProp(ParameterManager config);
			virtual void readParametersWorld(ParameterManager config);

			void setInput(Input_t input);																	 // store control input
			void setInputPwm(InputPwm_t input);																 // store PWM control input
			void stepEngine(SimState_t states, Inertial_t inertial, Environment_t environment);				 // engine physics step, container for the generic class
			virtual void updateRadPS(SimState_t states, Inertial_t inertial, Environment_t environment) = 0; // Step the angular speed
			void rotateProp();																				 // Update the propeller angle
			virtual void getForce(SimState_t states, Inertial_t inertial, Environment_t environment) = 0;	 // Calculate Forces
			virtual void getTorque(SimState_t states, Inertial_t inertial, Environment_t environment) = 0;	 //Calculate Torques
		};

#include "no_engine.hpp"

#include "beard_engine.hpp"

#include "piston_engine.hpp"

#include "electric_engine.hpp"

#include "omega_control_engine.hpp"

		Propulsion *buildPropulsion(ParameterManager propConfig, ParameterManager worldConfig);
	} // namespace propulsion
} // namespace last_letter_lib
