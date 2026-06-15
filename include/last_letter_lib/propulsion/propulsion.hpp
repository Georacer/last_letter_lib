///////////////////////////////////////////
// Propulsion class related declarations //
///////////////////////////////////////////

#include <Eigen/Eigen>
#include <string>
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
using last_letter_lib::math_utils::Inertial;
using last_letter_lib::math_utils::Polynomial;
using last_letter_lib::programming_utils::buildPolynomial;
using last_letter_lib::programming_utils::ParameterManager;

namespace last_letter_lib
{
	namespace propulsion
	{

		class Propulsion : public Parametrized
		{
		public:
			///////////
			// Variables
			double dt;          // Simulation time step
			double inputMotor;  // control input (0-1)
			int chanMotor;
			double omega;       // motor angular speed in rad/s
			double rotationDir; // motor direction of rotation
			double theta;       // propeller angle in rads
			double normalWind;  // scalar wind normal to propeller disc
			Vector3d relativeWind;
			Wrench_t wrenchProp;

			///////////
			// Functions
			Propulsion(string name);
			virtual ~Propulsion();

            void initialize_parameters() override
            {
                set_param<double>("deltaT", 0.0025, false);
                set_param<double>("rotationDir", 0, false);
                set_param<int>("chanMotor", 0, false);
                set_param<int>("motorType", 0, false);
            }
            void update_parameters() override;

			void setInput(Input input);                                                                     // store control input
			void setInputPwm(InputPwm_t input);                                                             // store PWM control input
			void stepEngine(SimState_t states, Inertial inertial, Environment_t environment);               // engine physics step, container for the generic class
			virtual void updateRadPS(SimState_t states, Inertial inertial, Environment_t environment) = 0;  // Step the angular speed
			void rotateProp();                                                                              // Update the propeller angle
			virtual void getForce(SimState_t states, Inertial inertial, Environment_t environment) = 0;     // Calculate Forces
			virtual void getTorque(SimState_t states, Inertial inertial, Environment_t environment) = 0;    // Calculate Torques
		};

#include "no_engine.hpp"

#include "beard_engine.hpp"

#include "piston_engine.hpp"

#include "electric_engine.hpp"

#include "omega_control_engine.hpp"

		Propulsion *buildPropulsion(ParameterManager propConfig);
	} // namespace propulsion
} // namespace last_letter_lib
