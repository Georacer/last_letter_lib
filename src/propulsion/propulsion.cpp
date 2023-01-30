#include "last_letter_lib/propulsion/propulsion.hpp"

using namespace std;

using Eigen::Quaterniond;
using Eigen::Vector3d;

namespace last_letter_lib
{
	namespace propulsion
	{

		//////////////////////////////
		// Propulsion interfrace class
		//////////////////////////////

		// Constructor
		Propulsion::Propulsion(YAML::Node propConfig, YAML::Node worldConfig)
		{
			readParametersWorld(worldConfig);
			readParametersProp(propConfig);

			theta = 0; // Initialize propeller angle

			inputMotor = 0.0;
		}

		// Destructor
		Propulsion::~Propulsion()
		{
		}

		void Propulsion::readParametersWorld(YAML::Node config)
		{
			getParameter(config, "deltaT", dt);
		}

		void Propulsion::readParametersProp(YAML::Node config)
		{
			if (!getParameter(config, "chanMotor", chanMotor, false))
			{
				chanMotor = -1;
			}

			if (!getParameter(config, "rotationDir", rotationDir, false))
			{
				rotationDir = 1.0;
			}
		}

		void Propulsion::setInput(Input_t input)
		{
			// TODO: I shouldn't check this all the time, since it is an optional parameter
			if (chanMotor > -1)
			{
				inputMotor = input.value[chanMotor];
			}
		}

		void Propulsion::setInputPwm(InputPwm_t p_input)
		{
			Input_t input;
			if (chanMotor > -1)
			{
				input.value[chanMotor] = PwmToHalfRange(p_input.value[chanMotor]);
			}

			setInput(input);
		}

		// Engine physics step, container for the generic class
		void Propulsion::stepEngine(SimState_t states, Inertial_t inertial, Environment_t environment)
		{
			// std::cout << "received states and wind: \n"
			// 		  << states.velocity.linear << "\n"
			// 		  << environment.wind << "\n"
			// 		  << std::endl;
			relativeWind = states.velocity.linear - environment.wind;
			normalWind = relativeWind.x();
			if (!std::isfinite(normalWind))
			{
				throw runtime_error("propulsion.cpp: airspeed is not finite");
			}
			updateRadPS(states, inertial, environment);
			rotateProp();
			getForce(states, inertial, environment);
			getTorque(states, inertial, environment);
		}

		void Propulsion::rotateProp() // Update propeller angle
		{
			if (!std::isfinite(omega))
			{
				throw runtime_error("propulsion.cpp: non-finite omega value");
			}
			theta += omega * dt;
			if (theta > 2.0 * M_PI)
				theta -= 2 * M_PI;
			if (theta < 0.0)
				theta += 2 * M_PI;
		}

#include "no_engine.cpp"

#include "beard_engine.cpp"

#include "piston_engine.cpp"

#include "electric_engine.cpp"

#include "omega_control_engine.cpp"

		// Build engine model
		Propulsion *buildPropulsion(YAML::Node propConfig, YAML::Node worldConfig)
		{
			int motorType;
			getParameter(propConfig, "motorType", motorType);
			std::cout << "building engine model: ";
			switch (motorType)
			{
			case 0:
				std::cout << "selecting no engine" << std::endl;
				return new NoEngine(propConfig, worldConfig);
			case 1:
				std::cout << "selecting Beard engine" << std::endl;
				return new EngBeard(propConfig, worldConfig);
			case 2:
				std::cout << "selecting piston engine" << std::endl;
				return new PistonEng(propConfig, worldConfig);
			case 3:
				std::cout << "selecting electric engine" << std::endl;
				return new ElectricEng(propConfig, worldConfig);
			case 4:
				std::cout << "selecting omega-controlled engine" << std::endl;
				return new EngOmegaControl(propConfig, worldConfig);
			case 5:
				std::cout << "selecting electric engine 2" << std::endl;
				return new ElectricEng2(propConfig, worldConfig);
			default:
				throw runtime_error("Error while constructing motor");
				break;
			}
		}

	} // namespace propulsion
} // namespace last_letter_lib
