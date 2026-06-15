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
		Propulsion::Propulsion(string name)
            : Parametrized(name)
		{
			theta = 0; // Initialize propeller angle
			inputMotor = 0.0;
		}

		// Destructor
		Propulsion::~Propulsion()
		{
		}

        void Propulsion::update_parameters()
        {
            dt = get_param<double>("deltaT");
            chanMotor = get_param<int>("chanMotor");
            rotationDir = get_param<float>("rotationDir");
        }

		void Propulsion::setInput(Input input)
		{
			// TODO: I shouldn't check this all the time, since it is an optional parameter
			if (chanMotor > -1)
			{
				inputMotor = input.value[chanMotor];
			}
		}

		void Propulsion::setInputPwm(InputPwm_t p_input)
		{
			Input input;
			if (chanMotor > -1)
			{
				input.value[chanMotor] = PwmToHalfRange(p_input.value[chanMotor]);
			}

			setInput(input);
		}

		// Engine physics step, container for the generic class
		void Propulsion::stepEngine(SimState_t states, Inertial inertial, Environment_t environment)
		{
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
        Propulsion *buildPropulsion(ParameterManager propConfig)
        {
			int motorType;
			motorType = propConfig.get<double>("motorType");
            string engineName = propConfig.get<string>("name");
			std::cout << "building engine model: ";
            Propulsion *engine;
			switch (motorType)
			{
			case 0:
				std::cout << "selecting no engine" << std::endl;
				engine = new NoEngine(engineName);
                break;
			case 1:
				std::cout << "selecting Beard engine" << std::endl;
				engine = new EngBeard(engineName);
                break;
			case 2:
				std::cout << "selecting piston engine" << std::endl;
				engine = new PistonEng(engineName);
                break;
			case 3:
				std::cout << "selecting electric engine" << std::endl;
				engine = new ElectricEng(engineName);
                break;
			case 4:
				std::cout << "selecting omega-controlled engine" << std::endl;
				engine = new EngOmegaControl(engineName);
                break;
			case 5:
				std::cout << "selecting electric engine 2" << std::endl;
				engine = new ElectricEng2(engineName);
                break;
			default:
				throw runtime_error("Error while constructing motor");
				break;
			}
            std::cout << "Initializing engine" << std::endl;
            engine->initialize(propConfig);
            return engine;
		}

	} // namespace propulsion
} // namespace last_letter_lib
