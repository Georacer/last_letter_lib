#include "last_letter_lib/dynamics.hpp"
#include "last_letter_lib/environment.hpp"
#include "last_letter_lib/propulsion/propulsion.hpp"
#include "last_letter_lib/uav_utils.hpp"

namespace last_letter_lib
{

	//////////////////////////
	// Define Dynamics class
	//////////////////////////

	///////////////////
	//Class Constructor
	Dynamics::Dynamics(ParameterManager p_worldConfig, ParameterManager p_aeroConfig, ParameterManager p_propConfig, ParameterManager p_groundConfig)
	{
		// Create and initialize aerodynamic objects array
		nWings = p_aeroConfig.get<int>("nWings");
        aerodynamics.clear();
		for (int i = 0; i < nWings; i++)
		{
			ParameterManager aeroConfig = p_aeroConfig.filter("airfoil" + std::to_string(i + 1) + "/");
            auto new_aero = aerodynamics::buildAerodynamics(aeroConfig);
            aerodynamics.push_back(new_aero.get());
            components.push_back(std::move(new_aero));
		}

		// Create and initialize motor objects array
		nMotors = p_propConfig.get<int>("nMotors");
        thrusters.clear();
		for (int i = 0; i < nMotors; i++)
		{
			ParameterManager propConfig = p_propConfig.filter("motor" + std::to_string(i + 1) + "/");
            propConfig.register_child_mngr(p_worldConfig);
            auto new_thruster = propulsion::buildThruster(propConfig);
            thrusters.push_back(new_thruster.get());
            components.push_back(std::move(new_thruster));
		}

		// Create and initialize ground reactions object
		groundReaction = ground_reaction::buildGroundReaction(p_groundConfig);
	}

	void Dynamics::readParametersAerodynamics(ParameterManager config)
	{
		for (int i = 0; i < nWings; i++)
		{
			ParameterManager aeroConfig = config.filter("airfoil" + std::to_string(i + 1) + "/");
            aerodynamics.at(i)->load_parameters(aeroConfig);
            aerodynamics.at(i)->update_parameters();
		}
	}

	void Dynamics::readParametersProp(ParameterManager config)
	{
		for (int i = 0; i < nMotors; i++)
		{
			ParameterManager propConfig = config.filter("motor" + std::to_string(i + 1) + "/");
            thrusters.at(i)->load_parameters(propConfig);
            thrusters.at(i)->update_parameters();
		}
	}

	void Dynamics::readParametersWorld(ParameterManager /*config*/)
	{
	}

	void Dynamics::readParametersGround(ParameterManager config)
	{
        groundReaction->load_parameters(config);
        groundReaction->update_parameters();
	}

	//Class Destructor
	Dynamics::~Dynamics()
	{
        delete groundReaction;
	}

	// Order subsystems to store control input
	void Dynamics::setInput(Input input)
	{
        for (auto &thruster : thrusters) {
            thruster->setInput(input);
        }
        for (auto &airfoil : aerodynamics) {
            airfoil->setInput(input);
        }
		groundReaction->setInput(input);
	}

	// Order subsystems to store control input, passed as PWM micorseconds
	void Dynamics::setInputPwm(InputPwm_t input)
	{
        for (auto &thruster : thrusters) {
            thruster->setInputPwm(input);
        }
        for (auto &airfoil : aerodynamics) {
            airfoil->setInputPwm(input);
        }
		groundReaction->setInputPwm(input);
	}

    void Dynamics::update_local_state(SimState_t states, Environment_t environment)
    {
        for (auto &component : components) {
            component->update_local_state(states, environment);
        }
    }

	// Calculate the forces and torques for each Wrench_t source
	void Dynamics::calc_model(SimState_t states)
	{
        WrenchSum_t new_sum;
        for (auto &component : components) {
            component->calc_model();
            new_sum += component->wrench_sum;
        }

		// Call ground reactions routines - MUST BE CALLED LAST!!!
		// This is needed for some ground reactions models, which are designed to counter the remaining force sum
		new_sum.wrenchGround.force = groundReaction->getForce(states, new_sum);
		if (!new_sum.wrenchGround.force.allFinite())
		{
			throw runtime_error("dynamicsLib.cpp: NaN member in groundReaction force vector");
		}

		new_sum.wrenchGround.torque = groundReaction->getTorque(states, new_sum);
		if (!new_sum.wrenchGround.torque.allFinite())
		{
			throw runtime_error("dynamicsLib.cpp: NaN member in groundReaction torque vector");
		}

        wrench_sum = new_sum;
	}

} // namespace last_letter_lib
