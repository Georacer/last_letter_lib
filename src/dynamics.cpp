#include "last_letter_lib/dynamics.hpp"
#include "last_letter_lib/environment.hpp"

namespace last_letter_lib
{

	//////////////////////////
	// Define Dynamics class
	//////////////////////////

	///////////////////
	//Class Constructor
	Dynamics::Dynamics(YAML::Node p_worldConfig, YAML::Node p_aeroConfig, YAML::Node p_propConfig, YAML::Node /*p_groundConfig*/)
	{
		// Create and initialize aerodynamic objects array
		getParameter(p_aeroConfig, "nWings", nWings);
		aerodynamicLinks = new LinkAerodynamic *[nWings];
		for (int i = 0; i < nWings; i++)
		{
			YAML::Node aeroConfig = filterConfig(p_aeroConfig, "airfoil" + std::to_string(i + 1) + "/");
			aerodynamicLinks[i] = new LinkAerodynamic(aeroConfig, p_worldConfig); // Create a new aerodynamics object, id's are 1-indexed
		}

		// Create and initialize gravity object
		// gravity = new Gravity();

		// Create and initialize motor objects array
		getParameter(p_propConfig, "nMotors", nMotors);
		propulsionLinks = new LinkPropulsion *[nMotors];
		for (int i = 0; i < nMotors; i++)
		{
			YAML::Node propConfig = filterConfig(p_propConfig, "motor" + std::to_string(i + 1) + "/");
			propulsionLinks[i] = new LinkPropulsion(propConfig, p_worldConfig); // Create a new propulsion object, id's are 1-indexed
		}

		// Create and initialize ground reactions object
		// groundReactionLink = new LinkGroundReaction(p_groundConfig, p_worldConfig);
	}

	void Dynamics::readParametersAerodynamics(YAML::Node config)
	{
		for (int i = 0; i < nWings; i++)
		{
			YAML::Node aeroConfig = filterConfig(config, "airfoil" + std::to_string(i + 1) + "/");
			aerodynamicLinks[i]->readParametersModel(aeroConfig); // Update aerodynamic parameters
		}
	}

	void Dynamics::readParametersProp(YAML::Node config)
	{
		for (int i = 0; i < nMotors; i++)
		{
			YAML::Node propConfig = filterConfig(config, "motor" + std::to_string(i + 1) + "/");
			propulsionLinks[i]->readParametersModel(propConfig); // Update propulsion parameters
		}
	}

	void Dynamics::readParametersWorld(YAML::Node config)
	{
		for (int i = 0; i < nMotors; i++)
		{
			propulsionLinks[i]->readParametersWorld(config); // Update world parameters
		}
		// groundReactionLink->readParametersWorld(config);
	}

	void Dynamics::readParametersGround(YAML::Node /*config*/)
	{
		// groundReactionLink->readParametersModel(config);
	}

	//Class Destructor
	Dynamics::~Dynamics()
	{
		for (int i = 0; i < nWings; i++)
		{
			delete aerodynamicLinks[i]; // delete all aerodynamics objects
		}
		delete aerodynamicLinks; // Must also separately delete the array of object pointers
		// delete gravity;
		for (int i = 0; i < nMotors; i++)
		{
			delete propulsionLinks[i]; // delete all propulsion objects
		}
		delete propulsionLinks; // Must also separately delete the array of object pointers
								// delete groundReactionLink;
	}

	// Order subsystems to store control input
	void Dynamics::setInput(Input_t input)
	{
		for (int i = 0; i < nMotors; i++)
		{
			propulsionLinks[i]->setInput(input);
		}
		for (int i = 0; i < nWings; i++)
		{
			aerodynamicLinks[i]->setInput(input);
		}
		// groundReactionLink->setInput(input);
	}

	// Order subsystems to store control input, passed as PWM micorseconds
	void Dynamics::setInputPwm(InputPwm_t input)
	{
		for (int i = 0; i < nMotors; i++)
		{
			propulsionLinks[i]->setInputPwm(input);
		}
		for (int i = 0; i < nWings; i++)
		{
			aerodynamicLinks[i]->setInputPwm(input);
		}
		// groundReactionLink->setInputPwm(input);
	}

	// Calculate the forces and torques for each Wrench_t source
	LinkWrenchMap_t Dynamics::calcWrench(LinkStateMap_t states, Inertial_t inertial, Environment_t environment)
	{
		LinkWrenchMap_t linkWrenches;

		// Vector3d tempVect;
		// // Call gravity calculation routines
		// forceGrav = gravity->getForce(states.pose.orientation.conjugate(), environment.gravity, inertial.mass);
		// if (!forceGrav.allFinite())
		// {
		// 	throw runtime_error("dynamicsLib.cpp: NaN member in gravity force vector");
		// }

		// torqueGrav = gravity->getTorque(states.pose.orientation.conjugate(), environment.gravity, inertial.mass);
		// if (!torqueGrav.allFinite())
		// {
		// 	throw runtime_error("dynamicsLib.cpp: NaN member in gravity torque vector");
		// }

		// Call  motors routines

		for (int i = 0; i < nMotors; i++)
		{
			// Execute one step in the motor dynamics
			std::string link_name = propulsionLinks[i]->name;
			auto link_state = states[link_name];
			propulsionLinks[i]->step(link_state, inertial, environment);
			linkWrenches[link_name] = propulsionLinks[i]->wrenchLinkFrame;
		}
		// Execute one step in the aerodynamics
		for (int i = 0; i < nWings; i++)
		{
			std::string link_name = aerodynamicLinks[i]->name;
			auto link_state = states[aerodynamicLinks[i]->name];
			aerodynamicLinks[i]->step(link_state, inertial, environment);
			linkWrenches[link_name] = aerodynamicLinks[i]->wrenchLinkFrame;
		}

		return linkWrenches;

		// // Call ground reactions routines - MUST BE CALLED LAST!!!
		// // This is needed for some ground reactions models, which are designed to counter the remaining force sum
		// WrenchSum_t wrenchSum;
		// wrenchSum.wrenchAero.force = forceAero;
		// wrenchSum.wrenchAero.torque = torqueAero;
		// wrenchSum.wrenchProp.force = forceProp;
		// wrenchSum.wrenchProp.torque = torqueProp;
		// wrenchSum.wrenchGrav.force = forceGrav;
		// wrenchSum.wrenchGrav.torque = torqueGrav;
		// forceGround = groundReactionLink->getForce(states, wrenchSum);
		// if (!forceGround.allFinite())
		// {
		// 	throw runtime_error("dynamicsLib.cpp: NaN member in groundReaction force vector");
		// }

		// torqueGround = groundReactionLink->getTorque(states, wrenchSum);
		// if (!torqueGround.allFinite())
		// {
		// 	throw runtime_error("dynamicsLib.cpp: NaN member in groundReaction torque vector");
		// }
	}

	// // Collect forces from underlying models
	// Vector3d Dynamics::getForce()
	// {
	// 	Vector3d accumulator;

	// 	accumulator = forceGrav;
	// 	accumulator = accumulator + forceProp;
	// 	accumulator = accumulator + forceAero;
	// 	accumulator = accumulator + forceGround;

	// 	return accumulator;
	// }

	// // Collect torques from underlying models
	// Vector3d Dynamics::getTorque()
	// {
	// 	Vector3d accumulator;

	// 	accumulator = torqueGrav;
	// 	accumulator = accumulator + torqueProp;
	// 	accumulator = accumulator + torqueAero;
	// 	accumulator = accumulator + torqueGround;

	// 	return accumulator;
	// }

} // namespace last_letter_lib
