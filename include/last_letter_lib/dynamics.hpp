#include <Eigen/Eigen>
#include "yaml-cpp/yaml.h"

#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/prog_utils.hpp"
// #include "last_letter_lib/aerodynamics.hpp"
#include "last_letter_lib/gravity.hpp"
// #include "last_letter_lib/propulsion/propulsion.hpp"
// #include "last_letter_lib/ground_reaction/ground_reaction.hpp"
#include "last_letter_lib/environment.hpp"
#include "last_letter_lib/link.hpp"

using namespace std;

using Eigen::Quaterniond;
using Eigen::Vector3d;

// Dynamic model aggregator class

namespace last_letter_lib
{
	// Container class
	class Dynamics
	{
	public:
		Vector3d forceGrav, forceAero, forceProp, forceGround;
		Vector3d torqueGrav, torqueAero, torqueProp, torqueGround;
		Dynamics(ParameterManager p_worldConfig, ParameterManager p_aeroConfig, ParameterManager p_propConfig, ParameterManager p_groundConfig);
		~Dynamics();
		void readParametersAerodynamics(ParameterManager config);
		void readParametersProp(ParameterManager config);
		void readParametersGround(ParameterManager config);
		void readParametersWorld(ParameterManager config);
		int nWings; // number of airfoils mounted on the aircraft
		LinkAerodynamic **aerodynamicLinks;
		// Gravity *gravity;
		int nMotors; // number of motors mounted on the aircraft
		LinkPropulsion **propulsionLinks;
		// LinkGroundReaction *groundReactionLink;
		void setInput(Input input);																	   // store and convert new input values
		void setInputPwm(InputPwm_t input);																   // store and convert new PWM input values
		LinkWrenchMap_t calcWrench(LinkStateMap_t states, Inertial inertial, Environment_t environment); // Calculate the forces and torques for each Wrench_t source
																										   // Vector3d getForce();																		   // Access class members and gather resulting forces
																										   // Vector3d getTorque();																		   // Access class members and gather resulting torques
	};
} // namespace last_letter_lib
