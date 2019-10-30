#include <Eigen/Eigen>
#include "yaml-cpp/yaml.h"

#include "uav_utils.hpp"
#include "aerodynamics.hpp"
#include "gravity.hpp"
#include "propulsion.hpp"
#include "ground_reaction.hpp"
#include "environment.hpp"

using namespace std;

using Eigen::Vector3d;

// Dynamic model aggregator class

// Container class
class Dynamics
{
	public:
	Vector3d force, forceGrav, forceAero, forceProp, forceGround;
	Vector3d torque, torqueGrav, torqueAero, torqueProp, torqueGround;
	Dynamics(YAML::Node p_worldConfig, YAML::Node p_aeroConfig, YAML::Node p_propConfig, YAML::Node p_groundConfig);
	~Dynamics();
	void readParametersAerodynamics(YAML::Node config);
	void readParametersProp(YAML::Node config);
	void readParametersGround(YAML::Node config);
	void readParametersWorld(YAML::Node config);
	int nWings; // number of airfoils mounted on the aircraft
	Aerodynamics ** aerodynamics;
	Gravity * gravity;
	int nMotors; // number of motors mounted on the aircraft
	Propulsion ** propulsion;
	GroundReaction * groundReaction;
	void setInput(Input_t input); // store and convert new input values
	void setInputPwm(InputPwm_t input); // store and convert new PWM input values
	void calcWrench(SimState_t states, Inertial_t inertial, Environment_t environment); // Calculate the forces and torques for each Wrench_t source
	Vector3d getForce(); // Access class members and gather resulting forces
	Vector3d getTorque(); // Access class members and gather resulting torques
};