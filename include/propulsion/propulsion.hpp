///////////////////////////////////////////
// Propulsion class related declarations //
///////////////////////////////////////////

#include <Eigen/Eigen>
#include "yaml-cpp/yaml.h"

#include "environment.hpp"
#include "math_utils.hpp"
#include "uav_utils.hpp"
#include "prog_utils.hpp"

////////////////////////////////////////////
// Propulsion interface class declaration //
////////////////////////////////////////////
class Propulsion
{
	public:
	///////////
	//Variables
	double dt; // Simulation time step
	Vector3d CGOffset; // vector from CG to engine coordinates
	Vector3d mountOrientation;
	double inputMotor, inputGimbal; // control input (0-1)
	double gimbalAngle_max;
	int chanMotor, chanGimbal;
	double omega; // motor angular speed in rad/s
	double rotationDir; // motor direction of rotation
	double theta; // propeller angle in rads
	double normalWind; // scalar wind normal to propeller disc
	Wrench_t wrenchProp;

    Eigen::Quaterniond q_bm, q_mg, q_bg; // Quaternion rotations
	Eigen::Transform<double, 3, Eigen::Affine> body_to_mount, mount_to_gimbal, gimbal_to_prop, body_to_prop; // Transformations in the propeller assembly for visual rendering
	Eigen::Transform<double, 3, Eigen::Affine> body_to_mount_rot, mount_to_gimbal_rot, gimbal_to_prop_rot, body_to_prop_rot; // Transformations in the propeller assembly for force and moment rotation

	///////////
	//Functions
	Propulsion(YAML::Node propConfig, YAML::Node worldConfig);
	virtual ~Propulsion();
	virtual void readParametersProp(YAML::Node config);
	virtual void readParametersWorld(YAML::Node config);

	void setInput(Input_t input); // store control input
	void setInputPwm(InputPwm_t input); // store PWM control input
	void stepEngine(SimState_t states, Inertial_t inertial, Environment_t environment); // engine physics step, container for the generic class
	void rotateWind(SimState_t states, Inertial_t inertial, Environment_t environment); // convert the wind to the propeller axes
	virtual void updateRadPS(SimState_t states, Inertial_t inertial, Environment_t environment) =0; // Step the angular speed
	void rotateProp(); // Update the propeller angle
	void rotateForce(); // convert the resulting force to the body axes
	void rotateTorque(Inertial_t inertial); // convert the resulting torque to the body axes
	virtual void getForce(SimState_t states, Inertial_t inertial, Environment_t environment) =0; // Calculate Forces
	virtual void getTorque(SimState_t states, Inertial_t inertial, Environment_t environment) =0; //Calculate Torques
};

#include "no_engine.hpp"

#include "beard_engine.hpp"

#include "piston_engine.hpp"

#include "electric_engine.hpp"

Propulsion * buildPropulsion(YAML::Node propConfig, YAML::Node worldConfig);