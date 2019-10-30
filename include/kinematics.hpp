#include <iostream>
#include <Eigen/Eigen>
#include "yaml-cpp/yaml.h"

#include "uav_utils.hpp"
#include "prog_utils.hpp"
#include "math_utils.hpp"


using namespace std;

using Eigen::Vector3d;
using Eigen::Quaterniond;

struct Derivatives_t {
	Vector3d posDot;
	Vector3d speedDot;
	Vector3d rateDot;
	Quaterniond quatDot;
	Vector3d coordDot;
};

// Kinematics equations related classes

// State integrator interface class
// Propagates the state derivatives onto the ModelPlane states
class Integrator
{
	public:
	double dt;
	Integrator(YAML::Node worldConfig);
	virtual ~Integrator();
	virtual SimState_t propagation(SimState_t states, Derivatives_t derivatives) =0;
};

// Forward Euler integrator class
class ForwardEuler : public Integrator
{
	public:
	ForwardEuler(YAML::Node worldConfig);
	SimState_t propagation(SimState_t states, Derivatives_t derivatives);
};

Integrator * buildIntegrator(YAML::Node worldConfig);

// Main generic class
class Kinematics
{
	public:
	Kinematics(YAML::Node inertialConfig, YAML::Node worldConfig);
	~Kinematics();

	double dt;
	Derivatives_t stateDot;
	Inertial_t inertial;
	void calcDerivatives(SimState_t states, Wrench_t inpWrench); // Do not use this method (private)
	SimState_t propagateState(SimState_t states, Wrench_t inpWrench); // Use this method to calculate state integral
	void readParametersWorld(YAML::Node worldConfig);
	void readParametersInertial(YAML::Node inertialConfig);
	Integrator * integrator;
};
