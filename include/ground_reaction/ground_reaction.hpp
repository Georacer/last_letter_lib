/// Ground reactions interface class

#include <iostream>
#include <Eigen/Eigen>

#include "prog_utils.hpp"
#include "uav_utils.hpp"

using namespace std;

using Eigen::Vector3d;

class GroundReaction
{
	public:
	GroundReaction(YAML::Node config, YAML::Node worldConfig);
	virtual ~GroundReaction();
	virtual void readParametersGround(YAML::Node config);
	virtual void readParametersWorld(YAML::Node config);
	double dt;
	Wrench_t wrenchGround;
	double inputSteer, inputBrake;
	double steerAngle_max;
	int chanSteer, chanBrake;
	void setInput(Input_t input);
	void setInputPwm(InputPwm_t input);
	virtual Vector3d getForce(const SimState_t states, const WrenchSum_t wrenchSum)=0;
	virtual Vector3d getTorque(const SimState_t states, const WrenchSum_t wrenchSum)=0;
};

#include "no_ground_reactions.hpp"

#include "panos_contact_points.hpp"

#include "point_friction.hpp"

GroundReaction * buildGroundReaction(YAML::Node config, YAML::Node worldConfig);