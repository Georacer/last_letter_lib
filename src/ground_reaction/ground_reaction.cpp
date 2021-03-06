////////////////////////////////////////
// Define GroundReaction interface class
////////////////////////////////////////

#include "ground_reaction.hpp"

// Class constructor
GroundReaction::GroundReaction(YAML::Node config, YAML::Node worldConfig)
{
	readParametersWorld(worldConfig);
	readParametersGround(config);

	inputSteer = 0.0;
	inputBrake = 0.0;
}

// Class destructor
GroundReaction::~GroundReaction()
{
}

void GroundReaction::readParametersGround(YAML::Node config)
{
	if (!getParameter(config, "chanSteer", chanSteer)) {chanSteer = -1;}
	if (!getParameter(config, "chanBrake", chanBrake)) {chanBrake = -1;}
	if (!getParameter(config, "steerAngle_max", steerAngle_max)) {steerAngle_max = 0.0;}
}

void GroundReaction::readParametersWorld(YAML::Node config)
{
	getParameter(config, "deltaT", dt);
}

// Store the steering and brake control inputs
void GroundReaction::setInput(Input_t input)
{
	if (chanSteer>-1) {inputSteer = steerAngle_max * input.value[chanSteer];}
	if (chanBrake>-1) {inputBrake = input.value[chanBrake];}
}

// Store the steering and brake control inputs
void GroundReaction::setInputPwm(InputPwm_t p_input)
{
	Input_t input;
	if (chanSteer>-1) {input.value[chanSteer] = PwmToFullRange(p_input.value[chanSteer]);}
	if (chanBrake>-1) {input.value[chanBrake] = PwmToHalfRange(p_input.value[chanBrake]);}

	setInput(input);
}


#include "no_ground_reactions.cpp"

#include "panos_contact_points.cpp"

#include "point_friction.cpp"


// Build ground reactions model
GroundReaction * buildGroundReaction(YAML::Node config, YAML::Node worldConfig)
{
	int groundReactionType;
	getParameter(config, "groundReactionType", groundReactionType);
	std::cout<< "building ground reactions model: ";
	switch (groundReactionType)
	{
	case 0:
		std::cout << "selecting no ground reactions" << std::endl;
		return new NoGroundReaction(config, worldConfig);
	case 1:
		std::cout << "selecting Panos ground reactions" << std::endl;
		return new PanosContactPoints(config, worldConfig);
	case 2:
		std::cout << "selecting PointFriction ground reactions" << std::endl;
		return new PointFriction(config, worldConfig);
	default:
		throw runtime_error("Error while constructing ground reactions");
		break;
	}
}