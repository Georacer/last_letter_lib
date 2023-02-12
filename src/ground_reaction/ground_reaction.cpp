////////////////////////////////////////
// Define GroundReaction interface class
////////////////////////////////////////

#include "last_letter_lib/ground_reaction/ground_reaction.hpp"

namespace last_letter_lib
{
	namespace ground_reaction
	{

		// Class constructor
		GroundReaction::GroundReaction(ParameterManager config, ParameterManager worldConfig)
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

		void GroundReaction::readParametersGround(ParameterManager config)
		{
			try {chanSteer = config.get<int>("chanSteer");}
			catch (const std::exception&) { chanSteer = -1;}
			try {chanBrake = config.get<int>("chanBrake");}
			catch (const std::exception&) { chanBrake = -1;}
			try {steerAngle_max = config.get<int>("steerAngle_max");}
			catch (const std::exception&) { steerAngle_max = -1;}
		}

		void GroundReaction::readParametersWorld(ParameterManager config)
		{
			dt = config.get<double>("deltaT");
		}

		// Store the steering and brake control inputs
		void GroundReaction::setInput(Input_t input)
		{
			if (chanSteer > -1)
			{
				inputSteer = steerAngle_max * input.value[chanSteer];
			}
			if (chanBrake > -1)
			{
				inputBrake = input.value[chanBrake];
			}
		}

		// Store the steering and brake control inputs
		void GroundReaction::setInputPwm(InputPwm_t p_input)
		{
			Input_t input;
			if (chanSteer > -1)
			{
				input.value[chanSteer] = PwmToFullRange(p_input.value[chanSteer]);
			}
			if (chanBrake > -1)
			{
				input.value[chanBrake] = PwmToHalfRange(p_input.value[chanBrake]);
			}

			setInput(input);
		}

#include "no_ground_reactions.cpp"

#include "panos_contact_points.cpp"

#include "point_friction.cpp"

		// Build ground reactions model
		GroundReaction *buildGroundReaction(ParameterManager config, ParameterManager worldConfig)
		{
			int groundReactionType;
			groundReactionType = config.get<double>("groundReactionType");
			std::cout << "building ground reactions model: ";
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
	} // namespace ground_reaction
} // namespace last_letter_lib
