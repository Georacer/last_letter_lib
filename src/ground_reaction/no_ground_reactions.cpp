/////////////////////////////////
// Define NoGroundReactions class
/////////////////////////////////

// Constructor
NoGroundReaction::NoGroundReaction(ParameterManager config, ParameterManager worldConfig) : GroundReaction(config, worldConfig)
{
}

// Destructor
NoGroundReaction::~NoGroundReaction()
{
}

// Force calculation function
Vector3d NoGroundReaction::getForce(const SimState_t /*states*/, const WrenchSum_t /*wrenchSum*/)
{
	return wrenchGround.force;
}

// Torque calculation function
Vector3d NoGroundReaction::getTorque(const SimState_t /*states*/, const WrenchSum_t /* wrenchSum */)
{
	return wrenchGround.torque;
}
