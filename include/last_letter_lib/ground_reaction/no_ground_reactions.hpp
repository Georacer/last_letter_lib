/// No ground reactions, dummy class

class NoGroundReaction : public GroundReaction
{
public:
	///This is the NoGroundReaction constructor brief description
	NoGroundReaction(ParameterManager config, ParameterManager worldConfig);
	~NoGroundReaction();
	Vector3d getForce(const SimState_t states, const WrenchSum_t wrenchSum);
	Vector3d getTorque(const SimState_t states, const WrenchSum_t wrenchSum);
};
