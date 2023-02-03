/// Elementary point-friction model

using Eigen::Dynamic;

class PointFriction : public GroundReaction
{
	/// Does not include rolling wheel model
public:
	PointFriction(YAML::Node config, YAML::Node worldConfig);
	~PointFriction();
	void readParametersGround(YAML::Node config);

	Eigen::Matrix<double, 3, Dynamic> pointCoords, cpi_up, cpi_down;
	Eigen::Matrix<double, 2, Dynamic> springIndex;
	Eigen::VectorXd materialIndex, spp, sppprev, spd;

	// double uavpos[3], normVe;
	double normVe;
	Vector3d uavpos, we;
	double len, frict[4];
	bool contact, safe;
	int contactPtsNo;
	Vector3d extForce, extTorque;
	Vector3d getForce(const SimState_t states, const WrenchSum_t wrenchSum);
	Vector3d getTorque(const SimState_t states, const WrenchSum_t wrenchSum);
};
