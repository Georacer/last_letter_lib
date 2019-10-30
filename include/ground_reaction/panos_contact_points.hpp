/// Panos Marantos ground reactions implementation

using Eigen::Dynamic;

class PanosContactPoints : public GroundReaction
{
	public:
	PanosContactPoints(YAML::Node config, YAML::Node worldConfig);
	~PanosContactPoints();
	void readParametersGround(YAML::Node config);

	Eigen::Matrix<double, 3, Dynamic> pointCoords, cpi_up, cpi_down;
	Eigen::Matrix<double, 2, Dynamic> springIndex;
	Eigen::VectorXd materialIndex, spp, sppprev, spd;

	double normVe;
	Vector3d uavpos, we;
	double len, frictForw[4], frictSide[4];
	bool contact, safe;
	int contactPtsNo;
	Vector3d getForce(const SimState_t states, const WrenchSum_t wrenchSum);
	Vector3d getTorque(const SimState_t states, const WrenchSum_t wrenchSum);
};