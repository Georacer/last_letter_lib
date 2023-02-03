////////////////////////////
// No engine, dummy class //
////////////////////////////

class NoEngine : public Propulsion
{
public:
	NoEngine(YAML::Node propConfig, YAML::Node worldConfig);
	~NoEngine();

	void updateRadPS(SimState_t states, Inertial_t inertial, Environment_t environment);
	void getForce(SimState_t states, Inertial_t inertial, Environment_t environment);
	void getTorque(SimState_t states, Inertial_t inertial, Environment_t environment);
};
