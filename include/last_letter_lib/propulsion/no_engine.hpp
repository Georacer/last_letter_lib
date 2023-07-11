////////////////////////////
// No engine, dummy class //
////////////////////////////

class NoEngine : public Propulsion
{
public:
	NoEngine(ParameterManager propConfig, ParameterManager worldConfig);
	~NoEngine();

	void updateRadPS(SimState_t states, Inertial inertial, Environment_t environment);
	void getForce(SimState_t states, Inertial inertial, Environment_t environment);
	void getTorque(SimState_t states, Inertial inertial, Environment_t environment);
};
