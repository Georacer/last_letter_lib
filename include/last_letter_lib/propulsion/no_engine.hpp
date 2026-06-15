////////////////////////////
// No engine, dummy class //
////////////////////////////

class NoEngine : public Propulsion
{
public:
	NoEngine(string name);
	~NoEngine();

	void updateRadPS(SimState_t states, Inertial inertial, Environment_t environment);
	void getForce(SimState_t states, Inertial inertial, Environment_t environment);
	void getTorque(SimState_t states, Inertial inertial, Environment_t environment);
};
