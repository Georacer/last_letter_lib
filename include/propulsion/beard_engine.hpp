////////////////////////////////////////////////////
// Electric engine model found in R. Beard's book //
////////////////////////////////////////////////////
class EngBeard: public Propulsion
{
	public:
	///////////
	//Variables
	double s_prop, c_prop, k_motor, k_t_p, k_omega;
	double airspeed, rho;

	///////////
	//Functions
	EngBeard(YAML::Node propConfig, YAML::Node worldConfig);
	~EngBeard();
	void readParametersProp(YAML::Node config);

	void updateRadPS(SimState_t states, Inertial_t inertial, Environment_t environment); //Step the angular speed
	void getForce(SimState_t states, Inertial_t inertial, Environment_t environment);
	void getTorque(SimState_t states, Inertial_t inertial, Environment_t environment);
};
