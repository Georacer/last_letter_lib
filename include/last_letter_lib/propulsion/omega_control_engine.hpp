////////////////////////////////////////////////////
// Electric engine model found in R. Beard's book //
////////////////////////////////////////////////////

class EngOmegaControl : public Propulsion
{
public:
	///////////
	//Variables
	double prop_diam, omega_max;
	double airspeed, rho;
	double thrust, torque;
	Polynomial *power_poly, *thrust_poly;

	///////////
	//Functions
	EngOmegaControl(ParameterManager propConfig, ParameterManager worldConfig);
	~EngOmegaControl();
	void readParametersProp(ParameterManager config);

	void updateRadPS(SimState_t states, Inertial_t inertial, Environment_t environment); //Step the angular speed
	void getForce(SimState_t states, Inertial_t inertial, Environment_t environment);
	void getTorque(SimState_t states, Inertial_t inertial, Environment_t environment);
};
