///////////////////
// Piston engine //
///////////////////

class PistonEng : public Propulsion
{
public:
	////////////
	// Variables
	double omegaMin, omegaMax;
	double propDiam, engInertia, rho;

	//////////
	// Members
	Polynomial *engPowerPoly;
	Polynomial *npPoly;
	Polynomial *propPowerPoly;

	////////////
	// Functions
	PistonEng(ParameterManager propConfig, ParameterManager worldConfig);
	~PistonEng();
	void readParametersProp(ParameterManager config);

	void updateRadPS(SimState_t states, Inertial_t inertial, Environment_t environment);
	void getForce(SimState_t states, Inertial_t inertial, Environment_t environment);
	void getTorque(SimState_t states, Inertial_t inertial, Environment_t environment);
};
