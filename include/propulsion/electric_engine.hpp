///////////////////////////
// Electric hobby engine //
///////////////////////////
class ElectricEng : public Propulsion
{
public:
	////////////////
	// Variables //
	////////////////
	double omegaMin, omegaMax;
	double propDiam, engInertia, rho;
	double Kv, Rm, I0;
	// Battery specification
	int Cells; // Number of LiPo cells
	double Rs; // Battery internal resistance
	// TODO: Find where the message needs to be introduced in the ROS wrapper
	// last_letter_msgs::ElectricEng message;
	// ros::Publisher pub;

	//////////////
	// Members //
	//////////////
	Polynomial * engPowerPoly, * npPoly, * propPowerPoly;

	////////////////
	// Functions //
	////////////////
	ElectricEng(YAML::Node propConfig, YAML::Node worldConfig);
	~ElectricEng();

	void updateRadPS(SimState_t states, Inertial_t inertial, Environment_t environment);
	void getForce(SimState_t states, Inertial_t inertial, Environment_t environment);
	void getTorque(SimState_t states, Inertial_t inertial, Environment_t environment);
};