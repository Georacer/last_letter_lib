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
	Polynomial *npPoly, *propPowerPoly;

	////////////////
	// Functions //
	////////////////
	ElectricEng(string name);
	~ElectricEng();
    void initialize_parameters() override
    {
        Propulsion::initialize_parameters();

        set_param<double>("propDiam", 0.4064, false);
        set_param<double>("engInertia", 200e-6, false);
        set_param<double>("Kv", 8.17, false);
        set_param<double>("Rm", 0.1, false);
        set_param<double>("Cells", 6, false);
        set_param<double>("I0", 0.5, false);
        std::vector<double> radpslimits = {0.01, 1000};
        set_param<vector<double>>("RadPSLimits", radpslimits, false);

        // TODO: Need realistic coefficients here.
        set_param<double>("nCoeffPoly/polyType", 0, false);
        set_param<double>("nCoeffPoly/polyNo", 2, false);
        std::vector<double> nCoeffPolyCoeffs = {0, 0.03, -0.0003};
        set_param<vector<double>>("nCoeffPoly/coeffs", nCoeffPolyCoeffs, false);

        set_param<double>("propPowerPoly/polyType", 0, false);
        set_param<double>("propPowerPoly/polyNo", 1, false);
        std::vector<double> propPowerPolyCoeffs = {-0.00405, 0.00289};
        set_param<vector<double>>("propPowerPoly/coeffs", propPowerPolyCoeffs, false);
    }
	void update_parameters();

	void updateRadPS(SimState_t states, Inertial inertial, Environment_t environment);
	void getForce(SimState_t states, Inertial inertial, Environment_t environment);
	void getTorque(SimState_t states, Inertial inertial, Environment_t environment);
};

class ElectricEng2 : public Propulsion
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
	double momentumDragCoeff;
	// TODO: Find where the message needs to be introduced in the ROS wrapper
	// last_letter_msgs::ElectricEng message;
	// ros::Publisher pub;

	//////////////
	// Members //
	//////////////
	Polynomial *propThrustPoly, *propPowerPoly;
	double propThrustMultiplier;

	////////////////
	// Functions //
	////////////////
	ElectricEng2(string name);
	~ElectricEng2();
    void initialize_parameters() override
    {
        Propulsion::initialize_parameters();

        set_param<double>("propDiam", 0.4064, false);
        set_param<double>("engInertia", 200e-6, false);
        set_param<double>("Kv", 8.17, false);
        set_param<double>("Rm", 0.1, false);
        set_param<double>("Cells", 6, false);
        set_param<double>("I0", 0.5, false);
        std::vector<double> radpslimits = {0.01, 1000};
        set_param<vector<double>>("RadPSLimits", radpslimits, false);

        set_param<double>("propThrustPoly/polyType", 0, false);
        set_param<double>("propThrustPoly/polyNo", 5, false);
        std::vector<double> propThrustPolyCoeffs = {0.0737, -0.4778, 2.2161, -5.5296, 6.2749, -2.6331};
        set_param<vector<double>>("propThrustPoly/coeffs", propThrustPolyCoeffs, false);

        set_param<double>("propThrustMultiplier", 1.1, false);
        set_param<double>("propPowerPoly/polyType", 0, false);
        set_param<double>("propPowerPoly/polyNo", 1, false);
        std::vector<double> propPowerPolyCoeffs = {0.00289, -0.00405};
        set_param<vector<double>>("propPowerPoly/coeffs", propPowerPolyCoeffs, false);

        set_param<double>("momentumDragCoeff", 0.0047, false);
    }
	void update_parameters();

	void updateRadPS(SimState_t states, Inertial inertial, Environment_t environment);
	void getForce(SimState_t states, Inertial inertial, Environment_t environment);
	void getTorque(SimState_t states, Inertial inertial, Environment_t environment);
};
