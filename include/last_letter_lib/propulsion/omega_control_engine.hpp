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
	EngOmegaControl(string name);
	~EngOmegaControl();
    void initialize_parameters() override
    {
        Propulsion::initialize_parameters();

        set_param<double>("prop_diam", 0.28, false);
        set_param<double>("omega_max", 1050, false);

        set_param<double>("thrust_poly/polyType", 0, false);
        set_param<double>("thrust_poly/polyNo", 2, false);
        std::vector<double> thrust_polyCoeffs = {0.1133, -0.0740, -0.1849};
        set_param<vector<double>>("thrust_poly/coeffs", thrust_polyCoeffs, false);

        set_param<double>("power_poly/polyType", 0, false);
        set_param<double>("power_poly/polyNo", 2, false);
        std::vector<double> power_polyCoeffs = {0.0440, 0.0059, -0.0732};
        set_param<vector<double>>("power_poly/coeffs", power_polyCoeffs, false);
    }

    void update_parameters() override;

	void updateRadPS(SimState_t states, Inertial inertial, Environment_t environment); //Step the angular speed
	void getForce(SimState_t states, Inertial inertial, Environment_t environment);
	void getTorque(SimState_t states, Inertial inertial, Environment_t environment);
};
