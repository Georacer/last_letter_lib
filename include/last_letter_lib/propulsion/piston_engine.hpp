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
	PistonEng(string name);
	~PistonEng();
    void initialize_parameters() override
    {
        Propulsion::initialize_parameters();

        // TODO: These defaults have to be replaced with realistic numbers.
        set_param<double>("propDiam", 0.5588, false);
        set_param<double>("engInertia", 0.01, false);
        std::vector<double> radpslimits = {80, 630};
        set_param<vector<double>>("RadPSLimits", radpslimits, false);

        // TODO: Replace this with a YAML string?
        set_param<double>("engPowerPoly/polyType", 0, false);
        set_param<double>("engPowerPoly/polyNo", 2, false);
        std::vector<double> engPowerPolyCoeffs = {0, 0.5, -0.005};
        set_param<vector<double>>("engPowerPoly/coeffs", engPowerPolyCoeffs, false);

        set_param<double>("engCoeffPoly/polyType", 0, false);
        set_param<double>("engCoeffPoly/polyNo", 2, false);
        std::vector<double> engCoeffPolyCoeffs = {0, 0.03, -0.0003};
        set_param<vector<double>>("engCoeffPoly/coeffs", engCoeffPolyCoeffs, false);

        set_param<double>("propPowerPoly/polyType", 0, false);
        set_param<double>("propPowerPoly/polyNo", 1, false);
        std::vector<double> propPowerPolyCoeffs = {-0.0405, 0.0289};
        set_param<vector<double>>("propPowerPoly/coeffs", propPowerPolyCoeffs, false);
    }
    void update_parameters() override;

	void updateRadPS(SimState_t states, Inertial inertial, Environment_t environment);
	void getForce(SimState_t states, Inertial inertial, Environment_t environment);
	void getTorque(SimState_t states, Inertial inertial, Environment_t environment);
};
