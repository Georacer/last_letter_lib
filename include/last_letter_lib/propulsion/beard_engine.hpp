////////////////////////////////////////////////////
// Electric engine model found in R. Beard's book //
////////////////////////////////////////////////////

class EngBeard : public Propulsion
{
public:
	///////////
	//Variables
	double s_prop, c_prop, k_motor, k_t_p, k_omega;
	double rho;

	///////////
	//Functions
	EngBeard(string name);
	~EngBeard();
    void initialize_parameters() override
    {
        Propulsion::initialize_parameters();

        set_param<double>("s_prop", 1.0, false);
        set_param<double>("c_prop", 0.33, false);
        set_param<double>("k_motor", 30, false);
        set_param<double>("k_t_p", 1e-6, false);
        set_param<double>("k_omega", 800, false);
    }

    void update_parameters() override;

	void updateRadPS(SimState_t states, Inertial inertial, Environment_t environment); //Step the angular speed
	void getForce(SimState_t states, Inertial inertial, Environment_t environment);
	void getTorque(SimState_t states, Inertial inertial, Environment_t environment);
};
