
#include "last_letter_lib/aerodynamics.hpp"
#include "last_letter_lib/prog_utils.hpp"

using namespace std;

using Eigen::Quaterniond;

namespace last_letter_lib
{

	//////////////////////////
	// Define Aerodynamics class
	//////////////////////////

	// Aerodynamics::Aerodynamics(ModelPlane * parent, int ID)
	// Pass the ID-filtered aerodynamics config YAML::Node
	Aerodynamics::Aerodynamics(YAML::Node config)
	{
		readParametersAerodynamics(config);
	}

	Aerodynamics::~Aerodynamics()
	{
	}

	void Aerodynamics::readParametersAerodynamics(YAML::Node config)
	{
		vector<double> doubleVect;

		if (!getParameter(config, "chanAileron", chanAileron, false))
		{
			chanAileron = -1;
		}
		if (!getParameter(config, "chanElevator", chanElevator, false))
		{
			chanElevator = -1;
		}
		if (!getParameter(config, "chanRudder", chanRudder, false))
		{
			chanRudder = -1;
		}

		getParameter(config, "deltaa_max", deltaa_max);
		getParameter(config, "deltae_max", deltae_max);
		getParameter(config, "deltar_max", deltar_max);

		inputAileron = 0.0;
		inputElevator = 0.0;
		inputRudder = 0.0;
	}

	void Aerodynamics::setInput(Input_t input)
	{
		//Convert -1 - 1 input range to radians
		if (chanAileron > -1)
		{
			inputAileron = deltaa_max * input.value[chanAileron];
		}
		if (chanElevator > -1)
		{
			inputElevator = deltae_max * input.value[chanElevator];
		}
		if (chanRudder > -1)
		{
			inputRudder = deltar_max * input.value[chanRudder];
		}
	}

	void Aerodynamics::setInputPwm(InputPwm_t p_input)
	{
		// Convert PPM to -1 - 1 range
		Input_t input;
		if (chanAileron > -1)
		{
			input.value[chanAileron] = PwmToFullRange(p_input.value[chanAileron]);
		}
		if (chanElevator > -1)
		{
			input.value[chanElevator] = PwmToFullRange(p_input.value[chanElevator]);
		}
		if (chanRudder > -1)
		{
			input.value[chanRudder] = PwmToFullRange(p_input.value[chanRudder]);
		}

		// Call normalized input setter
		setInput(input);
	}

	// One step in the physics engine
	void Aerodynamics::stepDynamics(const SimState_t states, const Inertial_t /*inertial*/, const Environment_t environment)
	{
		Airdata airdata;
		// std::cout << "received states and wind: \n"
		// 		  << states.velocity.linear << "\n"
		// 		  << environment.wind << "\n"
		// 		  << std::endl;

		airdata.calcAirData(states.velocity.linear, environment.wind);
		// Calculate the new, relative air data
		airspeed_ = airdata.airspeed;
		alpha_ = airdata.alpha;
		beta_ = airdata.beta;

		// std::cout << "Calculated airdata: \n"
		// 		  << airspeed_ << "\n"
		// 		  << alpha_ << "\n"
		// 		  << beta_ << "\n"
		// 		  << std::endl;

		getForce(environment);
		getTorque(environment);

		// std::cout << "Calculated local aeroforce: \n"
		// 		  << wrenchAero.force << std::endl;
	}

	/////////////////////////////
	// Define NoAerodynamics class
	/////////////////////////////

	// Class constructor
	NoAerodynamics::NoAerodynamics(YAML::Node config) : Aerodynamics(config)
	{
	}

	// Class destructor
	NoAerodynamics::~NoAerodynamics()
	{
	}

	// Force calculation function
	void NoAerodynamics::getForce(Environment_t /* environment */)
	{
	}

	// Torque calculation function
	void NoAerodynamics::getTorque(Environment_t /* environment */)
	{
	}

	/////////////////////////////
	// Define StdLinearAero class
	/////////////////////////////

	// Class constructor
	StdLinearAero::StdLinearAero(YAML::Node config) : Aerodynamics(config)
	{
		readParametersAerodynamics(config);
	}

	// Class destructor
	StdLinearAero::~StdLinearAero()
	{
	}

	void StdLinearAero::readParametersAerodynamics(YAML::Node config)
	{
		Aerodynamics::readParametersAerodynamics(config);
		// Read aerodynamic coefficients from parameter server
		getParameter(config, "c", c);
		getParameter(config, "b", b);
		getParameter(config, "s", s);
		getParameter(config, "c_L_0", c_lift_0);
		getParameter(config, "c_L_alpha", c_lift_a0);
		getParameter(config, "c_L_qn", c_lift_q);
		getParameter(config, "c_L_deltae", c_lift_deltae);
		getParameter(config, "c_D_qn", c_drag_q);
		getParameter(config, "c_D_0", c_drag_p);
		getParameter(config, "c_D_deltae", c_drag_deltae);
		getParameter(config, "c_Y_0", c_y_0);
		getParameter(config, "c_Y_beta", c_y_b);
		getParameter(config, "c_Y_pn", c_y_p);
		getParameter(config, "c_Y_rn", c_y_r);
		getParameter(config, "c_Y_deltaa", c_y_deltaa);
		getParameter(config, "c_Y_deltar", c_y_deltar);
		getParameter(config, "c_l_0", c_l_0);
		getParameter(config, "c_l_beta", c_l_b);
		getParameter(config, "c_l_pn", c_l_p);
		getParameter(config, "c_l_rn", c_l_r);
		getParameter(config, "c_l_deltaa", c_l_deltaa);
		getParameter(config, "c_l_deltar", c_l_deltar);
		getParameter(config, "c_m_0", c_m_0);
		getParameter(config, "c_m_alpha", c_m_a);
		getParameter(config, "c_m_qn", c_m_q);
		getParameter(config, "c_m_deltae", c_m_deltae);
		getParameter(config, "c_n_0", c_n_0);
		getParameter(config, "c_n_beta", c_n_b);
		getParameter(config, "c_n_pn", c_n_p);
		getParameter(config, "c_n_rn", c_n_r);
		getParameter(config, "c_n_deltaa", c_n_deltaa);
		getParameter(config, "c_n_deltar", c_n_deltar);
		getParameter(config, "oswald", oswald);
		getParameter(config, "mcoeff", M);
		getParameter(config, "alpha_stall", alpha0);
	}

	// Force calculation function
	void StdLinearAero::getForce(Environment_t environment)
	{
		// Read air density
		rho = environment.density;

		//request lift and drag alpha-coefficients from the corresponding functions
		double c_lift_a = liftCoeff(alpha_);
		double c_drag_a = dragCoeff(alpha_);

		// std::cout << "drag_coeff: " << alpha_ << ", " << c_drag_a << std::endl;

		//convert coefficients to the body frame
		double c_x_a = -c_drag_a * cos(alpha_) + c_lift_a * sin(alpha_);
		double c_x_q = -c_drag_q * cos(alpha_) + c_lift_q * sin(alpha_);
		double c_z_a = -c_drag_a * sin(alpha_) - c_lift_a * cos(alpha_);
		double c_z_q = -c_drag_q * sin(alpha_) - c_lift_q * cos(alpha_);

		// std::cout << "x coeffs: " << c_x_a << ", " << c_x_q << std::endl;

		//calculate aerodynamic force
		double qbar = 1.0 / 2.0 * rho * pow(airspeed_, 2) * s; //Calculate dynamic pressure
		double ax, ay, az;
		if (airspeed_ == 0)
		{
			ax = 0;
			ay = 0;
			az = 0;
		}
		else
		{
			ax = qbar * (c_x_a + c_x_q * c * q / (2 * airspeed_) - c_drag_deltae * cos(alpha_) * fabs(inputElevator) + c_lift_deltae * sin(alpha_) * inputElevator);
			// split c_x_deltae to include "abs" term
			ay = qbar * (c_y_0 + c_y_b * beta_ + c_y_p * b * p / (2 * airspeed_) + c_y_r * b * r / (2 * airspeed_) + c_y_deltaa * inputAileron + c_y_deltar * inputRudder);
			az = qbar * (c_z_a + c_z_q * c * q / (2 * airspeed_) - c_drag_deltae * sin(alpha_) * fabs(inputElevator) - c_lift_deltae * cos(alpha_) * inputElevator);
			// split c_z_deltae to include "abs" term
		}

		// std::cout << "drag force: " << ax << std::endl;

		wrenchAero.force = Vector3d(ax, ay, az);
	}

	// Torque calculation function
	void StdLinearAero::getTorque(Environment_t environment)
	{
		// Read air density
		rho = environment.density;

		//calculate aerodynamic torque
		double qbar = 1.0 / 2.0 * rho * pow(airspeed_, 2) * s; //Calculate dynamic pressure
		double la, na, ma;
		if (airspeed_ == 0)
		{
			la = 0;
			ma = 0;
			na = 0;
		}
		else
		{
			la = qbar * b * (c_l_0 + c_l_b * beta_ + c_l_p * b * p / (2 * airspeed_) + c_l_r * b * r / (2 * airspeed_) + c_l_deltaa * inputAileron + c_l_deltar * inputRudder);
			ma = qbar * c * (c_m_0 + c_m_a * alpha_ + c_m_q * c * q / (2 * airspeed_) + c_m_deltae * inputElevator);
			na = qbar * b * (c_n_0 + c_n_b * beta_ + c_n_p * b * p / (2 * airspeed_) + c_n_r * b * r / (2 * airspeed_) + c_n_deltaa * inputAileron + c_n_deltar * inputRudder);
		}

		wrenchAero.torque = Vector3d(la, ma, na);

		// Removing torque calculation, because the CG effect is now being taken into account in the 'rotateTorque' function
	}

	//////////////////////////
	//C_lift_alpha calculation
	double StdLinearAero::liftCoeff(double alpha)
	{
		double sigmoid = (1 + exp(-M * (alpha - alpha0)) + exp(M * (alpha + alpha0))) / (1 + exp(-M * (alpha - alpha0))) / (1 + exp(M * (alpha + alpha0)));
		double linear = (1.0 - sigmoid) * (c_lift_0 + c_lift_a0 * alpha);						 //Lift at small AoA
		double flatPlate = sigmoid * (2 * copysign(1, alpha) * pow(sin(alpha), 2) * cos(alpha)); //Lift beyond stall

		// std::cout << c_lift_0 << " " << c_lift_a0 << std::endl;

		double result = linear + flatPlate;
		return result;
	}

	//////////////////////////
	//C_drag_alpha calculation
	double StdLinearAero::dragCoeff(double alpha)
	{
		AR = pow(b, 2) / s;
		double c_drag_a = c_drag_p + pow(c_lift_0 + c_lift_a0 * alpha, 2) / (M_PI * oswald * AR);

		return c_drag_a;
	}

	/////////////////////////
	// Define HCUAVAero class
	/////////////////////////

	// Class constructor
	HCUAVAero::HCUAVAero(YAML::Node config) : StdLinearAero(config)
	{
		readParametersAerodynamics(config);
	}

	// Class destructor
	HCUAVAero::~HCUAVAero()
	{
	}

	void HCUAVAero::readParametersAerodynamics(YAML::Node config)
	{
		StdLinearAero::readParametersAerodynamics(config);

		// Create CLift polynomial
		YAML::Node liftPolyConfig = filterConfig(config, "cLiftPoly");
		liftCoeffPoly = programming_utils::buildPolynomial(liftPolyConfig);
		// Create CDrag polynomial
		YAML::Node dragPolyConfig = filterConfig(config, "cDragPoly");
		dragCoeffPoly = programming_utils::buildPolynomial(dragPolyConfig);
	}

	//////////////////////////
	//C_lift_alpha calculation
	double HCUAVAero::liftCoeff(double alpha)
	{
		return liftCoeffPoly->evaluate(alpha);
	}

	//////////////////////////
	//C_drag_alpha_ calculation
	double HCUAVAero::dragCoeff(double alpha)
	{
		return dragCoeffPoly->evaluate(alpha);
	}

	//////////////////////////
	// Define SimpleDrag class
	//////////////////////////
	SimpleDrag::SimpleDrag(YAML::Node config) : StdLinearAero(config)
	{
		readParametersAerodynamics(config);
	}

	SimpleDrag::~SimpleDrag()
	{
	}

	void SimpleDrag::readParametersAerodynamics(YAML::Node config)
	{
		StdLinearAero::readParametersAerodynamics(config);
		getParameter(config, "c_D_alpha", c_drag_a);
	}

	double SimpleDrag::dragCoeff(double alpha)
	{
		return c_drag_p + c_drag_a * fabs(alpha);
	}

	// Build aerodynamics model
	Aerodynamics *buildAerodynamics(YAML::Node config)
	{
		int aerodynamicsType;
		getParameter(config, "aerodynamicsType", aerodynamicsType);
		std::cout << "building aerodynamics model type " << aerodynamicsType << ":\n";
		switch (aerodynamicsType)
		{
		case 0:
			std::cout << "selecting no aerodynamics model" << std::endl;
			return new NoAerodynamics(config);
		case 1:
			std::cout << "selecting StdLinearAero aerodynamics" << std::endl;
			return new StdLinearAero(config);
		case 2:
			std::cout << "selecting HCUAVAero aerodynamics" << std::endl;
			return new HCUAVAero(config);
		case 3:
			std::cout << "selecting simplified drag aerodynamics" << std::endl;
			return new SimpleDrag(config);
		default:
			stringstream ss;
			ss << "Error while constructing aerodynamics: Unknown type " << aerodynamicsType;
			throw runtime_error(ss.str());
			break;
		}
	}
} // namespace last_letter_lib
