#include <stdexcept>
#include "last_letter_lib/aerodynamics.hpp"
#include "last_letter_lib/prog_utils.hpp"

using namespace std;
using last_letter_lib::programming_utils::ParameterManager;

using Eigen::Quaterniond;

namespace last_letter_lib
{

	//////////////////////////
	// Define Aerodynamics class
	//////////////////////////

	// Aerodynamics::Aerodynamics(ModelPlane * parent, int ID)
	// Pass the ID-filtered aerodynamics config ParameterManager
	Aerodynamics::Aerodynamics(ParameterManager config)
	{
		readParametersAerodynamics(config);
	}

	Aerodynamics::~Aerodynamics()
	{
	}

	void Aerodynamics::readParametersAerodynamics(ParameterManager config)
	{
		vector<double> doubleVect;

		try {chanAileron = config.get<int>("chanAileron");}
		catch (const std::exception&) { chanAileron = -1;}
		try {chanElevator = config.get<int>("chanElevator");}
		catch (const std::exception&) { chanElevator = -1;}
		try {chanRudder = config.get<int>("chanRudder");}
		catch (const std::exception&) { chanRudder = -1;}

		deltaa_max = config.get<double>("deltaa_max");
		deltae_max = config.get<double>("deltae_max");
		deltar_max = config.get<double>("deltar_max");

		inputAileron = 0.0;
		inputElevator = 0.0;
		inputRudder = 0.0;
	}

	void Aerodynamics::setInput(Input_t input)
	{
		// Convert -1 - 1 input range to radians
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
		p = states.velocity.angular(0);
		q = states.velocity.angular(1);
		r = states.velocity.angular(2);

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
	NoAerodynamics::NoAerodynamics(ParameterManager config) : Aerodynamics(config)
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
	StdLinearAero::StdLinearAero(ParameterManager config) : Aerodynamics(config)
	{
		readParametersAerodynamics(config);
	}

	// Class destructor
	StdLinearAero::~StdLinearAero()
	{
	}

	void StdLinearAero::readParametersAerodynamics(ParameterManager config)
	{
		Aerodynamics::readParametersAerodynamics(config);
		// Read aerodynamic coefficients from parameter server
		c = config.get<double>("c");
		b = config.get<double>("b");
		s = config.get<double>("s");
		c_lift_0 = config.get<double>("c_L_0");
		c_lift_a0 = config.get<double>("c_L_alpha");
		c_lift_q = config.get<double>("c_L_qn");
		c_lift_deltae = config.get<double>("c_L_deltae");
		c_drag_q = config.get<double>("c_D_qn");
		c_drag_p = config.get<double>("c_D_0");
		c_drag_deltae = config.get<double>("c_D_deltae");
		c_y_0 = config.get<double>("c_Y_0");
		c_y_b = config.get<double>("c_Y_beta");
		c_y_p = config.get<double>("c_Y_pn");
		c_y_r = config.get<double>("c_Y_rn");
		c_y_deltaa = config.get<double>("c_Y_deltaa");
		c_y_deltar = config.get<double>("c_Y_deltar");
		c_l_0 = config.get<double>("c_l_0");
		c_l_b = config.get<double>("c_l_beta");
		c_l_p = config.get<double>("c_l_pn");
		c_l_r = config.get<double>("c_l_rn");
		c_l_deltaa = config.get<double>("c_l_deltaa");
		c_l_deltar = config.get<double>("c_l_deltar");
		c_m_0 = config.get<double>("c_m_0");
		c_m_a = config.get<double>("c_m_alpha");
		c_m_q = config.get<double>("c_m_qn");
		c_m_deltae = config.get<double>("c_m_deltae");
		c_n_0 = config.get<double>("c_n_0");
		c_n_b = config.get<double>("c_n_beta");
		c_n_p = config.get<double>("c_n_pn");
		c_n_r = config.get<double>("c_n_rn");
		c_n_deltaa = config.get<double>("c_n_deltaa");
		c_n_deltar = config.get<double>("c_n_deltar");
		oswald = config.get<double>("oswald");
		M = config.get<double>("mcoeff");
		alpha0 = config.get<double>("alpha_stall");
	}

	// Force calculation function
	void StdLinearAero::getForce(Environment_t environment)
	{
		// Read air density
		rho = environment.density;

		// request lift and drag alpha-coefficients from the corresponding functions
		double c_lift_a = liftCoeff(alpha_);
		double c_drag_a = dragCoeff(alpha_);

		// std::cout << "drag_coeff: " << alpha_ << ", " << c_drag_a << std::endl;

		// convert coefficients to the body frame
		double c_x_a = -c_drag_a * cos(alpha_) + c_lift_a * sin(alpha_);
		double c_x_q = -c_drag_q * cos(alpha_) + c_lift_q * sin(alpha_);
		double c_z_a = -c_drag_a * sin(alpha_) - c_lift_a * cos(alpha_);
		double c_z_q = -c_drag_q * sin(alpha_) - c_lift_q * cos(alpha_);

		// std::cout << "x coeffs: " << c_x_a << ", " << c_x_q << std::endl;

		// calculate aerodynamic force
		double qbar = 1.0 / 2.0 * rho * pow(airspeed_, 2) * s; // Calculate dynamic pressure
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

		// calculate aerodynamic torque
		double qbar = 1.0 / 2.0 * rho * pow(airspeed_, 2) * s; // Calculate dynamic pressure
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
	// C_lift_alpha calculation
	double StdLinearAero::liftCoeff(double alpha)
	{
		double sigmoid = (1 + exp(-M * (alpha - alpha0)) + exp(M * (alpha + alpha0))) / (1 + exp(-M * (alpha - alpha0))) / (1 + exp(M * (alpha + alpha0)));
		double linear = (1.0 - sigmoid) * (c_lift_0 + c_lift_a0 * alpha);						 // Lift at small AoA
		double flatPlate = sigmoid * (2 * copysign(1, alpha) * pow(sin(alpha), 2) * cos(alpha)); // Lift beyond stall

		// std::cout << c_lift_0 << " " << c_lift_a0 << std::endl;

		double result = linear + flatPlate;
		return result;
	}

	//////////////////////////
	// C_drag_alpha calculation
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
	HCUAVAero::HCUAVAero(ParameterManager config) : StdLinearAero(config)
	{
		readParametersAerodynamics(config);
	}

	// Class destructor
	HCUAVAero::~HCUAVAero()
	{
	}

	void HCUAVAero::readParametersAerodynamics(ParameterManager config)
	{
		StdLinearAero::readParametersAerodynamics(config);

		// Create CLift polynomial
		ParameterManager liftPolyConfig = config.filter("cLiftPoly");
		liftCoeffPoly = programming_utils::buildPolynomial(liftPolyConfig);
		// Create CDrag polynomial
		ParameterManager dragPolyConfig = config.filter("cDragPoly");
		dragCoeffPoly = programming_utils::buildPolynomial(dragPolyConfig);
	}

	//////////////////////////
	// C_lift_alpha calculation
	double HCUAVAero::liftCoeff(double alpha)
	{
		return liftCoeffPoly->evaluate(alpha);
	}

	//////////////////////////
	// C_drag_alpha_ calculation
	double HCUAVAero::dragCoeff(double alpha)
	{
		return dragCoeffPoly->evaluate(alpha);
	}

	//////////////////////////
	// Define SimpleDrag class
	//////////////////////////
	SimpleDrag::SimpleDrag(ParameterManager config) : StdLinearAero(config)
	{
		readParametersAerodynamics(config);
	}

	SimpleDrag::~SimpleDrag()
	{
	}

	void SimpleDrag::readParametersAerodynamics(ParameterManager config)
	{
		StdLinearAero::readParametersAerodynamics(config);
		c_drag_a = config.get<double>("c_D_alpha");
	}

	double SimpleDrag::dragCoeff(double alpha)
	{
		return c_drag_p + c_drag_a * fabs(alpha);
	}

	// Build aerodynamics model
	Aerodynamics *buildAerodynamics(ParameterManager config)
	{
		int aerodynamicsType;
		aerodynamicsType = config.get<double>("aerodynamicsType");
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
