///////////////////////////////////////////
// Propulsion class related declarations //
///////////////////////////////////////////

#pragma once

#include <Eigen/Eigen>
#include <string>
#include "yaml-cpp/yaml.h"

#include "last_letter_lib/environment.hpp"
#include "last_letter_lib/math_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/prog_utils.hpp"

////////////////////////////////////////////
// Propulsion interface class declaration //
////////////////////////////////////////////

using Eigen::Vector3d;
using namespace last_letter_lib::uav_utils;
using last_letter_lib::math_utils::Inertial;
using last_letter_lib::math_utils::Polynomial;
using last_letter_lib::programming_utils::buildPolynomial;
using last_letter_lib::programming_utils::ParameterManager;

namespace last_letter_lib
{
namespace propulsion
{

class Propulsion : public Parametrized
{
public:
    ///////////
    // Variables
    double dt;          // Simulation time step
    double inputMotor;  // control input (0-1)
    int chanMotor;
    double omega;       // motor angular speed in rad/s
    double rotationDir; // motor direction of rotation
    double theta;       // propeller angle in rads
    double normalWind;  // scalar wind normal to propeller disc
    Vector3d relativeWind;
    Wrench_t wrenchProp;

    ///////////
    // Functions
    Propulsion(string name);
    virtual ~Propulsion();

    void initialize_parameters() override
    {
        set_param<double>("deltaT", 0.0025, false);
        set_param<double>("rotationDir", 0, false);
        set_param<int>("chanMotor", 0, false);
        set_param<int>("motorType", 0, false);
    }
    void update_parameters() override;

    void setInput(Input input);                                                                     // store control input
    void setInputPwm(InputPwm_t input);                                                             // store PWM control input
    void stepEngine(SimState_t states, Inertial inertial, Environment_t environment);               // engine physics step, container for the generic class
    virtual void updateRadPS(SimState_t states, Inertial inertial, Environment_t environment) = 0;  // Step the angular speed
    void rotateProp();                                                                              // Update the propeller angle
    virtual void getForce(SimState_t states, Inertial inertial, Environment_t environment) = 0;     // Calculate Forces
    virtual void getTorque(SimState_t states, Inertial inertial, Environment_t environment) = 0;    // Calculate Torques
};

class NoEngine : public Propulsion
{
public:
	NoEngine(string name);
	~NoEngine();

	void updateRadPS(SimState_t states, Inertial inertial, Environment_t environment);
	void getForce(SimState_t states, Inertial inertial, Environment_t environment);
	void getTorque(SimState_t states, Inertial inertial, Environment_t environment);
};

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

Propulsion *buildPropulsion(ParameterManager propConfig);

} // namespace propulsion
} // namespace last_letter_lib
