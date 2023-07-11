/////////////////////////////////////////////
// Aerodynamics class related declarations //
/////////////////////////////////////////////

#include <Eigen/Eigen>
#include "yaml-cpp/yaml.h"

#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/math_utils.hpp"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/environment.hpp"

using Eigen::Vector3d;

using namespace last_letter_lib::math_utils;
using namespace last_letter_lib::uav_utils;
using last_letter_lib::programming_utils::buildPolynomial;
using last_letter_lib::programming_utils::ParameterManager;

namespace last_letter_lib
{

    //////////////////////////////////////////////
    // Aerodynamics interface class declaration //
    //////////////////////////////////////////////
    class Aerodynamics
    {
    public:
        ////////////
        // Variables
        double airspeed_, alpha_, beta_;           // rotated airdata quantities
        Vector3d relativeWind, relativeRates;      // rotated angular rate container vector
        double p, q, r;                            // rotated angular rates
        double deltaa_max, deltae_max, deltar_max; // Control inputs and maximum surface deflections
        double inputAileron, inputElevator, inputRudder;
        int chanAileron, chanElevator, chanRudder;
        Wrench_t wrenchAero;

        ////////////
        // Functions
    public:
        Aerodynamics(ParameterManager config);
        virtual ~Aerodynamics();
        void setInput(Input_t input);
        void setInputPwm(InputPwm_t input);
        void stepDynamics(const SimState_t states, const Inertial inertial, const Environment_t environment); // perform one step in the aerodynamics
        virtual void readParametersAerodynamics(ParameterManager config);
        virtual void getForce(Environment_t environmet) = 0;
        virtual void getTorque(Environment_t environment) = 0;
    };

    ///////////////////////////////////////////
    // No aerodynamics, create a dummy class //
    ///////////////////////////////////////////
    class NoAerodynamics : public Aerodynamics
    {
    public:
        NoAerodynamics(ParameterManager config);
        ~NoAerodynamics();
        void getForce(Environment_t environment);
        void getTorque(Environment_t environment);
    };

    ////////////////////////////////////////////////////////
    // Typical aerodynamics, linear to their coefficients //
    ////////////////////////////////////////////////////////
    class StdLinearAero : public Aerodynamics
    {
    public:
        StdLinearAero(ParameterManager config);
        ~StdLinearAero();
        double rho, g;
        double c_lift_q, c_lift_deltae, c_drag_q, c_drag_deltae;
        double c, b, s;
        double c_y_0, c_y_b, c_y_p, c_y_r, c_y_deltaa, c_y_deltar;
        double c_l_0, c_l_b, c_l_p, c_l_r, c_l_deltaa, c_l_deltar;
        double c_m_0, c_m_a, c_m_q, c_m_deltae;
        double c_n_0, c_n_b, c_n_p, c_n_r, c_n_deltaa, c_n_deltar;
        double M, alpha0, c_lift_0, c_lift_a0;
        double c_drag_p, oswald, AR;
        virtual void readParametersAerodynamics(ParameterManager config);
        void getForce(Environment_t environment);
        void getTorque(Environment_t environment);
        // Calculate lift coefficient from alpha
        virtual double liftCoeff(double);
        // Calculate drag coefficient from alpha
        virtual double dragCoeff(double);
    };

    /////////////////////////////////////////////////////////////////////
    // Same as StdLinearAero but with linear-in-the-parameters drag model
    /////////////////////////////////////////////////////////////////////
    class SimpleDrag : public StdLinearAero
    {
    public:
        SimpleDrag(ParameterManager config);
        ~SimpleDrag();
        void readParametersAerodynamics(ParameterManager config);
        double c_drag_a;
        // Calculate drag coefficient from alpha
        double dragCoeff(double);
    };

    ////////////////////////////////////////////////////////
    // Extension for fine specification of the drag polar //
    ////////////////////////////////////////////////////////
    class HCUAVAero : public StdLinearAero
    {
    public:
        HCUAVAero(ParameterManager config);
        ~HCUAVAero();
        void readParametersAerodynamics(ParameterManager config);
        math_utils::Polynomial *liftCoeffPoly;
        math_utils::Polynomial *dragCoeffPoly;

        double liftCoeff(double alpha);
        double dragCoeff(double alpha);
    };

    Aerodynamics *buildAerodynamics(ParameterManager config);
} // namespace last_letter_lib
