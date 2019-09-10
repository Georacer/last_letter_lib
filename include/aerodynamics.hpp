/////////////////////////////////////////////
// Aerodynamics class related declarations //
/////////////////////////////////////////////

#include <Eigen/Eigen>
#include "yaml-cpp/yaml.h"

#include "uav_utils.hpp"
#include "math_utils.hpp"
#include "environment.hpp"

using Eigen::Vector3d;

//////////////////////////////////////////////
// Aerodynamics interface class declaration //
//////////////////////////////////////////////
class Aerodynamics
{
    public:
    ////////////
    // Variables
    Vector3d CGOffset; // vector from CG to engine coordinates
    Vector3d mountOrientation; // YPR mounting orientation of the wing
    double airspeed, alpha, beta; // rotated airdata quantities
    Vector3d relativeWind, relativeRates; // rotated angular rate container vector
    double p, q, r; // rotated angular rates
    double deltaa_max, deltae_max, deltar_max, gimbalAngle_max; // Control inputs and maximum surface deflections
    double inputAileron, inputElevator, inputRudder, inputGimbal;
    int chanAileron, chanElevator, chanRudder, chanGimbal;
    Wrench_t wrenchAero;

    Eigen::Quaterniond q_bm, q_mg, q_bg; // Quaternion rotations
    Eigen::Transform<double, 3, Eigen::Affine> body_to_mount, mount_to_gimbal, body_to_gimbal; // Transformations in the airfoil assembly for visual rendering
    Eigen::Transform<double, 3, Eigen::Affine> body_to_mount_rot, mount_to_gimbal_rot, body_to_gimbal_rot; // Transformations in the airfoil assembly for force and moment rotation

    ////////////
    // Functions
    Aerodynamics(YAML::Node config);
    ~Aerodynamics();
    void setInput(Input_t input, YAML::Node config);
    void setInputPwm(InputPwm_t input, YAML::Node config);
    void stepDynamics(const SimState_t states, const Inertial_t inertial, const Environment_t environment); // perform one step in the aerodynamics
    void rotateFrame(SimState_t states, Environment_t environment); // convert the wind to the propeller axes
    void rotateForce(); // convert the resulting force to the body axes
    void rotateTorque(Inertial_t inertial); // convert the resulting torque to the body axes
    virtual void getForce(Environment_t environmet) = 0;
    virtual void getTorque(Environment_t environment) = 0;
};


///////////////////////////////////////////
// No aerodynamics, create a dummy class //
///////////////////////////////////////////
class NoAerodynamics : public Aerodynamics{
public:
    NoAerodynamics(YAML::Node config);
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
    StdLinearAero(YAML::Node config);
    ~StdLinearAero();
    double rho,g;
    double c_lift_q, c_lift_deltae, c_drag_q, c_drag_deltae;
    double c,b,s;
    double c_y_0, c_y_b, c_y_p, c_y_r, c_y_deltaa, c_y_deltar;
    double c_l_0, c_l_b, c_l_p, c_l_r, c_l_deltaa, c_l_deltar;
    double c_m_0, c_m_a, c_m_q, c_m_deltae;
    double c_n_0, c_n_b, c_n_p, c_n_r, c_n_deltaa, c_n_deltar;
    double M, alpha0, c_lift_0, c_lift_a0;
    double c_drag_p, oswald, AR;
    void getForce(Environment_t environment);
    void getTorque(Environment_t environment);
    //Calculate lift coefficient from alpha
    virtual double liftCoeff(double);
    //Calculate drag coefficient from alpha
    virtual double dragCoeff(double);
};

////////////////////////////////////////////////////////
// Extension for fine specification of the drag polar //
////////////////////////////////////////////////////////
class HCUAVAero : public StdLinearAero
{
public:
    HCUAVAero(YAML::Node config);
    ~HCUAVAero();
    Polynomial * liftCoeffPoly;
    Polynomial * dragCoeffPoly;

    double liftCoeff(double alpha);
    double dragCoeff(double alpha);
};

Aerodynamics * buildAerodynamics(YAML::Node config);