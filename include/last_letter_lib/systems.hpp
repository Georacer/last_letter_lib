#pragma once

#include <Eigen/Eigen>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

#include <last_letter_lib/environment.hpp>
#include <last_letter_lib/gravity.hpp>
#include <last_letter_lib/math_utils.hpp>
#include <last_letter_lib/prog_utils.hpp>
#include <last_letter_lib/uav_utils.hpp>

using Eigen::Vector3d;
using namespace boost::numeric;

using namespace last_letter_lib::math_utils;
using namespace last_letter_lib::programming_utils;
using namespace last_letter_lib::uav_utils;

namespace last_letter_lib {
namespace systems {

class Component : public Parametrized
{
public:
    Component(string name) : Parametrized(name) {};

    void initialize_parameters() override
    {
        set_param("pose/position/x", 0.0, false);
        set_param("pose/position/y", 0.0, false);
        set_param("pose/position/z", 0.0, false);
        set_param("pose/orientation/w", 1.0, false);
        set_param("pose/orientation/x", 0.0, false);
        set_param("pose/orientation/y", 0.0, false);
        set_param("pose/orientation/z", 0.0, false);
        set_param("inertial/mass", 0.0, false);
        set_param("inertial/tensor/j_xx", 0.0, false);
        set_param("inertial/tensor/j_yy", 0.0, false);
        set_param("inertial/tensor/j_zz", 0.0, false);
        set_param("world/gravity/type", 0.0, false);
    }
    void update_parameters() override;
    // Update the local state given the aircraft state.
    // body_state is describing the body frame in the NED coordinate system.
    // environment is describing the wind in the NED coordinate system.
    void update_local_state(const SimState_t body_state, const Environment_t environment);
    virtual void calc_model(); // Perform any model calculations.

    WrenchSum_t wrench_sum; // The generated wrench in the local frame.
    Inertial inertial;
    SimState_t local_state; // W.r.t. Earth. This also contains rotor speeds, which is bloat, but we'll use it for now.
    Environment_t local_environment;

private:
    Pose relative_pose; // The pose of this component w.r.t. the airframe datum. Its quaternion is q_bc (component->body)
    GravityModel *gravity{nullptr};
};

typedef std::vector<double> stateType;

/* Class to implement arbitrary time-variant dynamic systems.
 * It is meant to be (optionally) multiple-inherited alongside Parametrized,
 * so it should not duplicate any of its functionality.
 *
 * DynamicSystems should handle state intialization, propagation, reset and
 * define the system derivative, while Parametrized handles updating of system
 * parameters.
*/
class DynamicSystem
{
public:
    stateType x;
    double t{0};

    DynamicSystem(stateType x_0_p, stateType u_0_p, double t_p=0);
    DynamicSystem();
    ~DynamicSystem() {};
    // Carry out any data handling after the time step.
    void step_dynamics(const stateType u, const double dt);
    virtual void reset();
    virtual void reset(stateType x_0, stateType u_0, double t=0);

    virtual stateType dynamics(const stateType x, const stateType u, const double t) = 0;
    virtual stateType outputs(const stateType x, const stateType u, const double t) = 0;

protected:
    stateType x_0;
    stateType u, u_0;
    double t_0{0};

    virtual void pre_propagation() {}; // Actions to take before stepping the dynaimcs.
    virtual void post_propagation() {}; // Actions to take after stepping the dynaimcs.

private:
    stateType pack_odeint_state(const stateType u_p);
    void unpack_odeint_state(const stateType odeint_x, stateType &x_p, stateType &u_p);

    odeint::runge_kutta4<stateType> stepper;

};


} // end namespace systems
} // end namespace last_letter_lib
