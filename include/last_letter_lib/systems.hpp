#pragma once

#include <Eigen/Eigen>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

#include <last_letter_lib/math_utils.hpp>
#include <last_letter_lib/prog_utils.hpp>
#include <last_letter_lib/uav_utils.hpp>

using Eigen::Quaterniond;
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
    Component(string name) : Parametrized(name)
    {
    }
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
    }
    void update_parameters() override;

    Pose pose;
    Inertial inertial;
};

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
    typedef std::vector<double> state_type;
    state_type x;
    double t{0};

    DynamicSystem(state_type x_0_p, state_type u_0_p, double t_p=0);
    DynamicSystem();
    ~DynamicSystem() {};
    // Carry out any data handling after the time step.
    void step_dynamics(const state_type u, const double dt);
    virtual void post_propagation() {};
    virtual void reset();
    virtual void reset(state_type x_0, state_type u_0, double t=0);

    virtual state_type dynamics(const state_type x, const state_type u, const double t) = 0;
    virtual state_type outputs(const state_type x, const state_type u, const double t) = 0;

protected:
    state_type x_0;
    state_type u, u_0;
    double t_0{0};

private:
    state_type pack_odeint_state(const state_type u_p);
    void unpack_odeint_state(const state_type odeint_x, state_type &x_p, state_type &u_p);

    odeint::runge_kutta4<state_type> stepper;

};


} // end namespace systems
} // end namespace last_letter_lib
