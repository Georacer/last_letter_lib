#include <last_letter_lib/systems.hpp>

namespace last_letter_lib {
namespace systems {

void Component::update_parameters()
{
    pose.position.x() = get_param<double>("pose/position/x");
    pose.position.y() = get_param<double>("pose/position/y");
    pose.position.z() = get_param<double>("pose/position/z");
    pose.orientation.w() = get_param<double>("pose/orientation/w");
    pose.orientation.x() = get_param<double>("pose/orientation/x");
    pose.orientation.y() = get_param<double>("pose/orientation/y");
    pose.orientation.z() = get_param<double>("pose/orientation/z");
    inertial.mass = get_param<double>("inertial/mass");
    inertial.tensor(0, 0) = get_param<double>("inertial/tensor/j_xx");
    inertial.tensor(1, 1) = get_param<double>("inertial/tensor/j_yy");
    inertial.tensor(2, 2) = get_param<double>("inertial/tensor/j_zz");
}


DynamicSystem::DynamicSystem(state_type x_0_p, state_type u_0_p, double t_p) :
        x_0(x_0_p), u_0(u_0_p), t_0(t_p)
    {
        reset();
    }

void DynamicSystem::reset(std::vector<double> x_0_p, std::vector<double> u_0_p, double t_p)
{
    x_0 = x_0_p;
    u_0 = u_0_p;
    t_0 = t_p;

    x = x_0;
    u = u_0;
    t = t_0;
}

void DynamicSystem::reset()
{
    reset(x_0, u_0, t_0);
}

DynamicSystem::state_type DynamicSystem::pack_odeint_state(const state_type u_p)
{
    state_type odeint_state;
    // Source - https://stackoverflow.com/a/3177252
    // Posted by Kirill V. Lyadvinsky
    // Retrieved 2026-06-24, License - CC BY-SA 2.5
    odeint_state.reserve(x.size() + u.size());
    odeint_state.insert( odeint_state.end(), x.begin(), x.end() );
    odeint_state.insert( odeint_state.end(), u_p.begin(), u_p.end() );

    return odeint_state;
}

void DynamicSystem::unpack_odeint_state(const state_type odeint_x, state_type &x_p, state_type &u_p)
{
    // Source - https://stackoverflow.com/a/421615
    // Posted by Greg Rogers
    // Retrieved 2026-06-24, License - CC BY-SA 2.5
    state_type::const_iterator u_begin = odeint_x.begin() + x_p.size();
    std::copy(odeint_x.begin(), u_begin, x_p.begin());
    std::copy(u_begin, odeint_x.end(), u_p.begin());
}

void DynamicSystem::step_dynamics(const std::vector<double> u, const double dt)
{
    state_type odeint_state = pack_odeint_state(u);
    // Declare a lambda, because do_step requires a static function.
    auto sys = [this, u](const state_type &odeint_state, state_type &dxdt, double t) {
        state_type x_temp{x};
        state_type u_temp{u};
        unpack_odeint_state(odeint_state, x_temp, u_temp);
        dxdt = dynamics(x_temp, u_temp, t);
    };
    stepper.do_step(sys, odeint_state , t, dt);
    state_type throwaway_u{u};
    unpack_odeint_state(odeint_state, x, throwaway_u);
    t += dt;
}

} // end namespace systems
} // end namespace last_letter_lib
