#include <last_letter_lib/systems.hpp>

namespace last_letter_lib
{
    namespace systems
    {

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

    } // end namespace systems

} // end namespace last_letter_lib
