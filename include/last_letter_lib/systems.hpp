#include <Eigen/Eigen>

#include <last_letter_lib/prog_utils.hpp>
#include <last_letter_lib/uav_utils.hpp>

using Eigen::Quaterniond;
using Eigen::Vector3d;

using namespace last_letter_lib::programming_utils;
using namespace last_letter_lib::uav_utils;

namespace last_letter_lib
{
    namespace systems
    {

        class Component : virtual public Parametrized
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

    } // end namespace systems

} // end namespace last_letter_lib
