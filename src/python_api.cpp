#define PYBIND11_DETAILED_ERROR_MESSAGES

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <iostream>

#include "last_letter_lib/math_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/environment.hpp"
#include "last_letter_lib/systems.hpp"

namespace py = pybind11;

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

using last_letter_lib::Environment_t;
using last_letter_lib::math_utils::EulerAngles;
using last_letter_lib::math_utils::Inertial;
using last_letter_lib::math_utils::UnitQuaternion;
using last_letter_lib::math_utils::Vector3;
using last_letter_lib::programming_utils::Parametrized;
using last_letter_lib::systems::Component;
using last_letter_lib::uav_utils::Airdata;
using last_letter_lib::uav_utils::Input;
using last_letter_lib::uav_utils::Pose;
using last_letter_lib::uav_utils::SimState_t;
using last_letter_lib::uav_utils::Wrench_t;

class PyParametrized : public Parametrized
{
public:
    // Inherit the constructor
    using Parametrized::Parametrized;

    // Create trampolines for each virtual function.
    void initialize_parameters() override
    {
        PYBIND11_OVERRIDE(
            void,                  // Return type
            Parametrized,          // Parent class
            initialize_parameters, // Name of function in C++ (must match Python name)
            // Argument(s)
        );
    }
    void update_parameters() override
    {
        PYBIND11_OVERRIDE_PURE(
            void,              // Return type
            Parametrized,      // Parent class
            update_parameters, // Name of function in C++ (must match Python name)
            // Argument(s)
        );
    }
};

UnitQuaternion unit_quaternion_from_euler_angles(EulerAngles euler) { return UnitQuaternion(euler); }
UnitQuaternion unit_quaternion_from_rotmat(Matrix3d R) { return UnitQuaternion(R); }
EulerAngles euler_angles_from_unit_quaternion(UnitQuaternion q) { return EulerAngles(q); }
EulerAngles euler_angles_from_rotmat(Matrix3d R) { return EulerAngles(R); }
SimState_t simstate_from_array(const VectorXd &v) { return SimState_t(v); }
SimState_t simstate_from_simstate(const SimState_t &s) { return SimState_t(s); }
Inertial inertial_simple(const double mass, const double j_xx, const double j_yy, const double j_zz)
{
    std::vector<double> v = {j_xx, j_yy, j_zz};
    return Inertial(mass, v);
}
Input input_from_array(const std::vector<double> v) { return Input(v.at(0), v.at(1), v.at(2), std::vector<double>(v.begin() + 3, v.end())); }

PYBIND11_MODULE(cpp_last_letter_lib, m)
{
    m.doc() = "The last_letter_lib python bindings.";

    auto m_math_utils = m.def_submodule("cpp_math_utils", "last_letter_lib math_utils submodule");
    py::class_<Vector3>(m_math_utils, "Vector3")
        .def(py::init<double, double, double>(), py::arg("x") = 0, py::arg("y") = 0, py::arg("z") = 0)
        .def(py::self + py::self)
        .def(py::self += py::self)
        .def(py::self - py::self)
        .def(py::self -= py::self)
        .def(py::self * double())
        .def(double() * py::self)
        .def(py::self *= double())
        .def(-py::self)
        .def(py::self == py::self)
        .def_property("x", &Vector3::get_x, &Vector3::set_x)
        .def_property("y", &Vector3::get_y, &Vector3::set_y)
        .def_property("z", &Vector3::get_z, &Vector3::set_z)
        .def_property_readonly("norm", &Vector3::norm)
        .def("to_array", &Vector3::to_array)
        .def(
            "__getitem__",
            [](const Vector3 &v, const size_t idx)
            { return v[idx]; })
        .def("__str__", &Vector3::to_str)
        .def("__repr__", &Vector3::repr)
        .def(py::pickle(
            [](const Vector3 &v)
            { return v.to_array(); },
            [](Vector3d v)
            {
                return Vector3(v.x(), v.y(), v.z());
            }));
    py::class_<UnitQuaternion>(m_math_utils, "UnitQuaternion")
        .def(py::init<double, double, double, double>(), py::arg("w") = 1, py::arg("x") = 0, py::arg("y") = 0, py::arg("z") = 0)
        .def_property("w", &UnitQuaternion::get_w, &UnitQuaternion::set_w)
        .def_property("x", &UnitQuaternion::get_x, &UnitQuaternion::set_x)
        .def_property("y", &UnitQuaternion::get_y, &UnitQuaternion::set_y)
        .def_property("z", &UnitQuaternion::get_z, &UnitQuaternion::set_z)
        .def_property_readonly("real", &UnitQuaternion::real)
        .def_property_readonly("imag", &UnitQuaternion::imag)
        .def("normalize", &UnitQuaternion::normalize)
        .def("conjugate", &UnitQuaternion::conjugate)
        .def("inverse", &UnitQuaternion::inverse)
        .def("flipped", &UnitQuaternion::flipped)
        .def("to_array", &UnitQuaternion::get_coeffs)
        .def("to_prodmat", &UnitQuaternion::to_prodmat)
        .def("to_euler", &UnitQuaternion::to_euler)
        .def("R_ib", &UnitQuaternion::R_ib)
        .def("R_bi", &UnitQuaternion::R_bi)
        .def("q_dot", &UnitQuaternion::q_dot)
        .def("__repr__", &UnitQuaternion::to_str)
        .def(
            "__eq__", [](const UnitQuaternion &a, UnitQuaternion &b)
            { return a.isApprox(b); },
            py::is_operator())
        .def(py::self * py::self)
        .def(py::self *= py::self)
        .def(
            "__mul__", [](const UnitQuaternion q, const Vector3d v)
            { return q * v; },
            py::is_operator())
        .def(
            "__mul__", [](const UnitQuaternion q, const Vector3 v)
            { return q * v; },
            py::is_operator())
        .def_static("from_euler", unit_quaternion_from_euler_angles)
        .def_static("from_rotmat", unit_quaternion_from_rotmat)
        // .def_static("from_two_vectors", &UnitQuaternion::UnitQuaternion<Vector3d, Vector3d, double>);
        .def("is_unit", &UnitQuaternion::is_unit)
        .def(py::pickle(
            [](const UnitQuaternion &q)
            { return q.get_coeffs(); },
            [](Vector4d v)
            {
                return UnitQuaternion(v(0), v(1), v(2), v(3));
            }));
    py::class_<EulerAngles>(m_math_utils, "EulerAngles")
        .def(py::init<double, double, double, bool>(), py::arg("roll") = 0, py::arg("pitch") = 0, py::arg("yaw") = 0, py::arg("in_degrees") = false)
        .def_readwrite("roll", &EulerAngles::roll)
        .def_readwrite("pitch", &EulerAngles::pitch)
        .def_readwrite("yaw", &EulerAngles::yaw)
        .def(py::self == py::self)
        .def("__repr__", &EulerAngles::to_str)
        .def_static("from_quaternion", euler_angles_from_unit_quaternion)
        .def_static("from_rotmat", euler_angles_from_rotmat)
        .def("to_quaternion", &EulerAngles::to_quaternion)
        .def("to_array", [](EulerAngles &self)
             { return Vector3d(self.roll, self.pitch, self.yaw); })
        .def("R_roll", &EulerAngles::R_roll)
        .def("R_pitch", &EulerAngles::R_pitch)
        .def("R_yaw", &EulerAngles::R_yaw)
        .def("R_bi", &EulerAngles::R_bi)
        .def("R_ib", &EulerAngles::R_ib)
        .def("T_eb", &EulerAngles::T_eb)
        .def("T_be", &EulerAngles::T_be);
    py::class_<Inertial>(m_math_utils, "Inertial")
        .def(py::init<double>(), py::arg("mass") = 0)
        .def(py::init<double, std::vector<double>>(), py::arg("mass"), py::arg("tensor"))
        .def_readwrite("mass", &Inertial::mass)
        .def_readwrite("tensor", &Inertial::tensor)
        .def_static("simple", inertial_simple);

    auto m_uav_utils = m.def_submodule("cpp_uav_utils", "last_letter_lib uav_utils submodule");
    py::class_<Pose>(m_uav_utils, "Pose")
        .def(py::init())
        .def("__matmul__", &Pose::operator*)
        // .def_property("position", &Pose::get_position_as_vector3, &Pose::set_position_from_vector3)
        .def_readwrite("position", &Pose::position)
        .def_readwrite("orientation", &Pose::orientation)
        .def_property_readonly("T", &Pose::T);
    py::class_<Wrench_t>(m_uav_utils, "Wrench")
        .def(py::init<Vector3d, Vector3d>(), py::arg("force"), py::arg("torque"))
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def_readwrite("force", &Wrench_t::force)
        .def_readwrite("torque", &Wrench_t::torque)
        .def("to_array", &Wrench_t::to_array);
    py::class_<Airdata>(m_uav_utils, "Airdata")
        .def(py::init<double, double, double>(), py::arg("airspeed") = 0, py::arg("alpha") = 0, py::arg("beta") = 0)
        .def("init_from_velocity", &Airdata::init_from_velocity, py::arg("vel_body"), py::arg("vel_wind") = Vector3d())
        .def("init_from_state_wind", &Airdata::init_from_state_wind, py::arg("state"), py::arg("wind"))
        .def_property_readonly("S_bw", &Airdata::S_bw)
        .def_property_readonly("S_wb", &Airdata::S_wb)
        .def("to_u", &Airdata::to_velocity)
        .def_readwrite("airspeed", &Airdata::airspeed)
        .def_readwrite("alpha", &Airdata::alpha)
        .def_readwrite("beta", &Airdata::beta);
    py::class_<SimState_t>(m_uav_utils, "UavState")
        .def(py::init())
        .def(py::init<Vector3d, UnitQuaternion, Vector3d, Vector3d, std::vector<double>>(),
             py::arg("position") = Vector3d(),
             py::arg("orientation") = UnitQuaternion(),
             py::arg("velocity_linear") = Vector3d(),
             py::arg("velocity_angular") = Vector3d(),
             py::arg("thrusters_velocity") = std::vector<double>(0))
        .def(py::init<Vector3, UnitQuaternion, Vector3, Vector3, std::vector<double>>(),
             py::arg("position") = Vector3(),
             py::arg("orientation") = UnitQuaternion(),
             py::arg("velocity_linear") = Vector3(),
             py::arg("velocity_angular") = Vector3(),
             py::arg("thrusters_velocity") = std::vector<double>(0))
        .def_static("from_array", simstate_from_array)
        .def_static("from_uavstate", simstate_from_simstate)
        .def_property("position", &SimState_t::get_position, &SimState_t::set_position)
        // Fix discrepancy between "attitude" property and "orientation" initialization argument
        .def_property("attitude", &SimState_t::get_orientation, &SimState_t::set_orientation)
        .def_property("velocity_linear", &SimState_t::get_velocity_linear, &SimState_t::set_velocity_linear)
        .def_property("velocity_angular", &SimState_t::get_velocity_angular, &SimState_t::set_velocity_angular)
        .def_readwrite("thrusters_velocity", &SimState_t::rotorspeed)
        .def("get_euler", [](SimState_t &s)
             { return s.pose.orientation.to_euler(); })
        .def("to_array", &SimState_t::to_array)
        .def("strip_thrusters", &SimState_t::strip_thrusters)
        .def("__deepcopy__", [](const SimState_t &self, py::dict)
             { return SimState_t(self); });
    py::class_<Input>(m_uav_utils, "Inputs")
        .def(py::init())
        .def(py::init<double, double, double, std::vector<double>>(), py::arg("delta_a"), py::arg("delta_e"), py::arg("delta_r"), py::arg("delta_t"))
        .def_property("delta_a", &Input::get_da, &Input::set_da)
        .def_property("delta_e", &Input::get_de, &Input::set_de)
        .def_property("delta_r", &Input::get_dr, &Input::set_dr)
        .def_property("delta_t", &Input::get_dt, &Input::set_dt)
        .def_property_readonly("num_thrusters", [](Input &i)
                               { return i.num_thrusters; })
        .def("to_array", [](Input &t)
             { return t.to_python_array(); })
        .def_static("from_array", input_from_array);

    auto m_programming_utils = m.def_submodule("cpp_programming_utils", "last_letter_lib programming_utils submodule");
    py::class_<Parametrized, PyParametrized>(m_programming_utils, "Parametrized")
        .def(py::init<string>(), py::arg("name"))
        .def("initialize", static_cast<void (Parametrized::*)(const std::string)>(&Parametrized::initialize), py::arg("parameters"))
        .def("get_param_double", &Parametrized::get_param<double>, py::arg("param_name"))
        .def("get_param_string", &Parametrized::get_param<std::string>, py::arg("param_name"))
        .def("set_param", &Parametrized::set_param<double>, py::arg("param_name"), py::arg("param_value"), py::arg("safe") = true)
        .def("set_param", &Parametrized::set_param<std::string>, py::arg("param_name"), py::arg("param_value"), py::arg("safe") = true);

    auto m_systems = m.def_submodule("cpp_systems", "last_letter_lib systems submodule");
    py::class_<Component, Parametrized>(m_systems, "Component", py::multiple_inheritance()) // Declaring as multiple_inheritance, because Parametrized is a virtual base of Component.
        .def(py::init<string>(), py::arg("name"))
        .def("update_parameters", &Component::update_parameters)
        .def_readwrite("pose", &Component::pose)
        .def_readwrite("inertial", &Component::inertial);

    auto m_environment = m.def_submodule("cpp_environment", "last_letter_lib environment submodule");
    py::class_<Environment_t>(m_environment, "EnvironmentData")
        .def(py::init())
        .def_readwrite("wind", &Environment_t::wind)
        .def_readwrite("rho", &Environment_t::density)
        .def_readwrite("pressure", &Environment_t::pressure)
        .def_readwrite("temperature", &Environment_t::temperature)
        .def_readwrite("gravity", &Environment_t::gravity);
}
