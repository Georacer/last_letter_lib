#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <iostream>

#include "last_letter_lib/math_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/systems.hpp"

namespace py = pybind11;

using Eigen::Quaterniond;
using Eigen::Vector3d;

using last_letter_lib::math_utils::Vector3;
using last_letter_lib::programming_utils::Parametrized;
using last_letter_lib::systems::Component;
using last_letter_lib::uav_utils::Pose;
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

PYBIND11_MODULE(cpp_last_letter_lib, m)
{
    m.doc() = "The last_letter_lib python bindings.";
    // m.def("add", &add, "A function that adds two numbers.", py::arg("i") = 1, py::arg("j") = 2);

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
    // m_math_utils.def("sub", &sub, "A function that subs two numbers.", py::arg("i") = 2, py::arg("j") = 1);

    auto m_uav_utils = m.def_submodule("cpp_uav_utils", "last_letter_lib uav_utils submodule");
    py::class_<Pose>(m_uav_utils, "Pose")
        .def(py::init())
        // .def("__matmul__", py::self * Wrench_t())
        .def("__matmul__", &Pose::operator*)
        .def_property("position", &Pose::get_position_as_vector3, &Pose::set_position_from_vector3)
        .def_property("orientation", &Pose::get_orientation_as_vector, &Pose::set_orientation_from_vector)
        .def_property_readonly("T", &Pose::T);
    py::class_<Wrench_t>(m_uav_utils, "Wrench")
        .def(py::init<Vector3d, Vector3d>(), py::arg("force"), py::arg("torque"))
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def_readwrite("force", &Wrench_t::force)
        .def_readwrite("torque", &Wrench_t::torque)
        .def("to_array", &Wrench_t::to_array);
    py::class_<Inertial>(m_uav_utils, "Inertial")
        .def(py::init())
        .def_readwrite("mass", &Inertial::mass)
        .def_readwrite("tensor", &Inertial::tensor);

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
}
