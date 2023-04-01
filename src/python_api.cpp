#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <iostream>

#include "last_letter_lib/math_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"

namespace py = pybind11;

using Eigen::Quaterniond;
using Eigen::Vector3d;

using last_letter_lib::math_utils::Vector3;
using last_letter_lib::uav_utils::Pose;

///////////////////////////////////////////////////////////////////////////////
// math_utils

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
        .def_property("position", &Pose::get_position_as_vector3, &Pose::set_position_from_vector3)
        .def_property("orientation", &Pose::get_orientation_as_vector, &Pose::set_orientation_from_vector)
        .def_property_readonly("T", &Pose::T);

    // auto m_systems = m.def_submodule("cpp_systems", "last_letter_lib systems submodule");
    // py::class_<Component>(m_uav_utils, "Pose")
    //     .def(py::init())
    //     .def_property("position", &Pose::get_position_as_vector3, &Pose::set_position_from_vector3)
    //     .def_property("orientation", &Pose::get_orientation_as_vector, &Pose::set_orientation_from_vector)
    //     .def_property_readonly("T", &Pose::T);
}
