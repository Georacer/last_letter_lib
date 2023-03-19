#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>

#include "last_letter_lib/math_utils.hpp"

namespace lll_math = last_letter_lib::math_utils;
namespace py = pybind11;

using Eigen::Quaterniond;
using Eigen::Vector3d;

///////////////////////////////////////////////////////////////////////////////
// math_utils

class Vector3
{
public:
    Vector3(double x, double y, double z)
    {
        vector_.x() = x;
        vector_.y() = y;
        vector_.z() = z;
    }
    Vector3d to_array() const
    {
        return vector_;
    }
    std::vector<double> to_vector() const
    {
        return std::vector<double>{vector_.x(), vector_.y(), vector_.z()};
    }
    double norm() const
    {
        return vector_.norm();
    }
    Vector3 operator+(const Vector3 &v) const
    {
        Vector3d other_vector = v.to_array();
        return Vector3(
            vector_.x() + other_vector.x(),
            vector_.y() + other_vector.y(),
            vector_.z() + other_vector.z());
    }
    Vector3 &operator+=(const Vector3 &v)
    {
        Vector3d other_vector = v.to_array();
        vector_ += other_vector;
        return *this;
    }
    Vector3 operator-() const
    {
        return Vector3(-vector_.x(), -vector_.y(), -vector_.z());
    }
    Vector3 operator-(const Vector3 &v) const
    {
        Vector3d other_vector = v.to_array();
        return Vector3(
            vector_.x() - other_vector.x(),
            vector_.y() - other_vector.y(),
            vector_.z() - other_vector.z());
    }
    Vector3 &operator-=(const Vector3 &v)
    {
        Vector3d other_vector = v.to_array();
        vector_ -= other_vector;
        return *this;
    }
    Vector3 operator*(double c) const
    {
        return Vector3(c * vector_.x(), c * vector_.y(), c * vector_.z());
    }
    Vector3 &operator*=(double c)
    {
        vector_ *= c;
        return *this;
    }
    friend Vector3 operator*(double c, const Vector3 &v)
    {
        Vector3d res = c * v.vector_;
        return Vector3(res.x(), res.y(), res.z());
    }
    bool operator==(const Vector3 &v) const
    {
        Vector3d other_vec = v.to_array();
        return (vector_.x() == other_vec.x()) && (vector_.y() == other_vec.y()) && (vector_.z() == other_vec.z());
    }
    std::string to_str() const
    {
        std::stringstream ss;
        ss << vector_;
        return ss.str();
    }
    std::string repr() const
    {
        std::stringstream ss;
        ss << "Vector3(" << vector_.x() << ", " << vector_.y() << ", " << vector_.z() << ")";
        return ss.str();
    }

private:
    Vector3d vector_{0, 0, 0};
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
        .def("norm", &Vector3::norm)
        .def("to_array", &Vector3::to_array)
        .def("__iter__", [](const Vector3 &v)
             { std::vector<double> vec{v.to_vector()}; return py::make_iterator(vec.begin(), vec.end()); })
        .def("__str__", &Vector3::to_str)
        .def("__repr__", &Vector3::repr);
    // m_math_utils.def("sub", &sub, "A function that subs two numbers.", py::arg("i") = 2, py::arg("j") = 1);
}
