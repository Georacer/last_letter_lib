#pragma once

// DataTamer TypeDefinition specializations for the last_letter_lib data types
// we log. Each must live in the SAME namespace as the type it describes, so
// DataTamer finds it via ADL, and must return a std::string_view type name and
// enumerate the type's fields via add("name", &member). Everything must bottom
// out in numeric scalars (double here).
//
// This header pulls in DataTamer's serialization contract only transitively
// through the add() callback shapes; it does NOT include DataTamer. Include it
// in the .cpp that calls registerValue(), alongside the DataTamer headers.

#include <string_view>

#include <Eigen/Eigen>

#include <last_letter_lib/math_utils.hpp>
#include <last_letter_lib/uav_utils.hpp>

// ---- Eigen ------------------------------------------------------------------
// Defined in namespace Eigen so ADL finds it for Eigen::Vector3d. Uses the
// accessor form (&v.x()), mirroring DataTamer's own PseudoEigen example.
namespace Eigen
{
template <typename AddField>
inline std::string_view TypeDefinition(Vector3d &v, AddField &add)
{
    add("x", &v.x());
    add("y", &v.y());
    add("z", &v.z());
    return "Vector3d";
}
} // namespace Eigen

// ---- last_letter_lib::math_utils -------------------------------------------
namespace last_letter_lib
{
namespace math_utils
{
template <typename AddField>
inline std::string_view TypeDefinition(UnitQuaternion &q, AddField &add)
{
    add("w", &q.w());
    add("x", &q.x());
    add("y", &q.y());
    add("z", &q.z());
    return "UnitQuaternion";
}
} // namespace math_utils
} // namespace last_letter_lib

// ---- last_letter_lib::uav_utils --------------------------------------------
namespace last_letter_lib
{
namespace uav_utils
{

template <typename AddField>
inline std::string_view TypeDefinition(Wrench_t &w, AddField &add)
{
    add("force", &w.force);
    add("torque", &w.torque);
    return "Wrench_t";
}

template <typename AddField>
inline std::string_view TypeDefinition(WrenchSum_t &w, AddField &add)
{
    add("gravity", &w.wrenchGrav);
    add("aero", &w.wrenchAero);
    add("prop", &w.wrenchProp);
    add("ground", &w.wrenchGround);
    add("external", &w.wrenchExternal);
    return "WrenchSum_t";
}

template <typename AddField>
inline std::string_view TypeDefinition(Airdata &a, AddField &add)
{
    add("airspeed", &a.airspeed);
    add("alpha", &a.alpha);
    add("beta", &a.beta);
    return "Airdata";
}

template <typename AddField>
inline std::string_view TypeDefinition(Pose &p, AddField &add)
{
    add("position", &p.position);
    add("orientation", &p.orientation);
    return "Pose";
}

template <typename AddField>
inline std::string_view TypeDefinition(Twist &t, AddField &add)
{
    add("linear", &t.linear);
    add("angular", &t.angular);
    return "Twist";
}

} // namespace uav_utils
} // namespace last_letter_lib
