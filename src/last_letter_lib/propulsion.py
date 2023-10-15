#!/usr/bin/env python3
"""Propulsion-related libraries

Contains models and functions relevant to propulsion and thrusters.

usage:
    Import the module for use in other scripts.
"""

__authors__ = ["George Zogopoulos", "Johanna Windh"]
__credits__ = []
__date__ = "Fri 10 Sep 2021"
__copyright__ = "Copyright 2021, George Zogopoulos"

from abc import ABC
from abc import ABCMeta
from abc import abstractmethod
from dataclasses import dataclass
from dataclasses import field
from enum import Enum
from typing import List
from typing import Optional
from typing import Union

import numpy as np
import yaml
from pydantic import BaseModel
from pydantic import Field
from scipy.optimize import minimize_scalar

from last_letter_lib.systems import Component
from last_letter_lib.systems import ComponentParameters
from last_letter_lib.systems import DynamicSystem
from last_letter_lib.utils.math import Vector3
from last_letter_lib.utils.math import Wrench
from last_letter_lib.utils.math import radps2rps


##############################################################################
# Propeller related functions


# @dataclass
# class PropellerDescription:
#     name: str = None
#     index: int = None
#     diameter_nominal: float = None
#     diameter_actual: float = None
#     pitch_nominal: float = None
#     pitch_actual: float = None
#     hub_length: float = None


# def get_prop_idx(prop_list_df, prop_description):
#     """
#     Return the propeller index in the propeller list given a propeller description

#     INPUT:
#         prop_list_df        a list of propellers as returned from build_prop_list_*
#         prop_description    a PropellerDescription object
#     OUTPUT:
#         An iterator with propeller indices that fit the description

#     This is useful for querying the propeller index from its database by passing
#     the propeller description. The propeller index is not fixed, as it depends on
#     the order of the sample data appearance, when parsing the test database. As
#     new propellers get added, the propeller indices will shift.

#     The PropellerDescription object will filter the database by the non-None
#     fields and return an iterator with the propellers that match the description.
#     """
#     description_dict = dataclasses.asdict(prop_description)

#     # Get all the float fields of the description. We want to add a margin to their comparison.
#     field_types = {
#         field.name: field.type for field in dataclasses.fields(prop_description)
#     }

#     # Get all the matching propellers
#     mask_iter = []
#     for key, value in description_dict.items():
#         if value is not None:  # Filter only non-None filters
#             # Compare for equality in non-float fields
#             if field_types[key] is not float:
#                 mask_iter.append(prop_list_df[key] == value)
#             else:
#                 margin = 0.03  # Relative margin accepted
#                 mask_iter.append(
#                     (prop_list_df[key] > value * (1 - margin))
#                     & (prop_list_df[key] < value * (1 + margin))
#                 )

#     # Apply the mask
#     filtered_df = pd.DataFrame(prop_list_df)
#     for mask in mask_iter:
#         filtered_df = filtered_df[mask]

#     # Get the indices
#     return list(filtered_df["index"])


# def get_prop_idx_by_name(propeller_list, prop_names):
#     """
#     Return the propeller indices in the propeller list given a list of propeller names

#     INPUT:
#         propeller_list      A list of propellers as returned from build_prop_list_*
#         prop_names          A list of propeller names to search for. If None,
#                             all the propeller indices will be returned.
#     OUTPUT:
#         An iterator with propeller indices that fit the description

#     This is useful for querying multiple propeller indices from their database by passing
#     a list of propeller names.
#     """

#     if prop_names is None:
#         prop_idx_iter = list(propeller_list["index"])
#     else:
#         prop_idx_iter = []
#         for name in prop_names:
#             new_desc = PropellerDescription(name=name)
#             prop_idx = get_prop_idx(propeller_list, new_desc)
#             prop_idx_iter.append(prop_idx[0])

#     return prop_idx_iter


def pitch_from_pitch_angle(diameter, beta):
    """
    Calculate nominal propeller pitch given its blade pitch angle

    Blade angle is given at 75% blade station.

    INPUTS:
        diameter    Propeller diameter, m
        beta        Blade pitch angle, rad
    """
    r_nom = 0.75 * diameter / 2  # Pitch angle is given at 75% of nominal radius
    return 2 * np.pi * r_nom * np.tan(beta)


def pitch_angle_from_pitch(diameter, pitch):
    """
    Calculate nominal propeller pitch angel given its pitch.

    Blade angle is given at 75% blade station.

    INPUTS:
        diameter    Propeller diameter, m
        pitch       Propeller pitch, m
    """
    r_nom = 0.75 * diameter / 2  # Pitch angle is given at 75% of nominal radius
    return np.arctan(pitch / (2 * np.pi * r_nom))


def correct_pitch_for_hub(pitch, diam_nom, diam_actual):
    """
    Recalculates effective pitch when using non-standard hub_length.

    Applicable to propellers with foldable or removable blades.

    INPUTS:
        pitch       The nominal propeller pitch, m
        diam_nom    The nominal propeller diameter, m
        diam_actual The actual propeller diameter, m
    """

    angle = pitch_angle_from_pitch(diam_nom, pitch)
    pitch_actual = pitch_from_pitch_angle(diam_actual, angle)
    return pitch_actual


class PropellerParameters(BaseModel):
    """
    A propeller model.

    Its parameters are according to these equations:

    .. math::

        J = V_a / (n D)

        T = \\rho n^2 D^4 C_{thrust}(J)

        P = \\rho n^3 D^5 C_{power}(J)

    """

    diameter: float = 0.3048
    """
    The nominal propeller diameter. It is assumed that its coefficients were
    normalized by this diameter.
    """


class Propeller(ABC):
    """
    A propeller model.

    Its parameters are according to these equations:

    .. math::

        J = V_a / (n D)

        T = \\rho n^2 D^4 C_{thrust}(J)

        P = \\rho n^3 D^5 C_{power}(J)

    """

    params: PropellerParameters
    """
    The nominal propeller diameter. It is assumed that its coefficients were
    normalized by this diameter.
    """

    def __init__(self, desc: PropellerParameters):
        """
        Initialize a propeller with an equivalent description.
        """
        self.params = desc

    def max_drag(self, V):
        """
        The maximum drag the propeller produces when freewheeling.

        Used to cap the negative force produced, since the propeller will stall.
        A flat plate model is used.
        """
        c_d = 1.17  # Flat plate drag coefficient from https://en.wikipedia.org/wiki/Drag_coefficient
        rho = 1.225
        chord = self.params.diameter / 10  # rough number for chord
        S = self.params.diameter * chord
        return 0.5 * rho * V**2 * S * c_d

    def calc_advance_ratio(self, V, n):
        """
        Calculate the advance ratio of a propeller

        INPUTS:
            V   Freestream airspeed, m/s
            n   Propeller speed, revolutions per second
        """
        if np.abs(n) != 0:
            return V / (n * self.params.diameter)
        elif V == 0:
            return 0
        else:
            return np.sign(V) * np.infty

    @abstractmethod
    def calc_coeff_thrust(self, ar):
        raise NotImplementedError

    @abstractmethod
    def calc_coeff_power(self, ar):
        raise NotImplementedError

    def calc_thrust(self, V, n, rho):
        """
        Calculate thrust of propeller.
        """
        ar = self.calc_advance_ratio(V, n)
        Ct = self.calc_coeff_thrust(ar)

        thrust = Ct * rho * n**2 * self.params.diameter**4
        # Cap the thrust of the propeller if negative
        if thrust < 0:
            thrust = np.max([thrust, -self.max_drag(V)])
        return thrust

    def calc_power(self, V, n, rho):
        """
        Calculate power of propeller.
        """
        ar = self.calc_advance_ratio(V, n)
        Cp = self.calc_coeff_power(ar)

        power = Cp * rho * n**3 * self.params.diameter**5
        thrust = self.calc_thrust(V, n, rho)
        # Don't perform power calculations if thrust is negative.
        # Because the propeller is outside its operational envelope.
        if thrust < 0:
            power = 0
        return power

    def calc_torque(self, V, n, rho):
        """
        Calculate torque of propeller.
        """
        ar = self.calc_advance_ratio(V, n)
        c_p = self.calc_coeff_power(ar)
        thrust = self.calc_thrust(V, n, rho)
        # Don't perform power calculations if thrust is negative.
        # Because the propeller is outside its operational envelope.
        if thrust > 0:
            c_q = c_p / (2 * np.pi)
        else:
            c_q = 0
        return c_q * rho * n**2 * self.params.diameter**5

    def calc_efficiency(self, ar):
        """
        Calculate efficiency of propeller from advance ratio.
        """
        Ct = self.calc_coeff_thrust(ar)
        Cp = self.calc_coeff_power(ar)
        return Ct * ar / Cp

    def calc_wrench(self, V, n, rho):
        f_x = self.calc_thrust(V, n, rho)
        t_x = self.calc_torque(V, n, rho)
        force = Vector3(f_x, 0, 0)
        torque = Vector3(t_x, 0, 0)
        return Wrench(force.to_array(), torque.to_array())


class PropellerStandardParameters(PropellerParameters):
    pitch: float = 0
    """The nominal propeller pitch."""
    c_thrust: List[float] = [0]
    """Coefficients of the thrust polynomial, higher order degree first."""
    c_power: List[float] = [0]
    """Coefficients of the power polynomial, higher order degree first."""


class PropellerStandard(Propeller):
    def __init__(self, desc):
        super().__init__(desc)

    def calc_coeff_thrust(self, ar):
        if np.isfinite(ar):
            c_t = np.polyval(self.params.c_thrust, ar)
        else:  # This applies in omega=0 situations and is a numerical hack
            c_t = self.c_thrust[-1]
        return c_t

    def calc_coeff_power(self, ar):
        if np.isfinite(ar):
            c_p = np.polyval(self.params.c_power, ar)
        else:  # This applies in omega=0 situations and is a numerical hack
            c_p = self.c_power[-1]
        return c_p


class PropellerBladeElementParameters(PropellerParameters):
    pitch_nominal: Optional[float] = None
    """The nominal propeller pitch. Either this must be specified or pitch_angle."""
    num_blades: int = 2
    """The number of the propeller blades."""
    chord_mean: float = 0.03
    """The mean chord of each blade airfoil."""
    pitch_angle: Optional[float] = None
    """The nominal blade angle. Either this must be specified or pitch."""


class PropellerBladeElement(Propeller):
    """
    Theoretical propeller model based on blade element theory.

    Based on McCormick, B. W. (1979). Aerodynamics Aeronautics and Flight Mechanics, p.348.
    It contains mixed blade element and momentum disk theories.
    """

    # Actual pitch is defined by a getter method, applying the compensation coefficient
    k_pitch = 1.0  # Coefficient compensating for wrongly defined pitch.
    c_t_0 = None  # Coefficient of thrust on standstill (J=0)
    lift_coeff = None  # Blade lift coefficient
    parasitic_drag = 0.01  # Blade parasitic drag coefficient

    def __init__(self, desc):
        super().__init__(desc)

        if desc.pitch_nominal is not None:
            self.params.pitch_nominal = desc.pitch_nominal
            self.params.pitch_angle = self.beta(
                desc.pitch, self.params.diameter / 2 * 0.75
            )
        elif desc.pitch_angle is not None:
            self.params.pitch_nominal = self.pitch_from_pitch_angle(desc.pitch_angle)
            self.params.pitch_angle = desc.pitch_angle
        else:
            raise ValueError(
                "One of pitch or pitch angle must be passed to constructor."
            )

    @property
    def pitch(self):
        return self.k_pitch * self.params.pitch_nominal

    def calc_lift_coeff(self, c_t_ref, v_ref, omega_ref):
        """
        Calculate the lift coefficient of the blades given a sample point.

        Also internally stores the lift coefficient.

        INPUT:
            c_t_ref     Coefficient of thrust at sample point
            v_ref       Airspeed at sample point, m/s
            omega_ref   Propeller speed at sample point, rad/s
        OUTPUT:
            lift_coeff  Lift coefficient of propeller blades
        """

        radius = self.params.diameter / 2
        sigma = self.sigma(self.params.num_blades, self.params.chord_mean, radius)
        lam_val_0 = self.lamda(v_ref, omega_ref, radius)
        speed_rps = omega_ref / (2 * np.pi)
        j_ref = self.calc_advance_ratio(v_ref, speed_rps)

        def fun_1(a):
            return np.abs(
                self._C_T(
                    j_ref,
                    sigma,
                    lam_val_0,
                    radius,
                    self.params.pitch,
                    self.parasitic_drag,
                    self.params.num_blades,
                    a,
                    omega_ref,
                )
                - c_t_ref
            )

        # Solve the numerical system to calculate Cl
        optim_res = minimize_scalar(fun_1, tol=0.01)
        self.lift_coeff = optim_res.x

        return self.lift_coeff

    @staticmethod
    def V_T(omega, R):
        """
        Propeller tip speed

        INPUT:
            omega   Propeller speed, rad/s
            R       Propeller radius, m
        OUTPUT:
            tip speed, m/s
        """
        return omega * R

    @staticmethod
    def V_r(V_T, x, lam):
        """
        Airspeed relative to propeller blade rotation
        """
        return V_T * (np.sqrt(x**2 + lam**2))

    @staticmethod
    def lamda(V, omega, R):
        """
        Propeller advance-like ratio
        """
        return V / (omega * R)

    @staticmethod
    def beta(p, r):
        """
        Propeller blade station pitch angle
        """
        return np.arctan(p / (2 * np.pi * r))

    def pitch_from_pitch_angle(self, beta):
        """
        Calculate nominal propeller pitch given its blade pitch angle
        """
        return pitch_from_pitch_angle(self.params.diameter, beta)

    @staticmethod
    def x(r, R):
        """
        Propeller relative station
        """
        return r / R

    @staticmethod
    def phi(lam, x):
        """
        Propeller advance angle
        """
        return np.arctan(lam / x)

    @staticmethod
    def sigma(B, c, R):
        """
        Propeller solidity ratio
        """
        return B * c / (np.pi * R)

    @staticmethod
    def _a_i(sigma, x, a, V_r, V_T, beta, lam):
        """
        Propeller induced angle of attack at reference station at reference station
        """
        phi_val = PropellerBladeElement.phi(lam, x)
        term_1 = lam / x + (sigma * a * V_r) / (8 * x**2 * V_T)
        term_2 = sigma * a * V_r / (2 * x**2 * V_T) * (beta - phi_val)
        a_i = 1 / 2 * (-term_1 + np.sqrt(term_1**2 + term_2))
        return a_i

    def calc_a_i(self, v_ref, omega_ref):
        """
        Calculate the induced angle of attack at an operational point.

        The induced angle is given at the blade reference station at 75% radius.

        INPUTS:
            v_ref       Freestream airspeed at operational point, m/s
            omega_ref   Propeller speed at operational point, rad/s
        OUTPUTS:
            induced angle of attack, radians
        """

        x = 0.75
        radius = self.params.diameter / 2
        lamda = self.lamda(v_ref, omega_ref, radius)
        V_T = self.V_T(omega_ref, radius)
        V_r = self.V_r(V_T, x, lamda)
        sigma = self.sigma(self.params.num_blades, self.params.chord_mean, radius)

        return self._a_i(
            sigma, x, self.lift_coeff, V_r, V_T, self.params.pitch_angle, lamda
        )

    def calc_aoa(self, v_ref, omega_ref):
        """
        Calculate blade angle of attack at reference station.

        INPUTS:
            v_ref       Freestream airspeed at operational point, m/s
            omega_ref   Propeller speed at operational point, rad/s
        OUTPUTS:
            angle of attack, radians
        """
        x = 0.75
        radius = self.params.diameter / 2
        lamda = self.lamda(v_ref, omega_ref, radius)
        phi = self.phi(lamda, x)
        a_i = self.calc_a_i(v_ref, omega_ref)

        return self.pitch_angle - phi - a_i

    def calc_phi(self, v_ref, omega_ref):
        """
        Calculate propeller helix angle.

        This is the angle corresponding to the advance ratio.

        INPUTS:
            v_ref       Freestream airspeed at operational point, m/s
            omega_ref   Propeller speed at operational point, rad/s
        OUTPUTS:
            Helix angle, radians
        """

        x = 0.75
        radius = self.params.diameter / 2
        lamda = self.lamda(v_ref, omega_ref, radius)
        return self.phi(lamda, x)

    def calc_coeff_thrust(self, v_ref, omega_ref):
        """
        Calculate the propeller coefficient of thrust.

        INPUTS:
            v_ref       Freestream airspeed at point of operation, m/s
            omega_ref   Propeller speed at point of operations, m/s
        OUTPUTS:
            c_t         Coefficient of thrust
        """
        radius = self.params.diameter / 2
        sigma = self.sigma(self.params.num_blades, self.params.chord_mean, radius)
        lamda = self.lamda(v_ref, omega_ref, radius)
        speed_rps = omega_ref / (2 * np.pi)
        j = self.calc_advance_ratio(v_ref, speed_rps)

        c_t = self._C_T(
            j,
            sigma,
            lamda,
            radius,
            self.params.pitch_nominal,
            self.parasitic_drag,
            self.params.num_blades,
            self.lift_coeff,
            omega_ref,
        )
        return c_t

    @staticmethod
    def _C_T(J, sigma, lam, R, pitch, c_d_p, B, a, omega):
        """
        Propeller coefficient of thrust
        """
        x_h = 0.15  # Blade ratio below which no lift is generated
        stations = np.linspace(x_h, 1, 100)

        def int_argument(x):
            phi_val = PropellerBladeElement.phi(lam, x)
            r_val = R * x
            beta_val = PropellerBladeElement.beta(pitch, r_val)
            phi_val = PropellerBladeElement.phi(lam, x)
            V_T_val = PropellerBladeElement.V_T(omega, R)
            V_r_val = PropellerBladeElement.V_r(V_T_val, x, lam)
            a_i_val = PropellerBladeElement._a_i(
                sigma, x, a, V_r_val, V_T_val, beta_val, lam
            )
            C_l_val = PropellerBladeElement.C_l(a, beta_val, phi_val, a_i_val)
            C_d_val = PropellerBladeElement.C_d(C_l_val, c_d_p, B, sigma)
            return (
                (J**2 + np.pi**2 * x**2)
                * sigma
                * (
                    C_l_val * np.cos(phi_val + a_i_val)
                    - C_d_val * np.sin(phi_val + a_i_val)
                )
            )

        y = [int_argument(x) for x in stations]

        return np.pi / 8 * np.trapz(y, stations)

    def calc_coeff_power(self, v_ref, omega_ref):
        """
        Calculate the propeller coefficient of power.

        INPUTS:
            v_ref       Freestream airspeed at point of operation, m/s
            omega_ref   Propeller speed at point of operations, m/s
        OUTPUTS:
            c_p         Coefficient of power
        """
        radius = self.params.diameter / 2
        sigma = self.sigma(self.params.num_blades, self.params.chord_mean, radius)
        lamda = self.lamda(v_ref, omega_ref, radius)
        speed_rps = omega_ref / (2 * np.pi)
        j = self.calc_advance_ratio(v_ref, speed_rps)

        c_p = self._C_P(
            j,
            sigma,
            lamda,
            radius,
            self.params.pitch_nominal,
            self.parasitic_drag,
            self.params.num_blades,
            self.lift_coeff,
            omega_ref,
        )
        return c_p

    @staticmethod
    def _C_P(J, sigma, lam, R, pitch, c_d_p, B, a, omega):
        """
        Propeller coefficient of power
        """
        x_h = 0.15
        stations = np.linspace(x_h, 1, 100)

        def int_argument(x):
            phi_val = PropellerBladeElement.phi(lam, x)
            r_val = R * x
            beta_val = PropellerBladeElement.beta(pitch, r_val)
            phi_val = PropellerBladeElement.phi(lam, x)
            V_T_val = PropellerBladeElement.V_T(omega, R)
            V_r_val = PropellerBladeElement.V_r(V_T_val, x, lam)
            a_i_val = PropellerBladeElement._a_i(
                sigma, x, a, V_r_val, V_T_val, beta_val, lam
            )
            C_l_val = PropellerBladeElement.C_l(a, beta_val, phi_val, a_i_val)
            C_d_val = PropellerBladeElement.C_d(C_l_val, c_d_p, B, sigma)
            return (
                np.pi
                * x
                * (J**2 + np.pi**2 * x**2)
                * sigma
                * (
                    C_l_val * np.sin(phi_val + a_i_val)
                    + C_d_val * np.cos(phi_val + a_i_val)
                )
            )

        y = [int_argument(x) for x in stations]

        return np.pi / 8 * np.trapz(y, stations)

    def calc_efficiency(self, v_ref, omega_ref):
        """
        Calculate the propeller efficiency.

        INPUTS:
            v_ref       Freestream airspeed at point of operation, m/s
            omega_ref   Propeller speed at point of operations, m/s
        OUTPUTS:
            n           Propeller efficiency
        """
        c_t = self.calc_coeff_thrust(v_ref, omega_ref)
        c_p = self.calc_coeff_power(v_ref, omega_ref)
        speed_hz = radps2rps(omega_ref)
        j = self.calc_advance_ratio(v_ref, speed_hz)

        return c_t * j / c_p

    @staticmethod
    def C_l(a, beta, phi, a_i):
        """
        Blade station coefficient of lift
        """
        c_l_0 = 0.0
        return c_l_0 + a * (beta - phi - a_i)

    @staticmethod
    def C_d(C_l, c_d_p, B, sigma):
        """
        Blade station coefficient of drag
        """
        e = 0.9
        ar = B / (sigma * np.pi)  # Equals R/mean_c
        return c_d_p + C_l**2 / (np.pi * e * ar)


def build_propeller(desc: PropellerParameters) -> Propeller:
    """
    Factory method for building Propeller objects.
    """
    if isinstance(desc, PropellerStandardParameters):
        return PropellerStandard(desc)
    elif isinstance(desc, PropellerBladeElementParameters):
        return PropellerBladeElement(desc)
    else:
        raise ValueError(f"Unsupported propeller type {desc.__class__.__name__}.")


class DirectionEnum(str, Enum):
    """
    Rotation direction of a thruster.

    To find the rotation direction, align the thrust direction with your right thumb.
    ccw: rotation complies with right-hand rule.
    cw: rotation does not comply with right-hand rule.
    """

    CCW = "ccw"
    CW = "cw"


class ThrusterTypeEnum(str, Enum):
    """
    Enumeration of the supported thruster types.

    For more information read up on the subclasses of :class:`Thruster`.
    """

    SIMPLE = "simple"
    BEARD = "beard"
    SPEED_CONTROLLED = "speed_controlled"
    ELECTRIC = "electric"
    MOTOR_CURVE = "motor_curve"


class ThrusterUseEnum(str, Enum):
    """
    Enumeration of what the thruster is used for.

    Useful for scripts that want to know how many propellers a VTOL has per mode.
    """

    MULTICOPTER = "multicopter"
    AIRPLANE = "airplane"
    MIXED = "mixed"


class ThrusterParameters(ComponentParameters):
    """Thruster abstract base class. Not to be used directly."""

    rotation_dir: DirectionEnum = DirectionEnum.CCW
    """Rotation direction of the thruster, if applicable."""
    thruster_type: ThrusterTypeEnum = ThrusterTypeEnum.SIMPLE
    """Thruster type."""
    usage: ThrusterUseEnum = ThrusterUseEnum.MIXED
    """What this thruster is used for."""


class ThrusterMeta(type(DynamicSystem), type(Component)):
    pass


class Thruster(DynamicSystem, Component, metaclass=ThrusterMeta):
    """
    The first six elements of its output vector must be the thruster-frame wrench elements.

    IMPORTANT: The constructor of every child class must call the DynamicSystems's constructor.
    """

    def __init__(self, x_0, u_0, desc):
        DynamicSystem.__init__(self, x_0, u_0)
        Component.__init__(self, desc.name)
        yaml_desc = yaml.load(desc.json(), Loader=yaml.SafeLoader)
        self.initialize(yaml.dump(yaml_desc))

        self.params = desc

    @property
    def velocity(self):
        """
        Return zero velocity as default for the thrusters that don't have an internal state.
        """
        return 0

    @velocity.setter
    def velocity(self, omega):
        """
        Don't set any velocity as default for the thrusters that don't have an internal state.
        """
        pass

    @property
    def torque_sign(self) -> int:
        """
        Return the sign of the torque direction.

        Negative for CCW propellers. Positive for CW propellers.
        """
        if self.params.rotation_dir == DirectionEnum.CCW:
            return -1
        else:
            return 1

    @property
    @abstractmethod
    def wrench(self) -> Wrench:
        """
        Return the 3D wrench applied by this thruster.
        Use the last state and input applied.
        Expressed in the Thruster frame.
        """
        pass


class ThrusterSimpleParameters(ThrusterParameters):
    """
    Simple thruster model. Outputs thrust and torque proportional to its input.

    Its parameters are according to these equations:

    .. math::

        f_x = (thrust_{max} - thrust_{min}) \\delta_t + thrust_{min}

        t_x = (torque_{max} - torque_{min}) \\delta_t + torque_{min}

    """

    thrust_min: float = 0
    """Produced thrust at minimum input."""
    thrust_max: float = 0
    """Produced thrust at maximum input."""
    torque_min: float = 0
    """Produced torque at minimum input."""
    torque_max: float = 0
    """Produced torque at maximum input."""


class ThrusterSimple(Thruster):
    """
    Simple thruster model. Outputs thrust and torque proportional to its input.

    It follows these equations:

    .. math::

        f_x = (thrust_{max} - thrust_{min}) \\delta_t + thrust_{min}

        t_x = (torque_{max} - torque_{min}) \\delta_t + torque_{min}

    """

    delta_t: float = 0

    def __init__(self, desc):
        """
        Initialize the ThrusterSimple using an equivalent description.
        """
        super().__init__(np.array([]), np.array([0]), desc)

    def _dynamics(self, x, u, t):
        del x, t
        self.delta_t = u[0]
        return np.array([])  # The simple motor has no dynamics

    def _outputs(self, x, u, t):
        delta_t = u[0]
        if delta_t < 0 or delta_t > 1:
            raise ValueError("Thruster input must be in [0,1].")
        f_x = (
            self.params.thrust_max - self.params.thrust_min
        ) * delta_t + self.params.thrust_min
        t_x = self.torque_sign * (
            (self.params.torque_max - self.params.torque_min) * delta_t
            + self.params.torque_min
        )
        return np.array([f_x, 0, 0, t_x, 0, 0])

    @property
    def wrench(self):
        output = self.y
        force = Vector3(x=output[0], y=output[1], z=output[2])
        torque = Vector3(x=output[3], y=output[4], z=output[5])
        return Wrench(force.to_array(), torque.to_array())


class ThrusterBeardParameters(ThrusterParameters):
    """A simple thruster whose thrust recedes as the incoming airspeed increases.
    Useful to simulate simple fixed-wing propellers.

    Its parameters are according to these equations:

    .. math::

        f_x = 0.5 \\rho k_{thrust} ( (k_{speed} \\delta_t)^2 - V_a^2))

        t_x = k_{torque} (k_{speed} \\delta_t)^2

    """

    k_speed: float = 0
    """The pitch speed coefficient."""
    k_thrust: float = 0
    """The thrust coefficient."""
    k_torque: float = 0
    """The torque coefficient."""


class ThrusterBeard(Thruster):
    """A simple thruster whose thrust recedes as the incoming airspeed increases.
    Useful to simulate simple fixed-wing propellers.

    Its works according to these equations:

    .. math::

        f_x = 0.5 \\rho k_{thrust} ( (k_{speed} \\delta_t)^2 - V_a^2))

        t_x = k_{torque} (k_{speed} \\delta_t)^2

    INPUTS:
        0. Throttle signal (0-1)
        1. Relative airspeed
        2. Air density

    """

    delta_t: float = 0

    def __init__(self, desc):
        """
        Initialize the ThrusterBeard using an equivalent description.
        """
        super().__init__(np.array([]), np.zeros(3), desc)

    def _dynamics(self, x, u, t):
        del x, u, t
        return np.array([])  # The Beard motor has no dynamics

    def _outputs(self, x, u, t):
        delta_t = u[0]
        V_a = u[1]
        rho = u[2]
        f_x = (
            0.5
            * rho
            * self.params.k_thrust
            * ((self.params.k_speed * delta_t) ** 2 - V_a**2)
        )
        t_x = self.torque_sign * (
            self.params.k_torque * (self.params.k_speed * delta_t) ** 2
        )
        return np.array([f_x, 0, 0, t_x, 0, 0])

    @property
    def wrench(self):
        output = self.y
        force = Vector3(x=output[0], y=output[1], z=output[2])
        torque = Vector3(x=output[3], y=output[4], z=output[5])
        return Wrench(force.to_array(), torque.to_array())


class Motor(DynamicSystem, ABC):
    """
    Motor interface class.

    The first state of this DynamicSystem is its rotational velocity.
    """

    mass: float = 0
    """The mass of the motor. Not actually used for mass calculations in the aircraft.
    Set the mass of the parent Thruster instead."""
    J_m: float = 0.001
    """Rotational inertia of the motor."""

    def __init__(self, x_0, u_0, desc):
        super().__init__(x_0, u_0)
        self.mass = desc.mass
        self.J_m = desc.J_m

    @property
    def velocity(self):
        """
        Return the motor rotational velocity, in rad/s.
        """
        return self.x[0]

    @velocity.setter
    def velocity(self, omega):
        """
        Set the motor rotational velocity, in rad/s.
        """
        self.x[0] = omega

    @property
    def velocity_rpm(self):
        """
        Return the motor rotational velocity, in RPM.
        """
        return self.x[0] * 60 / (2 * np.pi)

    @velocity_rpm.setter
    def velocity_rpm(self, rpm):
        """
        Set the motor rotational velocity, in RPM.
        """
        self.x[0] = rpm / 60 * (2 * np.pi)

    @property
    def velocity_rps(self):
        """
        Return the motor rotational velocity, in RPS.
        """
        return self.x[0] / (2 * np.pi)

    @velocity_rps.setter
    def velocity_rps(self, rps):
        """
        Set the motor rotational velocity, in RPS.
        """
        self.x[0] = rps * (2 * np.pi)

    @property
    @abstractmethod
    def power(self):
        raise NotImplementedError


class MotorElectricParameters(BaseModel):
    """
    Model of an electric motor.

    Its parameters are according to the equivalent single-phase circuit:

    .. math::

        \\omega = K_v E

        V_m = I_m R_m + E

        I_m = I_{armature} + I_0

    """

    Kv: float = 0
    """The speed coefficient of the motor, in rad/s/volt."""
    Rm: float = 0
    """The winding resistance of the motor."""
    I0: float = 0
    """The idle current of the motor."""
    mass: Optional[float] = None
    """The mass of the motor. Not actually used for mass calculations in the aircraft.
    Set the mass of the parent Thruster instead."""
    J_m: float = 0.001
    """Rotational inertia of the motor, in kg*m^2."""


class MotorElectric(Motor):
    """
    Model of an electric motor.

    Its parameters are according to the equivalent single-phase circuit:

    .. math::

        \\omega = K_v E

        V_m = I_m R_m + E

        I_m = I_{armature} + I_0

    or equivalently in a dynamic system expression:

    .. math::

        E = \\omega / K_v

        I_m = (V_m - E) / R_m

        I_{armature} = I_m - I_0

        T_m = I_{armature} K_T = I_{armature} / K_V

        \\dot{\\omega} = (T_m - T_L) / J_m

    INPUTS
        0. Voltage
        1. Load torque

    """

    K_v: float = None
    """The speed coefficient of the motor, in rad/s/volt."""
    R_m: float = 0
    """The winding resistance of the motor."""
    I_0: float = 0
    """The idle current of the motor."""

    def __init__(self, desc):
        super().__init__(np.array([0]), np.zeros(2), desc)

        self.K_v = desc.Kv
        self.R_m = desc.Rm
        self.I_0 = desc.I0

    def _calc_current(self, x, u, t):
        del t
        omega = x[0]
        V_m = u[0]
        E = omega / self.K_v
        I_m = (V_m - E) / self.R_m
        return I_m

    def _dynamics(self, x, u, t):
        T_L = u[1]
        I_m = self._calc_current(x, u, t)
        # Test if the motor current exceeds the idle current
        if np.abs(I_m) < self.I_0:  # Armament not activated
            I_a = 0
        else:  # Motor in normal operation, works for regeneration as well
            I_a = I_m - self.I_0
        T_m = I_a / self.K_v
        dot_omega = (T_m - T_L) / self.J_m

        return np.array([dot_omega])

    def _outputs(self, x, u, t):
        del t
        omega = x[0]
        I_m = self.current
        return np.array([omega, I_m])

    @property
    def current(self):
        return self._calc_current(self.x, self.u, self.t)

    @property
    def power(self):
        V_m = self.u[0]
        I_m = self.current
        power = I_m * V_m
        return power


# Replace this with a "find steady-state" method
# def get_motor_speed_from_voltage(Mt, Me, Rs, Km, v_bat, omega_pitch):
#     """
#     Calculate motor speed by input voltage, as a solution to the system:
#     1. T_prop = Km (omega-omega_pitch)^2
#     2. T_mot = Mt I_mot
#     3. V_em = Me omega
#     4. I_mot = (v_bat - v_no_load) / Rs
#     Or equivalently the solution to the quadratic equation
#     Km (omega - omega_pitch)^2 + Mt Me / Rs w - Mt / Rs (v_bat - V_em) = 0
#     """
#     a = Km
#     b = -2 * Km * omega_pitch + Mt * Me / Rs
#     c = Km * omega_pitch ** 2 - Mt * v_bat / Rs
#     delta = b ** 2 - 4 * a * c
#     # Using only positive discriminant, since it usually dominates the nominator
#     omega = (-b + np.sqrt(delta)) / (2 * a)
#     return omega


class ThrusterElectricParameters(ThrusterParameters):
    """
    A thruster as a combination of an electric motor and a propeller.
    """

    motor: MotorElectricParameters = Field(default=MotorElectricParameters())
    """An electric motor definition."""
    propeller: Union[
        PropellerStandardParameters, PropellerBladeElementParameters
    ] = Field(default=PropellerStandardParameters())
    """A propeller definition."""


class ThrusterElectric(Thruster):
    """
    A thruster as a combination of an electric motor and a propeller.

    STATES:
        0. The motor rotational velocity

    INPUTS:
        0. Input Voltage
        1. Relative wind velocity X, positive on normal flight for airplanes
        2. Air density

    SUBSYSTEMS:
        * A MotorElectric
    """

    def __init__(self, desc):
        self._x = np.array([0])
        self._u = np.array([0, 0, 0])
        self.motor: MotorElectric = MotorElectric(desc.motor)
        """An electric motor definition or name."""
        self.propeller: Propeller = PropellerStandard(desc.propeller)
        """A propeller definition or name."""

        super().__init__(self.x, self.u, desc)

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, x: np.ndarray):
        """
        Update the own state and the subsystems state.
        """
        self._x = x
        self.motor.x = x

    @property
    def u(self):
        return self._u

    @u.setter
    def u(self, u: np.ndarray):
        """
        Update the own input and the subsystems inputs.
        """
        self._u = u
        voltage = u[0]
        V_a = u[1]
        rho = u[2]
        rps = self.motor.velocity_rps
        torque = self.propeller.calc_wrench(V_a, rps, rho).torque[0]
        self.motor.u = np.array([voltage, torque])

    @property
    def wrench(self):
        """
        Return the 3D wrench applied by this thruster.
        Expressed in the Thruster frame.
        """
        V_a = self.u[1]
        rho = self.u[2]
        rps = self.motor.velocity_rps
        return self.propeller.calc_wrench(V_a, rps, rho)

    def _dynamics(self, x, u, t):
        V_m = u[0]
        rps = self.motor.velocity_rps
        V_a = u[1]
        rho = u[2]
        T_m = self.propeller.calc_wrench(V_a, rps, rho).torque[0]

        # Calculate the motor submodel dynamics
        x_motor = np.array([x[0]])
        u_motor = np.array([V_m, T_m])
        dot_state_motor = self.motor._dynamics(x_motor, u_motor, t)

        return np.array([dot_state_motor[0]])

    def _outputs(self, x, u, t):
        V_a = u[1]
        rho = u[2]
        rps = self.motor.velocity_rps
        wrench = self.propeller.calc_wrench(V_a, rps, rho)

        omega = self.motor.velocity
        current = self.motor.current

        return np.array(
            [
                wrench.force[0],
                wrench.force[1],
                wrench.force[2],
                self.torque_sign * wrench.torque[0],
                wrench.torque[1],
                wrench.torque[2],
                omega,
                current,
            ]
        )

    @property
    def current(self):
        return self.motor.current

    @property
    def velocity(self):
        return self.motor.velocity


class ThrusterDisk:
    pass


def get_disk_power(diam, rho, u0, thrust):
    """
    Calculate ideal power generating by disk, using momentum theory.
    Compensate this ideal result at will. 15% increase is suggested.
    Taken from https://web.mit.edu/16.unified/www/FALL/thermodynamics/notes/node86.html
    This should be equivalent to McCormick eq. 6.22.
    """
    prop_area = np.pi * (diam / 2) ** 2
    exit_velocity_ratio = thrust / (prop_area * u0**2 * rho / 2) + 1
    power = 0.5 * thrust * u0 * (exit_velocity_ratio**0.5 + 1)
    return power


class ThrusterSpeedControlledParameters(ThrusterParameters):
    """
    A thruster as a standalone propeller.

    It is assumed that the motor is strong enough to turn it and a speed controller
    circuit regulates the desired angular velocity.
    """

    propeller: Union[
        PropellerStandardParameters, PropellerBladeElementParameters
    ] = Field(default=PropellerStandardParameters())
    """A propeller definition."""
    velocity_max: float = 0
    """The maximum propeller velocity, in rad/s."""


class ThrusterSpeedControlled(Thruster):
    """
    A thruster as a standalone propeller.

    It is assumed that the motor is strong enough to turn it and a speed controller
    circuit regulates the desired angular velocity.

    INPUTS:
        0. Input velocity, rad/s
        1. Relative wind velocity X
        2. Air density, kg/m^3
    """

    def __init__(self, desc):
        super().__init__(np.array([]), np.zeros(3), desc)
        self.propeller: Propeller = PropellerStandard(desc.propeller)
        self.velocity_max: float = desc.velocity_max
        """A propeller definition or name."""

    @property
    def wrench(self):
        """
        Return the 3D wrench applied by this thrusters.
        Expressed in the Thruster frame.
        """
        rps = radps2rps(self.u[0])
        V_a = self.u[1]
        rho = self.u[2]
        return self.propeller.calc_wrench(V_a, rps, rho)

    def _dynamics(self, x, u, t):
        return np.array([])  # ThrusterElectric has no dynamics of its own.

    def _outputs(self, x, u, t):
        omega = self.u[0]
        V_a = u[1]
        rho = u[2]

        rps = radps2rps(self.u[0])
        wrench = self.propeller.calc_wrench(V_a, rps, rho)

        return np.array(
            [
                wrench.force[0],
                wrench.force[1],
                wrench.force[2],
                self.torque_sign * wrench.torque[0],
                wrench.torque[1],
                wrench.torque[2],
                omega,
            ]
        )


class ThrusterMotorCurve(Thruster):
    """
    A thruster as a combination of a motor and a propeller.

    The motor is defined by its power characteristic curve.
    """

    pass


def build_thruster(desc: ThrusterParameters):
    """
    Factory method for building Thruster objects.
    """
    if isinstance(desc, ThrusterSimpleParameters):
        return ThrusterSimple(desc)
    elif isinstance(desc, ThrusterBeardParameters):
        return ThrusterBeard(desc)
    elif isinstance(desc, ThrusterElectricParameters):
        return ThrusterElectric(desc)
    elif isinstance(desc, ThrusterSpeedControlledParameters):
        return ThrusterSpeedControlled(desc)
    elif isinstance(desc, ThrusterMotorCurveParameters):
        raise NotImplementedError
    else:
        raise ValueError(f"Unsupported thruster type {desc.__class__.__name__}.")


class BatteryChemistryEnum(str, Enum):
    """
    Enumeration of the supported battery chemistries.
    """

    lipo = "LiPo"


def get_cell_voltage_nominal(chemistry: BatteryChemistryEnum):
    """
    Return nominal cell voltage per battery chemistry.
    """

    if chemistry == BatteryChemistryEnum.lipo:
        return 3.75
    else:
        raise ValueError(f"Unknown chemistry {chemistry}.")


class BatterySpec(BaseModel):
    """
    The specification of a battery.
    """

    chemistry: BatteryChemistryEnum = BatteryChemistryEnum.lipo
    """The battery chemistry."""
    cells: int = 3
    """The number of battery cells."""
    capacity_ah: Optional[float] = None
    """The actual battery capacity. It may be less than the nominal capacity rating."""
    capacity_nominal_ah: float = 10
    """The nominal battery capacity."""
    resistance: Optional[float] = 0
    """The internal resistance of the battery."""


class BatteryParameters(ComponentParameters):
    """
    A battery component.
    """

    spec: BatterySpec = Field(default=BatterySpec())
    """The specification of the battery or a reference to an existing model."""


class Battery(Component):
    """A battery component

    Currently is only usable as a point mass.
    """

    def __init__(self, desc: BatteryParameters):
        """
        Initialize a Battery model.

        Uses a propulsion.BatteryParameters class as data input.
        """

        yaml_desc = yaml.load(desc.json(), Loader=yaml.SafeLoader)
        Component.__init__(desc.name)
        self.initialize(yaml.dump(yaml_desc))

        # TODO: Fully flesh out a battery DynamicSystem

    @property
    def cell_voltage_nominal(self):
        return get_cell_voltage_nominal(self.spec.chemistry)

    @property
    def energy_density_nominal_wh_kg(self):
        """
        Return the nominal energy density of the battery in Wh/kg. Based on the supplier's nominal cell voltage anb total capacity.
        """
        energy_density_nominal_wh_kg = (
            self.spec.cells
            * self.cell_voltage_nominal
            * self.spec.capacity_nominal_ah
            / self.inertial.mass
        )
        return energy_density_nominal_wh_kg
