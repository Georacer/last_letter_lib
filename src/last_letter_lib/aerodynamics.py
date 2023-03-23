#!/usr/bin/env python3
"""Aerodynamics-related libraries

Contains functions and classes that relate to aerodynamics calculations.

usage:
    Import the module for use in other scripts.
"""

__authors__ = ["George Zogopoulos"]
__credits__ = []
__date__ = "Fri 24 Dec 2022"
__copyright__ = "Copyright 2022, Avy B.V."

import copy
import dataclasses
from dataclasses import dataclass
from dataclasses import field
from typing import List
from typing import Optional

import numpy as np
from pydantic import Field
from scipy.optimize import minimize_scalar

from last_letter_lib.environment import EnvironmentData
from last_letter_lib.systems import Component
from last_letter_lib.systems import ComponentParameters
from last_letter_lib.utils.math import Vector3
from last_letter_lib.utils.math import Wrench
from last_letter_lib.utils.math import build_vector3_from_array
from last_letter_lib.utils.math import sigmoid
from last_letter_lib.utils.uav import Airdata
from last_letter_lib.utils.uav import Inputs
from last_letter_lib.utils.uav import UavState


def calc_dynamic_pressure(rho, v_a):
    """
    Calculate dynamic pressure.
    INPUTS:
    rho: air density, kg/m^3
    v_a: airspeed, m/s
    OUTPUTS:
    dynamic pressure, Pa
    """
    return 0.5 * rho * v_a**2


def calc_max_l_d(Cl_coeffs, Cd_coeffs, xtol=0.0001):
    """
    Find the maximum lift over drag ratio and the corresponding AoA.

    Calculate the optimum angle of attack at which best lift over drag ratio is achieved.
    This coincides with the best range operational point.

    INPUTS:     Ct_coeffs: an iterator with the polynomial coefficients of the lift curve.
                    Higest degree first. Sized for radians.
                Cp_coeffs: an iterator with the polynomial coefficients of the drag curve.
                    Higest degree first. Sized for radians.
                xtol: the accuracy of the optimal AoA, in radians.
    OUTPUTS:    best_l_d: optimal lift/drag ratio
                best_aoa: optimal angle of attack angle
    """

    def cost_function(x):
        return -np.polyval(Cl_coeffs, x) / np.polyval(Cd_coeffs, x)

    method = "brent"
    options = dict()
    options["xtol"] = xtol
    glide_ratio_bounds = 30

    try:
        optim_res = minimize_scalar(cost_function, method=method, options=options)
    except RuntimeWarning:  # Ensure there is no number overflow warning, signifying failure
        raise RuntimeError("Errors occurred during optimization.")
    if abs(optim_res.fun) > glide_ratio_bounds:
        optim_res.success = False
    if not optim_res.success:
        raise RuntimeError("Could not find an optimal glide solution.")

    return (-optim_res.fun, optim_res.x)


def calc_bank_from_radius(R, v, gamma, g):
    """
    Calculate the required bank angle to sustain a turn.
    Source: Beard, R. W., & McLain, T. W. (2012). Small Unmanned Aircraft: Theory and Practice. Eq. 5.16.

    INPUTS:
        R       The turn radius, m
        v       The ground speed, m/s
        gamma   The flight path angle, rad
        g       Gravity acceleration, m/s^2

    Assumes a coordinated turn.
    """
    return np.arctan2(v**2 * np.cos(gamma), R * g)


class AerodynamicParameters(ComponentParameters):
    """A component with aerodynamic behaviour.

    Note:
    * All polynomial coefficients are expected to have a zero offset. There are separate scalar parameters to assign the offset value.
    """

    s: float = 0.45
    """Total surface of the aerodynamic aera."""
    b: float = 1.88
    """Wingspan."""
    c: float = 0.24
    """Mean wing chord."""

    c_L_0: float = 0.4
    """Constant Lift coefficient."""
    c_L_alpha: List[float] = Field(default=[6.5, 0])
    """Coefficient of lift as a polynomial of angle of attack, starting with
    highest order term. Angle of attack assumed to be in rad."""
    c_L_beta: List[float] = Field(default=[0])
    """Coefficient of lift as a polynomial of absolute angle of sideslip, starting with
    highest order term. Angle of attack assumed to be in rad.

    c = c_L_beta * abs(beta)
    """
    c_L_qn: List[float] = Field(default=[0])
    """Coefficient of lift in terms of normalized pitch rate."""
    c_L_deltae: List[float] = Field(default=[0])
    """Coefficient of lift in terms of elevator deflection. Deflection assumed
    in radians."""
    c_D_0: float = 0.09
    """Constant Drag coefficient."""
    c_D_alpha: List[float] = Field(default=[0.14, 0])
    """Coefficient of drag as a polynomial of angle of attack, starting with
    highest order term. Angle of attack assumed to be in rad."""
    c_D_beta: List[float] = Field(default=[0])
    """Coefficient of drag in terms of angle of sideslip. Assumed symmetic around x=0. Angle assumed to be in rad."""
    c_D_deltae: List[float] = Field(default=[0])
    """Coefficient of drag in terms of elevator deflection angle. Angle assumed to be in rad."""
    c_D_qn: List[float] = Field(default=[0])
    """Coefficient of drag in terms of normalized pitch rate."""
    c_Y_0: float = 0
    """Constant sideforce coefficient."""
    c_Y_beta: List[float] = Field(default=[0])
    """Sideforce coefficient in terms of angle of sideslip. Assumed symmetic around x=0."""
    c_Y_pn: List[float] = Field(default=[0])
    """Sideforce coefficient in terms of normalized roll rate."""
    c_Y_rn: List[float] = Field(default=[0])
    """Sideforce coefficient in terms of normalized yaw rate."""
    c_Y_deltaa: List[float] = Field(default=[0])
    """Sideforce coefficient in terms of aileron deflection. Deflection assumed
    in radians."""
    c_Y_deltar: List[float] = Field(default=[0])
    """Sideforce coefficient in terms of rudder deflection. Deflection assumed
    in radians."""
    c_l_0: float = 0
    """Constant rolling moment coefficient."""
    c_l_beta: List[float] = Field(default=[0])
    """Rolling moment coefficient in terms of angle of sideslip. Assumed symmetic around x=0."""
    c_l_pn: List[float] = Field(default=[0])
    """Rolling moment coefficient in terms of normalized pitch rate."""
    c_l_rn: List[float] = Field(default=[0])
    """Rolling moment coefficient in terms of normalized yaw rate."""
    c_l_deltaa: List[float] = Field(default=[0])
    """Rolling moment coefficient in terms of aileron deflection. Deflection assumed
    in radians."""
    c_l_deltar: List[float] = Field(default=[0])
    """Rolling moment coefficient in terms of rudder deflection. Deflection assumed
    in radians."""
    c_m_0: float = 0
    """Constant pitching moment coefficient."""
    c_m_alpha: List[float] = Field(default=[0])
    """Pitching moment coefficient as a polynomial of angle of attack, starting with
    highest order term. Angle of attack assumed to be in rad."""
    c_m_beta: List[float] = Field(default=[0])
    """Pitching moment coefficient as a polynomial of absolute angle of sideslip, starting with
    highest order term. Angle of attack assumed to be in rad.

    c = c_m_beta * abs(beta)
    """
    c_m_qn: List[float] = Field(default=[0])
    """Pitching moment coefficient in terms of normalized pitch rate."""
    c_m_deltae: List[float] = Field(default=[0])
    """Pitching moment coefficient in terms of elevator deflection. Deflection assumed
    in radians."""
    c_n_0: float = 0
    """Constant yawing moment coefficient."""
    c_n_beta: List[float] = Field(default=[0])
    """Yawing moment coefficient in terms of angle of sideslip. Assumed symmetic around x=0."""
    c_n_pn: List[float] = Field(default=[0])
    """Yawing moment coefficient in terms of normalized roll rate."""
    c_n_rn: List[float] = Field(default=[0])
    """Yawing moment coefficient in terms of normalized yaw rate."""
    c_n_deltaa: List[float] = Field(default=[0])
    """Yawing moment coefficient in terms of aileron deflection. Deflection assumed
    in radians."""
    c_n_deltar: List[float] = Field(default=[0])
    """Yawing moment coefficient in terms of rudder deflection. Deflection assumed
    in radians."""
    alpha_stall: float = np.deg2rad(15)
    """Stall angle, in radians."""
    alpha_stall_neg: float = np.deg2rad(-10)
    """Negative stall angle, in radians."""
    stall_width: float = 100
    """
    Parameter controlling how "wide" the zone around the stall angle is.
    Bigger values make for narrower transition.
    """
    delta_a_max: float = np.deg2rad(25)
    """Maximum aileron deflection, in radians."""
    delta_e_max: float = np.deg2rad(25)
    """Maximum elevator deflection, in radians."""
    delta_r_max: float = np.deg2rad(25)
    """Maximum rudder deflection, in radians."""

    def _validate_zero_offset_polynomial(cls, name, value):
        if not (v[-1] == 0):
            raise ValueError(
                f"Polynomial {name} must have the zero-order coefficient equal to 0."
            )
        return v

    def __post__init__(self):
        zero_order_polys = [
            "c_L_alpha",
            "c_L_qn",
            "c_L_deltae",
            "c_D_alpha",
            "c_D_beta",
            "c_D_deltae",
            "c_D_qn",
            "c_Y_beta",
            "c_Y_pn",
            "c_Y_rn",
            "c_Y_deltaa",
            "c_Y_deltar",
            "c_l_beta",
            "c_l_pn",
            "c_l_rn",
            "c_l_deltaa",
            "c_l_deltar",
            "c_m_alpha",
            "c_m_qn",
            "c_m_deltae",
            "c_n_beta",
            "c_n_pn",
            "c_n_rn",
            "c_n_deltaa",
            "c_n_deltar",
        ]

        for poly_name in zero_order_polys:
            poly_value = getattr(self, poly_name)
            self._validate_zero_offset_polynomial(poly_name, poly_value)


class Aerodynamic(Component):
    """A component with aerodynamic behaviour.

    Can be the lumped aerodynamic model of the aircraft. In this case, it is
    suggested that only one :class:`Aerodynamic` is defined.

    Alternatively, it can represent a single airfoil. In this case, many
    airfroils can be added to a single :class:`Aircraft`.
    """

    airdata: Airdata = Airdata()
    params: AerodynamicParameters

    def __init__(self, desc: AerodynamicParameters):
        """
        Initialize an Aerodynamic model.

        Uses a AerodynamicParameters class as data input.
        """

        self.params = desc

        super().__init__(desc)

    def _lift_coeff_alpha(self, alpha):
        s = sigmoid(alpha, self.params.alpha_stall, self.params.stall_width)
        linear = np.polyval(self.params.c_L_alpha, alpha) + self.params.c_L_0
        flat_plate = (
            2 * np.sign(alpha) * np.power(np.sin(alpha), 2) * np.cos(alpha)
        )  # Lift beyond stall
        return (1 - s) * linear + s * flat_plate

    def lift_coeff(self, af_state, environment, u):
        """
        Calculate the lift coefficient in the wind frame.
        """
        alpha = self.airdata.alpha
        beta = self.airdata.beta
        qn = af_state.velocity_angular.y * self.params.c / (2 * self.airdata.airspeed)
        c_lift = (
            self._lift_coeff_alpha(alpha)
            + np.polyval(self.params.c_L_beta, abs(beta))
            + np.polyval(self.params.c_L_qn, qn)
            + np.polyval(self.params.c_L_deltae, u[1])
        )
        return c_lift

    def _drag_coeff_alpha(self, alpha):
        s = sigmoid(alpha, self.params.alpha_stall, self.params.stall_width)
        linear = np.polyval(self.params.c_D_alpha, alpha) + self.params.c_D_0
        flat_plate = (
            2 * np.sign(alpha) * np.power(np.sin(alpha), 3)
        )  # Drag beyond stall
        drag_coeff_alpha = (1 - s) * linear + s * flat_plate
        return drag_coeff_alpha

    def drag_coeff(self, af_state, environment, u):
        """
        Calculate the drag coefficient in the wind frame.
        """
        alpha = self.airdata.alpha
        beta = self.airdata.beta
        # If AoS is >90deg, revert it so that its effect diminishes as it approaches 180deg
        if beta > np.pi / 2:
            beta = np.pi - beta
        # If AoS is <-90deg, revert it so that its effect diminishes as it approaches -180deg
        if beta < -np.pi / 2:
            beta = -np.pi - beta

        qn = af_state.velocity_angular.y * self.params.c / (2 * self.airdata.airspeed)
        deltae = u[1]
        drag_coeff = (
            self._drag_coeff_alpha(alpha)
            + np.polyval(self.params.c_D_beta, np.abs(beta))
            + np.polyval(self.params.c_D_qn, qn)
            + np.polyval(self.params.c_D_deltae, deltae)
        )
        return drag_coeff

    def sideforce_coeff(self, af_state, environment, u):
        V_a = self.airdata.airspeed

        beta = self.airdata.beta
        # If AoS is >90deg, revert it so that its effect diminishes as it approaches 180deg
        if beta > np.pi / 2:
            beta = np.pi - beta
        # If AoS is <-90deg, revert it so that its effect diminishes as it approaches -180deg
        if beta < -np.pi / 2:
            beta = -np.pi - beta
        # Sideforce should also be 0 at 90deg and -90 AoS.
        beta = beta * np.cos(beta) ** 2

        p, q, r = af_state.velocity_angular
        da, de, dr = u
        return (
            self.params.c_Y_0
            + np.polyval(self.params.c_Y_beta, beta)
            + np.polyval(self.params.c_Y_pn, self.params.b * p / (2 * V_a))
            + np.polyval(self.params.c_Y_rn, self.params.b * r / (2 * V_a))
            + np.polyval(self.params.c_Y_deltaa, da)
            + np.polyval(self.params.c_Y_deltar, dr)
        )

    def roll_moment_coeff(self, af_state, environment, u):
        V_a = self.airdata.airspeed

        beta = self.airdata.beta
        # If AoS is >90deg, revert it so that its effect diminishes as it approaches 180deg
        if beta > np.pi / 2:
            beta = np.pi - beta
        # If AoS is <-90deg, revert it so that its effect diminishes as it approaches -180deg
        if beta < -np.pi / 2:
            beta = -np.pi - beta

        p, q, r = af_state.velocity_angular
        da, de, dr = u
        return (
            self.params.c_l_0
            + np.polyval(self.params.c_l_beta, beta)
            + np.polyval(self.params.c_l_pn, self.params.b * p / (2 * V_a))
            + np.polyval(self.params.c_l_rn, self.params.b * r / (2 * V_a))
            + np.polyval(self.params.c_l_deltaa, da)
            + np.polyval(self.params.c_l_deltar, dr)
        )

    def pitch_moment_coeff(self, af_state, environment, u):
        V_a = self.airdata.airspeed

        alpha = self.airdata.alpha
        beta = self.airdata.beta
        # If AoA is >90deg, revert it so that its effect diminishes as it approaches 180deg
        if alpha > np.pi / 2:
            alpha = np.pi - alpha
        # If AoA is <-90deg, revert it so that its effect diminishes as it approaches -180deg
        if alpha < -np.pi / 2:
            alpha = -np.pi - alpha

        p, q, r = af_state.velocity_angular
        da, de, dr = u
        return (
            self.params.c_m_0
            + np.polyval(self.params.c_m_alpha, alpha)
            + np.polyval(self.params.c_m_beta, abs(beta))
            + np.polyval(self.params.c_m_qn, self.params.c * q / (2 * V_a))
            + np.polyval(self.params.c_m_deltae, de)
        )

    def yaw_moment_coeff(self, af_state, environment, u):
        V_a = self.airdata.airspeed

        beta = self.airdata.beta
        # If AoS is >90deg, revert it so that its effect diminishes as it approaches 180deg
        if beta > np.pi / 2:
            beta = np.pi - beta
        # If AoS is <-90deg, revert it so that its effect diminishes as it approaches -180deg
        if beta < -np.pi / 2:
            beta = -np.pi - beta

        p, q, r = af_state.velocity_angular
        da, de, dr = u
        return (
            self.params.c_n_0
            + np.polyval(self.params.c_n_beta, beta)
            + np.polyval(self.params.c_n_pn, self.params.b * p / (2 * V_a))
            + np.polyval(self.params.c_n_rn, self.params.b * r / (2 * V_a))
            + np.polyval(self.params.c_n_deltaa, da)
            + np.polyval(self.params.c_n_deltar, dr)
        )

    def calc_forces(
        self, af_state: UavState, environment: EnvironmentData, u: np.array
    ) -> Vector3:
        """
        Calculate the aerodynamic forces in the airfoil frame.

        INPUTS:
            uav_state: a UavState dataclass
            environment: an EnvironmentData dataclass
            u: The control surface deflections as [delta_a, delta_e, delta_r], in radians
        """

        # Read wind off of environment
        V_a = self.airdata.airspeed
        qbar = calc_dynamic_pressure(environment.rho, V_a)
        # Read linear, angular velocities off of uav_state
        p, q, r = af_state.velocity_angular

        L = qbar * self.params.s * self.lift_coeff(af_state, environment, u)
        D = qbar * self.params.s * self.drag_coeff(af_state, environment, u)
        Y = qbar * self.params.s * self.sideforce_coeff(af_state, environment, u)

        return Vector3(-D, -Y, -L)

    def calc_moments(
        self, af_state: UavState, environment: EnvironmentData, u: np.array
    ):
        """
        Calculate the aerodynamic moments in the airfoil frame.

        INPUTS:
            uav_state: a UavState dataclass
            environment: an EnvironmentData dataclass
            u: The control surface deflections as [delta_a, delta_e, delta_r], in radians
        """
        # Read wind off of environment
        V_a = self.airdata.airspeed
        qbar = calc_dynamic_pressure(environment.rho, V_a)
        # Read linear, angular velocities off of uav_state

        M_roll = (
            qbar
            * self.params.s
            * self.params.b
            * self.roll_moment_coeff(af_state, environment, u)
        )
        M_pitch = (
            qbar
            * self.params.s
            * self.params.c
            * self.pitch_moment_coeff(af_state, environment, u)
        )
        M_yaw = (
            qbar
            * self.params.s
            * self.params.b
            * self.yaw_moment_coeff(af_state, environment, u)
        )

        return Vector3(M_roll, M_pitch, M_yaw)

    def _store_airdata(self, airdata):
        self.airdata = airdata
        # Make sure airspeed is never exactly 0, because it will lead to zero-divisions
        if self.airdata.airspeed == 0:
            self.airdata.airspeed = 1e-6

    def get_wrench_airfoil(
        self, af_state: UavState, environment: EnvironmentData, u: Inputs
    ) -> Wrench:
        """
        Calculate the aerodynamic wrench in the local airfoil frame
        """
        # Test if airspeed is zero
        self._store_airdata(Airdata.from_state_environment(af_state, environment))
        V_a = self.airdata.airspeed
        if V_a == 0:
            # If yes, then return zero wrench
            return Wrench()

        # Collect aerodynamic inputs
        u = [
            u.delta_a * self.params.delta_a_max,
            u.delta_e * self.params.delta_e_max,
            u.delta_r * self.params.delta_r_max,
        ]
        force_wind = self.calc_forces(af_state, environment, u)
        force = build_vector3_from_array(self.airdata.S_wb @ force_wind.to_array())
        torque = self.calc_moments(af_state, environment, u)
        return Wrench(force, torque)

    def get_wrench(
        self, uav_state: UavState, environment: EnvironmentData, u: Inputs
    ) -> Wrench:
        """
        Calculate the aerodynamic wrench in the UAV aircraft frame.

        Wrapper to transform aircraft-frame quantities to airfoil-frame.
        """
        # Copy over the uav state
        af_state = copy.deepcopy(uav_state)
        # Transform the uav state and environment into the airfoil frame
        af_state.position += self.pose.orientation * self.pose.position
        af_state.attitude *= self.pose.orientation
        af_state.velocity_linear = build_vector3_from_array(
            self.pose.orientation.conjugate() * uav_state.velocity_linear.to_array()
            + np.cross(
                uav_state.velocity_angular.to_array(), self.pose.position.to_array()
            )
        )
        af_state.velocity_angular = (
            self.pose.orientation.conjugate() * af_state.velocity_angular
        )
        wrench_airfoil = self.get_wrench_airfoil(af_state, environment, u)
        return self.pose @ wrench_airfoil
