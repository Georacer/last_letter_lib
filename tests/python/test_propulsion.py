#!/usr/bin/env python3
"""Tests for the propulsion module.

usage:
    Have pytest run the tests in this file.
"""

__authors__ = ["George Zogopoulos"]
__credits__ = []
__date__ = "Fri 12 Jan 2022"
__copyright__ = "Copyright 2021, George Zogopoulos"

import numpy as np
import pytest

from last_letter_lib import propulsion
from last_letter_lib import systems
from last_letter_lib.utils import math
from last_letter_lib.utils.math import UnitQuaternion
from last_letter_lib.utils.math import Vector3


EPS = 1e-6

###############################################################################
# Generate data for passing them to the propulsion functionality #######


@pytest.fixture
def build_desc_propeller():
    """
    Build a 16x8 propeller
    """
    return propulsion.PropellerStandardParameters(
        diameter=16 * 0.0254,
        pitch=8 * 0.0254,
        c_thrust=[-0.161243, -0.045845, 0.064295],
        c_power=[-0.084206, 0.016194, 0.018441],
    )


@pytest.fixture
def build_desc_propeller_be():
    """
    Build a blade element propeller
    """
    return propulsion.PropellerBladeElementParameters(
        diameter=16 * 0.0254, pitch_angle=8 * 0.0254, num_blades=2, chord_mean=0.03
    )


@pytest.fixture
def build_desc_motor_electric():
    return propulsion.MotorElectricParameters(
        Kv=math.rpm2radps(390), Rm=0.1, I0=0.1, mass=0.3, J_m=200e-6
    )


@pytest.fixture
def build_desc_thruster_simple():
    return propulsion.ThrusterSimpleParameters(
        name="thruster_simple",
        pose=systems.PoseParameters(),
        rotation_dir=propulsion.DirectionEnum.CW,
        thruster_type=propulsion.ThrusterTypeEnum.SIMPLE,
        usage=propulsion.ThrusterUseEnum.AIRPLANE,
        thrust_min=0,
        thrust_max=50,
        torque_min=0,
        torque_max=1,
    )


@pytest.fixture
def build_desc_thruster_beard():
    return propulsion.ThrusterBeardParameters(
        name="thruster_beard",
        pose=systems.PoseParameters(),
        rotation_dir=propulsion.DirectionEnum.CW,
        thruster_type=propulsion.ThrusterTypeEnum.BEARD,
        usage=propulsion.ThrusterUseEnum.AIRPLANE,
        k_speed=30,
        k_thrust=0.01,
        k_torque=0.001,
    )


@pytest.fixture
def build_desc_thruster_electric(build_desc_motor_electric, build_desc_propeller):
    prop_desc = build_desc_propeller
    motor_desc = build_desc_motor_electric
    return propulsion.ThrusterElectricParameters(
        name="thruster_electric",
        pose=systems.PoseParameters(),
        rotation_dir=propulsion.DirectionEnum.CW,
        thruster_type=propulsion.ThrusterTypeEnum.ELECTRIC,
        usage=propulsion.ThrusterUseEnum.AIRPLANE,
        motor=motor_desc,
        propeller=prop_desc,
    )


@pytest.fixture
def build_desc_thruster_speed_controlled(build_desc_propeller):
    prop_desc = build_desc_propeller
    return propulsion.ThrusterSpeedControlledParameters(
        name="thruster_speed_controlled",
        pose=systems.PoseParameters(),
        rotation_dir=propulsion.DirectionEnum.CW,
        thruster_type=propulsion.ThrusterTypeEnum.SPEED_CONTROLLED,
        usage=propulsion.ThrusterUseEnum.AIRPLANE,
        propeller=prop_desc,
        velocity_max=942,
    )


@pytest.fixture
def build_propeller_standard(build_desc_propeller):
    return propulsion.PropellerStandard(build_desc_propeller)


@pytest.fixture
def build_propeller_be(build_desc_propeller_be):
    return propulsion.PropellerBladeElement(build_desc_propeller_be)


@pytest.fixture
def build_propeller_standard_wrench(build_propeller_standard, V, n):
    rho = 1.225
    wrench = build_propeller_standard.calc_wrench(V, n, rho)
    return wrench


@pytest.fixture
def build_motor_electric(build_desc_motor_electric):
    return propulsion.MotorElectric(build_desc_motor_electric)


@pytest.fixture
def build_thruster_simple(build_desc_thruster_simple):
    return propulsion.ThrusterSimple(build_desc_thruster_simple)


@pytest.fixture
def build_thruster_beard(build_desc_thruster_beard):
    return propulsion.ThrusterBeard(build_desc_thruster_beard)


@pytest.fixture
def build_thruster_electric(build_desc_thruster_electric):
    return propulsion.ThrusterElectric(build_desc_thruster_electric)


@pytest.fixture
def build_thruster_speed_controlled(build_desc_thruster_speed_controlled):
    return propulsion.ThrusterSpeedControlled(build_desc_thruster_speed_controlled)


###############################################################################
# Declare the tests themselves ################################################


class TestPropellerStandard:
    def test_constructor(self, build_desc_propeller):
        propeller = propulsion.PropellerStandard(build_desc_propeller)
        assert isinstance(propeller, propulsion.PropellerStandard)

    def test_diameter(self, build_desc_propeller, build_propeller_standard):
        assert build_propeller_standard.params.diameter == build_desc_propeller.diameter

    def test_pitch(self, build_desc_propeller, build_propeller_standard):
        assert build_propeller_standard.params.pitch == build_desc_propeller.pitch

    def test_calc_j(self, build_propeller_standard):
        V = 20  # airspeed
        n = 50  # propeller RPS
        D = build_propeller_standard.params.diameter
        J = V / (n * D)
        tests = []
        tests.append(build_propeller_standard.calc_advance_ratio(V, n) == J)
        tests.append(build_propeller_standard.calc_advance_ratio(0, n) == 0)
        tests.append(build_propeller_standard.calc_advance_ratio(V, 0) == np.infty)
        tests.append(build_propeller_standard.calc_advance_ratio(-V, 0) == -np.infty)
        tests.append(build_propeller_standard.calc_advance_ratio(0, 0) == 0)
        assert np.all(tests)

    def test_c_thrust(self, build_desc_propeller, build_propeller_standard):
        c_thrust_iter = build_desc_propeller.c_thrust
        V = 20  # airspeed
        n = 50  # propeller RPS
        J = build_propeller_standard.calc_advance_ratio(V, n)
        c_thrust = np.polyval(c_thrust_iter, J)
        assert build_propeller_standard.calc_coeff_thrust(J) == c_thrust

    def test_c_power(self, build_desc_propeller, build_propeller_standard):
        c_power_iter = build_desc_propeller.c_power
        V = 20  # airspeed
        n = 50  # propeller RPS
        J = build_propeller_standard.calc_advance_ratio(V, n)
        c_power = np.polyval(c_power_iter, J)
        assert build_propeller_standard.calc_coeff_power(J) == c_power

    def test_c_static_thrust(self, build_desc_propeller, build_propeller_standard):
        c_thrust_iter = build_desc_propeller.c_thrust
        V = 0  # airspeed
        n = 50  # propeller RPS
        J = build_propeller_standard.calc_advance_ratio(V, n)
        assert build_propeller_standard.calc_coeff_thrust(J) == c_thrust_iter[-1]

    def test_c_static_thrust_2(self, build_desc_propeller, build_propeller_standard):
        c_thrust_iter = build_desc_propeller.c_thrust
        V = 0  # airspeed
        n = 0  # propeller RPS
        J = build_propeller_standard.calc_advance_ratio(V, n)
        assert build_propeller_standard.calc_coeff_thrust(J) == c_thrust_iter[-1]

    def test_c_static_power(self, build_desc_propeller, build_propeller_standard):
        c_power_iter = build_desc_propeller.c_power
        V = 0  # airspeed
        n = 50  # propeller RPS
        J = build_propeller_standard.calc_advance_ratio(V, n)
        assert build_propeller_standard.calc_coeff_power(J) == c_power_iter[-1]

    def test_c_static_power_2(self, build_desc_propeller, build_propeller_standard):
        c_power_iter = build_desc_propeller.c_power
        V = 0  # airspeed
        n = 0  # propeller RPS
        J = build_propeller_standard.calc_advance_ratio(V, n)
        assert build_propeller_standard.calc_coeff_power(J) == c_power_iter[-1]

    def test_efficiency(self, build_desc_propeller, build_propeller_standard):
        c_thrust_iter = build_desc_propeller.c_thrust
        c_power_iter = build_desc_propeller.c_power
        V = 20  # airspeed
        n = 50  # propeller RPS
        J = build_propeller_standard.calc_advance_ratio(V, n)
        c_thrust = np.polyval(c_thrust_iter, J)
        c_power = np.polyval(c_power_iter, J)
        efficiency = c_thrust * J / c_power
        assert build_propeller_standard.calc_efficiency(J) == efficiency

    def test_efficiency_zero_V(self, build_propeller_standard):
        J = 0
        assert build_propeller_standard.calc_efficiency(J) == 0

    def test_power(self, build_propeller_standard):
        V = 1  # airspeed
        n = 50  # propeller RPS
        assert build_propeller_standard.calc_power(V, n, 1.225) > 0

    def test_torque(self, build_propeller_standard):
        """
        Verify torque is zero at zero airspeed and RPM
        """
        assert build_propeller_standard.calc_torque(0, 0, 1.225) == 0

    @pytest.mark.parametrize("V, n", [(0, 100)])
    def test_wrench(self, build_propeller_standard_wrench):
        """
        Verify correct constructor instance
        """
        assert isinstance(build_propeller_standard_wrench, math.Wrench)

    @pytest.mark.parametrize("V, n", [(0, 100)])
    def test_wrench_2(self, build_propeller_standard_wrench):
        """
        Verify thrust is only on the positive x-axis in absence of sidewind
        """
        wrench = build_propeller_standard_wrench
        tests = []
        tests.append(wrench.force[0] > 0)
        tests.append(wrench.force[1] == 0)
        tests.append(wrench.force[2] == 0)
        tests.append(wrench.torque[0] > 0)
        tests.append(wrench.torque[1] == 0)
        tests.append(wrench.torque[2] == 0)
        assert np.all(tests)

    @pytest.mark.parametrize("V, n", [(10, 0.001)])
    def test_wrench_3(self, build_propeller_standard_wrench):
        """
        Verify propeller produces negative thrust when free-wheeling
        """
        wrench = build_propeller_standard_wrench
        assert wrench.force[0] < 0

    def test_propeller_factory(self, build_desc_propeller):
        propeller = propulsion.build_propeller(build_desc_propeller)
        assert isinstance(propeller, propulsion.PropellerStandard)


class TestPropellerBladeElement:
    def test_constructor(self, build_desc_propeller_be):
        propeller = propulsion.PropellerBladeElement(build_desc_propeller_be)
        assert isinstance(propeller, propulsion.PropellerBladeElement)

    def test_diameter(self, build_desc_propeller_be, build_propeller_be):
        assert build_propeller_be.params.diameter == build_desc_propeller_be.diameter

    def test_pitch(self, build_desc_propeller_be, build_propeller_be):
        assert (
            build_propeller_be.params.pitch_angle == build_desc_propeller_be.pitch_angle
        )

    def test_num_blades(self, build_desc_propeller_be, build_propeller_be):
        assert (
            build_propeller_be.params.num_blades == build_desc_propeller_be.num_blades
        )

    def test_chord_mean(self, build_desc_propeller_be, build_propeller_be):
        assert (
            build_propeller_be.params.chord_mean == build_desc_propeller_be.chord_mean
        )

    def test_propeller_factory(self, build_desc_propeller_be):
        propeller = propulsion.build_propeller(build_desc_propeller_be)
        assert isinstance(propeller, propulsion.PropellerBladeElement)


class TestMotor:
    # Test MotorElectric class
    def test_motor_electric_constructor(self, build_desc_motor_electric):
        motor = propulsion.MotorElectric(build_desc_motor_electric)
        assert isinstance(motor, propulsion.MotorElectric)

    def test_motor_electric_kv(self, build_desc_motor_electric, build_motor_electric):
        assert build_motor_electric.K_v == build_desc_motor_electric.Kv

    def test_motor_electric_rm(self, build_desc_motor_electric, build_motor_electric):
        assert build_motor_electric.R_m == build_desc_motor_electric.Rm

    def test_motor_electric_i0(self, build_desc_motor_electric, build_motor_electric):
        assert build_motor_electric.I_0 == build_desc_motor_electric.I0

    def test_motor_electric_mass(self, build_desc_motor_electric, build_motor_electric):
        assert build_motor_electric.mass == build_desc_motor_electric.mass

    def test_motor_electric_jm(self, build_desc_motor_electric, build_motor_electric):
        assert build_motor_electric.J_m == build_desc_motor_electric.J_m

    def test_motor_electric_set_velocity(self, build_motor_electric):
        velocity = 10
        motor = build_motor_electric
        motor.velocity = velocity
        assert motor.velocity == velocity

    def test_motor_electric_velocity_conversion_radps2rps(self, build_motor_electric):
        velocity = 10
        motor = build_motor_electric
        motor.velocity = velocity
        assert motor.velocity_rps == math.radps2rps(velocity)

    def test_motor_electric_velocity_conversion_radps2rpm(self, build_motor_electric):
        velocity = 10
        motor = build_motor_electric
        motor.velocity = velocity
        assert motor.velocity_rpm == pytest.approx(math.radps2rpm(velocity))

    def test_motor_electric_current(self, build_motor_electric):
        """
        Verify correct current calculation
        """
        motor = build_motor_electric

        omega = 1
        voltage = 10
        T_load = 0.1

        motor.velocity = omega
        u = np.array([voltage, T_load])
        motor.u = u
        dt = 0.01
        x, t = motor.rk4(u, dt)

        I_m = (voltage - motor.velocity / motor.K_v) / motor.R_m

        assert motor.current == I_m

    def test_motor_electric_power(self, build_motor_electric):
        """
        Verify correct current calculation
        """
        motor = build_motor_electric

        omega = 1
        voltage = 10
        T_load = 0.1

        motor.velocity = omega
        u = np.array([voltage, T_load])
        dt = 0.01
        x, t = motor.rk4(u, dt)

        I_m = (voltage - motor.velocity / motor.K_v) / motor.R_m

        assert motor.power == I_m * voltage

    def test_motor_electric_idle_current(self, build_motor_electric):
        """
        Make sure the motor won't spin until the idle current is exceeded.
        """
        motor = build_motor_electric
        voltage = 0.5 * motor.I_0 * motor.R_m
        T_load = 0
        u = np.array([voltage, T_load])
        dt = 0.01
        x, t = motor.rk4(u, dt)
        omega = x[0]
        assert omega == 0

    def test_motor_electric_idle_accelerate(self, build_motor_electric):
        """
        Make sure the motor spins from idle to the correct direction when voltage is applied
        """
        motor = build_motor_electric
        voltage = 10 * motor.I_0 * motor.R_m
        T_load = 0
        u = np.array([voltage, T_load])
        dt = 0.01
        x, t = motor.rk4(u, dt)
        omega = x[0]
        assert omega > 0

    def test_motor_electric_idle_decelerate(self, build_motor_electric):
        """
        Make sure the motor winds down if input voltage is lower than the back EMF.
        """
        motor = build_motor_electric
        omega = 100
        voltage = 0
        T_load = 0
        u = np.array([voltage, T_load])
        dt = 0.01

        motor.velocity = omega
        x, t = motor.rk4(u, dt)
        omega_new = x[0]
        assert omega_new < omega

    def test_motor_electric_power_regeneration(self, build_motor_electric):
        """
        Verify correct current calculation
        """
        motor = build_motor_electric

        omega = 100
        voltage = 1
        T_load = 0.1

        motor.velocity = omega
        u = np.array([voltage, T_load])
        dt = 0.01
        x, t = motor.rk4(u, dt)

        I_m = (voltage - motor.velocity / motor.K_v) / motor.R_m
        power = I_m * voltage

        assert power < 0

    def test_motor_electric_idle_steady_state(self, build_motor_electric):
        """
        Make sure the motor reaches the steady-state by the rated Kv
        """
        motor = build_motor_electric
        voltage = 10
        T_load = 0
        u = np.array([voltage, T_load])

        dt = 0.01
        t_end = 10
        t = 0
        omega = None
        while t < t_end:
            x, t = motor.rk4(u, dt)
            omega_prev = omega
            omega = x[0]
            t += dt
        assert np.abs(omega - omega_prev) < EPS


class TestThruster:
    # Test ThrusterSimple class
    def test_thruster_simple_constructor(self, build_desc_thruster_simple):
        thruster = propulsion.ThrusterSimple(build_desc_thruster_simple)
        assert isinstance(thruster, propulsion.ThrusterSimple)

    def test_thruster_simple_factory(self, build_desc_thruster_simple):
        thruster = propulsion.build_thruster(build_desc_thruster_simple)
        assert isinstance(thruster, propulsion.ThrusterSimple)

    def test_thruster_simple_normal_operation(self, build_thruster_simple):
        thruster = build_thruster_simple
        delta_t = 0.5
        u = np.array([delta_t])
        dt = 0.01

        x, t = thruster.rk4(u, dt)
        wrench = thruster.wrench

        tests = []
        tests.append(
            wrench.force[0]
            == delta_t * (thruster.params.thrust_max - thruster.params.thrust_min)
            + thruster.params.thrust_min
        )
        tests.append(
            wrench.torque[0]
            == delta_t * (thruster.params.torque_max - thruster.params.torque_min)
            + thruster.params.torque_min
        )
        assert np.all(tests)

    def test_thruster_simple_input_range_min(self, build_thruster_simple):
        thruster = build_thruster_simple
        with pytest.raises(ValueError):
            _, _ = thruster.rk4(np.array([-0.1]), 0.01)
            _ = thruster.y

    def test_thruster_simple_input_range_max(self, build_thruster_simple):
        thruster = build_thruster_simple
        with pytest.raises(ValueError):
            _, _ = thruster.rk4(np.array([1.1]), 0.01)
            _ = thruster.y

    def test_thruster_simple_direction(self, build_thruster_simple):
        thruster = build_thruster_simple
        thruster.params.rotation_dir = "ccw"
        delta_t = 0.5
        u = np.array([delta_t])
        dt = 0.01

        x, t = thruster.rk4(u, dt)
        wrench = thruster.wrench
        assert wrench.torque[0] < 0

    # Test ThrusterBeard class
    def test_thruster_beard_constructor(self, build_desc_thruster_beard):
        thruster = propulsion.ThrusterBeard(build_desc_thruster_beard)
        assert isinstance(thruster, propulsion.ThrusterBeard)

    def test_thruster_beard_factory(self, build_desc_thruster_beard):
        thruster = propulsion.build_thruster(build_desc_thruster_beard)
        assert isinstance(thruster, propulsion.ThrusterBeard)

    def test_thruster_beard_normal_operation(self, build_thruster_beard):
        thruster = build_thruster_beard
        delta_t = 0.5
        V_a = 0
        rho = 1.225
        u = np.array([delta_t, V_a, rho])
        dt = 0.01

        x, t = thruster.rk4(u, dt)
        wrench = thruster.wrench

        tests = []
        tests.append(wrench.force[0] > 0)
        tests.append(wrench.torque[0] > 0)
        assert np.all(tests)

    def test_thruster_beard_overspeed(self, build_thruster_beard):
        thruster = build_thruster_beard
        delta_t = 0.5
        V_a = thruster.params.k_speed + 10
        rho = 1.225
        u = np.array([delta_t, V_a, rho])
        dt = 0.01

        x, t = thruster.rk4(u, dt)
        wrench = thruster.wrench

        tests = []
        tests.append(wrench.force[0] < 0)
        tests.append(wrench.torque[0] > 0)
        assert np.all(tests)

    # Test ThrusterElectric class
    def test_thruster_electric_constructor(self, build_desc_thruster_electric):
        thruster = propulsion.ThrusterElectric(build_desc_thruster_electric)
        assert isinstance(thruster, propulsion.ThrusterElectric)

    def test_thruster_electric_factory(self, build_desc_thruster_electric):
        thruster = propulsion.build_thruster(build_desc_thruster_electric)
        assert isinstance(thruster, propulsion.ThrusterElectric)

    def test_thruster_electric_normal_operation(self, build_thruster_electric):
        thruster = build_thruster_electric
        voltage = 10
        V_a = 0
        rho = 1.225
        u = np.array([voltage, V_a, rho])

        dt = 0.01
        t_end = 1
        t = 0
        while t < t_end:
            x, t = thruster.rk4(u, dt)
            t += dt

        y = thruster.y

        tests = []
        tests.append(y[0] > 0)  # Thrust positive
        tests.append(y[3] > 0)  # Torque positive
        tests.append(y[6] > 0)  # velocity positive
        tests.append(y[7] > 0)  # current positive
        tests.append(
            np.array_equal(thruster.wrench.to_array(), np.array(y[0:6]))
        )  # Wrench is the head of the ouptut

        assert np.all(tests)

    def test_thruster_electric_idle_steady_state(self, build_thruster_electric):
        """
        Make sure the thruster reaches a reasonable steady-state
        """
        thruster = build_thruster_electric
        voltage = 10
        V_a = 0
        rho = 0
        u = np.array([voltage, V_a, rho])

        dt = 0.01
        t_end = 10
        t = 0
        omega = None
        while t < t_end:
            x, t = thruster.rk4(u, dt)
            omega_prev = omega
            omega = thruster.velocity
            t += dt

        tests = []
        tests.append(np.abs(omega - omega_prev) < EPS)  # Speed is constant
        tests.append(omega > math.rpm2radps(1))  # Speed is significant
        tests.append(omega < math.rpm2radps(20000))  # Speed is reasonable

        assert np.all(tests)

    # Test ThrusterSpeedControlled class
    def test_thruster_speed_controlled_constructor(
        self, build_desc_thruster_speed_controlled
    ):
        thruster = propulsion.ThrusterSpeedControlled(
            build_desc_thruster_speed_controlled
        )
        assert isinstance(thruster, propulsion.ThrusterSpeedControlled)

    def test_thruster_speed_controlled_factory(
        self, build_desc_thruster_speed_controlled
    ):
        thruster = propulsion.build_thruster(build_desc_thruster_speed_controlled)
        assert isinstance(thruster, propulsion.ThrusterSpeedControlled)

    def test_thruster_speed_controlled_idle(self, build_thruster_speed_controlled):
        """
        Make sure the thruster produces zero thrust on idle
        """
        thruster = build_thruster_speed_controlled
        omega_ref = 0
        V_a = 0
        rho = 1.225
        u = np.array([omega_ref, V_a, rho])

        dt = 0.01
        x, t = thruster.rk4(u, dt)
        wrench = thruster.wrench

        tests = []
        tests.append(wrench.force[0] == 0)
        tests.append(wrench.torque[0] == 0)

        assert np.all(tests)

    def test_thruster_speed_controlled_output(self, build_thruster_speed_controlled):
        thruster = build_thruster_speed_controlled
        omega_ref = math.rpm2radps(3000)
        V_a = 0
        rho = 1.225
        u = np.array([omega_ref, V_a, rho])

        dt = 0.01
        x, t = thruster.rk4(u, dt)
        wrench = thruster.wrench
        y = thruster.y

        tests = []
        tests.append(np.array_equal(wrench.to_array(), y[0:6]))
        tests.append(omega_ref == y[6])

        assert np.all(tests)

    def test_thruster_speed_controlled_normal_operation(
        self, build_thruster_speed_controlled
    ):
        thruster = build_thruster_speed_controlled
        omega_ref = math.rpm2radps(3000)
        V_a = 0
        rho = 1.225
        u = np.array([omega_ref, V_a, rho])

        dt = 0.01
        x, t = thruster.rk4(u, dt)
        wrench = thruster.wrench

        tests = []
        tests.append(wrench.force[0] > 0)
        tests.append(wrench.force[0] < 100)
        tests.append(wrench.torque[0] > 0)
        tests.append(wrench.torque[0] < 10)

        assert np.all(tests)
