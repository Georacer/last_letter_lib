#!/usr/bin/env python3
"""Tests for the simulation library.

usage:
    Have pytest run the tests in this file.
"""

__authors__ = ["George Zogopoulos"]
__credits__ = []
__date__ = "Fri 21 Jan 2022"
__copyright__ = "Copyright 2022, Avy B.V."

import numpy as np
import pytest

from last_letter_lib import aerodynamics as aero
from last_letter_lib import propulsion
from last_letter_lib import simulation as sim
from last_letter_lib import systems
from last_letter_lib.utils.math import EulerAngles
from last_letter_lib.utils.math import Vector3
from last_letter_lib.utils.math import build_vector3_from_array
from last_letter_lib.utils.uav import Inputs
from last_letter_lib.utils.uav import UavState


###############################################################################
# Declare fixtures


@pytest.fixture
def build_desc_aircraft():
    return sim.AircraftParameters(
        name="test_aircraft",
        aerodynamics=[
            aero.AerodynamicParameters(
                name="test_airfoil",
                pose=systems.PoseParameters(position=(0, 0, 0), orientation=(0, 0, 0)),
                inertial=systems.InertialParameters(
                    mass=2, inertia=[0.8244, 0, -0.12, 0, 1.135, 0, -0.12, 0, 1.759]
                ),
                s=0.45,
                b=1.88,
                c=0.24,
                c_L_0=0.4,
                c_L_alpha=[6.5, 0],
                c_D_0=0.09,
                c_D_alpha=[0.14, 0],
                c_Y_beta=[0.98, 0],
                c_l_0=0,
                c_l_pn=[-1.0, 0],
                c_l_beta=[-0.12, 0],
                c_l_deltaa=[0.25, 0],
                c_m_0=0.01,
                c_m_alpha=[-1.3, 0],
                c_m_qn=[-30, 0],
                c_m_deltae=[1.0, 0],
                c_n_0=0,
                c_n_beta=[0.25, 0],
                c_n_rn=[-1.0, 0],
                c_n_deltar=[0.1, 0],
                delta_a_max=0.3491,
                delta_e_max=0.3491,
                delta_r_max=0.3491,
            ),
        ],
        thrusters=[
            propulsion.ThrusterSimpleParameters(
                name="thruster_simple",
                pose=systems.PoseParameters(
                    position=(-0.5, 0, 0), orientation=(0, 0, 0)
                ),
                inertial=systems.InertialParameters(mass=0.1),
                rotation_dir=propulsion.DirectionEnum.CW,
                thruster_type=propulsion.ThrusterTypeEnum.SIMPLE,
                usage=propulsion.ThrusterUseEnum.AIRPLANE,
                thrust_min=0,
                thrust_max=10,
                torque_min=0,
                torque_max=0.01,
            )
        ],
        other_components=[
            systems.ComponentParameters(
                name="ballast",
                pose=systems.PoseParameters(
                    position=(0.5, 0, 0), orientation=(0, 0, 0)
                ),
                inertial=systems.InertialParameters(mass=0.1),
            )
        ],
    )


@pytest.fixture
def build_desc_aircraft_2():
    return sim.AircraftParameters(
        name="test_aircraft_2",
        aerodynamics=[
            aero.AerodynamicParameters(
                name="test_airfoil", pose=systems.PoseParameters()
            )
        ],
        thrusters=[
            propulsion.ThrusterSimpleParameters(
                name="thruster_simple",
                pose=systems.PoseParameters(),
                usage=propulsion.ThrusterUseEnum.MULTICOPTER,
            ),
            propulsion.ThrusterBeardParameters(
                name="thruster_beard",
                pose=systems.PoseParameters(),
                usage=propulsion.ThrusterUseEnum.AIRPLANE,
            ),
        ],
        other_components=[
            systems.ComponentParameters(
                name="ballast",
                pose=systems.PoseParameters(
                    position=(0.5, 0, 0), orientation=(0, 0, 0)
                ),
                inertial=systems.InertialParameters(mass=0.1),
            )
        ],
    )


###############################################################################
# Declare the tests themselves ################################################


class TestAircraftParameters:
    """
    Test the Aircraft class.
    build_aircraft uses the following components:
    - build_aerodynamic, mass 1kg, at pos (0.0,0.0,0.0)
    - build_thruster_simple, mass 1kg, at pos (0.0,0.0,0.0)
    - build_thruster_beard, mass 1kg, at pos (1.0,1.0,1.0)
    - build_battery, mass 0kg, at pos (0.0,0.0,0.0)
    """

    def test_constructor(self, build_desc_aircraft):
        test_ac = build_desc_aircraft
        assert isinstance(test_ac, sim.AircraftParameters)

    def test_components(self, build_desc_aircraft):
        """
        - Test that correct number of components is returned
        - Test that the types of the components are correct
        """
        test_ac = build_desc_aircraft
        test_ac_components = test_ac.components
        component_types = [type(component) for component in test_ac_components]
        assert len(test_ac_components) == 3
        assert component_types == [
            aero.AerodynamicParameters,
            propulsion.ThrusterSimpleParameters,
            systems.ComponentParameters,
        ]

    def test_get_num_thrusters(self, build_desc_aircraft_2):
        """
        Test that correct number of thrusters is returned.
        - no query: all thrusters are counted.
        - query: only thrusters of which the use is specified by the query.
        """
        test_ac = build_desc_aircraft_2
        num_thrusters_all = test_ac.get_num_thrusters(thruster_use=None)
        # is "thruster_use=None" supposed to be explicitly stated? Using no argument fails
        num_thrusters_multicopter = test_ac.get_num_thrusters(
            thruster_use=propulsion.ThrusterUseEnum.MULTICOPTER
        )
        num_thrusters_airplane = test_ac.get_num_thrusters(
            thruster_use=propulsion.ThrusterUseEnum.AIRPLANE
        )
        tests = [
            num_thrusters_all == 2,
            num_thrusters_multicopter == 1,
            num_thrusters_airplane == 1,
        ]
        assert np.all(tests)


class TestAircraft:
    def test_construtor(self, build_desc_aircraft):
        ac = sim.Aircraft(build_desc_aircraft)
        assert isinstance(ac, sim.Aircraft)

    def test_airfoils(self, build_desc_aircraft):
        ac = sim.Aircraft(build_desc_aircraft)
        assert len(ac.airfoils) == 1

    def test_thrusters(self, build_desc_aircraft):
        ac = sim.Aircraft(build_desc_aircraft)
        assert len(ac.thrusters) == 1

    def test_components(self, build_desc_aircraft):
        ac = sim.Aircraft(build_desc_aircraft)
        assert len(ac.components) == 3

    def test_mass(self, build_desc_aircraft):
        ac = sim.Aircraft(build_desc_aircraft)
        assert ac.mass == pytest.approx(2.2)

    def test_freefall(self, build_desc_aircraft):
        ac = sim.Aircraft(build_desc_aircraft)
        u = Inputs(0, 0, 0, [0])
        dt = 0.01
        t, t_end = 0, 1
        while t < t_end:
            ac.step(u, dt)
            t += dt

        assert ac.state.position.z > 0

    def test_glide(self, build_desc_aircraft):
        ac = sim.Aircraft(build_desc_aircraft)
        vel = 15
        state = ac.state
        state.attitude = EulerAngles(pitch=-10, in_degrees=True).to_quaternion()
        state.velocity_linear = Vector3(vel, 0, 0)
        ac.reset(state)
        u = Inputs(0, 0, 0, [0])
        dt = 0.1
        t, t_end = 0, 5
        while t < t_end:
            ac.step(u, dt)
            t += dt

        tests = []
        # Position is reasonable
        tests.append(ac.state.position.x > vel * t_end / 2)
        tests.append(abs(ac.state.position.y) < 1)
        tests.append(ac.state.position.z > 0)
        # Angles are reasonable
        angles = ac.state.attitude.to_euler()
        tests.append(angles.pitch < 0)
        tests.append(abs(angles.roll) < np.deg2rad(30))
        tests.append(abs(angles.yaw) < np.deg2rad(30))
        # Velocity is reasonable
        tests.append(ac.state.velocity_linear.x > vel / 2)
        tests.append(abs(ac.state.velocity_linear.y) < 2)
        tests.append(ac.state.velocity_linear.z > 0)
        tests.append(ac.state.velocity_linear.z < vel / 2)
        # Angular velocity is close to 0
        tests.append(abs(ac.state.velocity_angular.x) < 0.1)
        tests.append(abs(ac.state.velocity_angular.y) < 0.1)
        tests.append(abs(ac.state.velocity_angular.z) < 0.1)
        assert np.all(tests)

    def test_roll(self, build_desc_aircraft):
        ac = sim.Aircraft(build_desc_aircraft)
        vel = 15
        state = ac.state
        state.attitude = EulerAngles(pitch=-10, in_degrees=True).to_quaternion()
        state.velocity_linear = Vector3(vel, 0, 0)
        ac.reset(state)
        u = Inputs(0.5, 0, 0, [0])
        dt = 0.1
        t, t_end = 0, 1
        while t < t_end:
            ac.step(u, dt)
            t += dt

        tests = []
        # Position is reasonable
        tests.append(ac.state.position.x > vel * t_end / 2)
        tests.append(ac.state.position.y > 0)
        tests.append(ac.state.position.z > 0)
        # Angles are reasonable
        angles = ac.state.attitude.to_euler()
        tests.append(angles.pitch < 0)
        tests.append(0 < angles.roll < np.deg2rad(60))
        tests.append(0 < angles.yaw < np.deg2rad(30))
        # Velocity is reasonable
        tests.append(ac.state.velocity_linear.x > vel / 2)
        tests.append(abs(ac.state.velocity_linear.y) < 2)
        tests.append(abs(ac.state.velocity_linear.z) < 1)
        # Angular velocity is reasonable
        tests.append(ac.state.velocity_angular.x > 0)
        tests.append(abs(ac.state.velocity_angular.y) < 0.1)
        tests.append(abs(ac.state.velocity_angular.z) < 0.5)
        assert np.all(tests)

    def test_pitch(self, build_desc_aircraft):
        ac = sim.Aircraft(build_desc_aircraft)
        vel = 15
        state = ac.state
        state.attitude = EulerAngles(pitch=-10, in_degrees=True).to_quaternion()
        state.velocity_linear = Vector3(vel, 0, 0)
        ac.reset(state)
        u = Inputs(0, 0.5, 0, [0])
        dt = 0.1
        t, t_end = 0, 1
        while t < t_end:
            ac.step(u, dt)
            t += dt

        tests = []
        # Position is reasonable
        tests.append(ac.state.position.x > vel * t_end / 2)
        tests.append(abs(ac.state.position.y) < 1)
        tests.append(abs(ac.state.position.z) < vel * t_end / 2)
        # Angles are reasonable
        angles = ac.state.attitude.to_euler()
        tests.append(0 < angles.pitch < np.deg2rad(90))
        tests.append(abs(angles.roll) < np.deg2rad(10))
        tests.append(abs(angles.yaw) < np.deg2rad(10))
        # Velocity is reasonable
        tests.append(ac.state.velocity_linear.x > vel / 2)
        tests.append(abs(ac.state.velocity_linear.y) < 2)
        tests.append(0 < ac.state.velocity_linear.z < vel / 2)
        # Angular velocity is reasonable
        tests.append(abs(ac.state.velocity_angular.x) < 0.1)
        tests.append(ac.state.velocity_angular.y > 0.1)
        tests.append(abs(ac.state.velocity_angular.z) < 0.1)
        assert np.all(tests)

    def test_thruster_1(self, build_desc_aircraft):
        ac = sim.Aircraft(build_desc_aircraft)
        u = Inputs(0, 0, 0, [1])
        dt = 0.1
        ac.step(u, dt)

        assert ac.acceleration_linear.x > 0
        assert ac.acceleration_linear.y == pytest.approx(0)
        assert ac.acceleration_linear.z > 0
        assert ac.acceleration_angular.x > 0
        assert ac.acceleration_angular.y == pytest.approx(0)
        assert ac.acceleration_angular.z > 0

    def test_thruster_2(self, build_desc_aircraft):
        ac = sim.Aircraft(build_desc_aircraft)
        vel = 15
        state = ac.state
        state.attitude = EulerAngles(pitch=-10, in_degrees=True).to_quaternion()
        state.velocity_linear = Vector3(vel, 0, 0)
        ac.reset(state)
        u = Inputs(0, 0, 0, [0])
        dt = 0.1
        # Let aircraft glide
        t, t_end = 0, 3
        while t < t_end:
            ac.step(u, dt)
            t += dt
        state_1 = UavState.from_uavstate(ac.state)
        # Then apply full throttle
        u = Inputs(0, 0, 0, [1])
        t_end += 3
        while t < t_end:
            ac.step(u, dt)
            t += dt
        state_2 = UavState.from_uavstate(ac.state)

        tests = []
        tests.append(state_1.velocity_linear.x < state_2.velocity_linear.x)
        assert np.all(tests)
