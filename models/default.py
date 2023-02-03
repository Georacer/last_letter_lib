#!/usr/bin/env python3
import os
from math import pi

import model_creator as mc
import numpy as np


# Set world and environment settings
# Set model name

aircraft_name = "avy_aera_v1_5"

aircraft = mc.Aircraft(aircraft_name)

# Set airfoils and bodies

# Create body link
# Parameters taken from https://github.com/AvyFly/modelling/blob/master/src/sim_settings.json
aircraft_body = mc.AircraftBody()

aircraft_body.link.set_mass(10)
aircraft_body.link.set_cog([0, 0, 0])
aircraft_body.link.set_inertia_ixx(2.0530)
aircraft_body.link.set_inertia_iyy(1.0855)
aircraft_body.link.set_inertia_izz(2.0063)
aircraft_body.link.set_inertia_ixz(0.1152)

z_adjust = 0.05  # Adjust of the CoG location wrt Body FRD

uav_shape = mc.Mesh(
    os.path.join(os.path.expanduser("~/last_letter_models/meshes"), "fig.stl"),
    [0.001, 0.001, 0.001],
)
visual_body = mc.Visual("body")
visual_body.set_geometry(uav_shape)
# Rotations are body_frd-x -> body_frd-y -> body_frd-z
visual_body.pose = [
    -0.938 / 2 - 0.370,
    1.20,
    0.25 - z_adjust,
    -pi / 2,
    0,
    -pi / 2,
]
aircraft_body.link.add_visual(visual_body)

sphere = mc.Sphere(0.1)
collision_nose = mc.Collision("nose")
collision_nose.pose = [0.31, 0, -z_adjust, 0, 0, 0]
collision_nose.set_geometry(sphere)
aircraft_body.link.add_collision(collision_nose)

collision_nose.set_geometry(sphere)
collision_left_wing = mc.Collision("left_wing")
collision_left_wing.pose = [-0.34, -1.15, -z_adjust, 0, 0, 0]
collision_left_wing.set_geometry(sphere)
aircraft_body.link.add_collision(collision_left_wing)

collision_nose.set_geometry(sphere)
collision_right_wing = mc.Collision("right_wing")
collision_right_wing.pose = [-0.34, 1.15, -z_adjust, 0, 0, 0]
collision_right_wing.set_geometry(sphere)
aircraft_body.link.add_collision(collision_right_wing)

box_gear = mc.Box([0.94, 0.1, 0.16])
collision_left_gear = mc.Collision("left_gear")
collision_left_gear.pose = [0, -0.674 / 2, 0.16 - z_adjust, 0, 0, 0]
collision_left_gear.set_geometry(box_gear)
aircraft_body.link.add_collision(collision_left_gear)

collision_right_gear = mc.Collision("right_gear")
collision_right_gear.pose = [0, 0.674 / 2, 0.16 - z_adjust, 0, 0, 0]
collision_right_gear.set_geometry(box_gear)
aircraft_body.link.add_collision(collision_right_gear)

aircraft.add_airfoil(aircraft_body)

# Create motor links

thruster_x = 1.222 / 2
thruster_y = 0.687 / 2
motor_tilt = np.deg2rad(5)

thruster_0 = mc.Thruster("thruster_0")
thruster_0.pose = [
    thruster_x,
    -thruster_y,
    +z_adjust,
    pi / 2,
    -pi / 2 + motor_tilt,
    -pi / 2,
]
thruster_0.parameters["rotationDir"] = -1.0
thruster_0.parameters["chanMotor"] = 4
aircraft.add_thruster(thruster_0)

thruster_1 = mc.Thruster("thruster_1")
thruster_1.pose = [
    -thruster_x,
    thruster_y,
    +z_adjust,
    pi / 2,
    -pi / 2 + motor_tilt,
    +pi / 2,
]
thruster_1.parameters["rotationDir"] = -1.0
thruster_1.parameters["chanMotor"] = 5
aircraft.add_thruster(thruster_1)

thruster_2 = mc.Thruster("thruster_2")
thruster_2.pose = [
    thruster_x,
    thruster_y,
    +z_adjust,
    pi / 2,
    -pi / 2 + motor_tilt,
    +pi / 2,
]
thruster_2.parameters["rotationDir"] = 1.0
thruster_2.parameters["chanMotor"] = 6
aircraft.add_thruster(thruster_2)

thruster_3 = mc.Thruster("thruster_3")
thruster_3.pose = [
    -thruster_x,
    -thruster_y,
    +z_adjust,
    pi / 2,
    -pi / 2 + motor_tilt,
    -pi / 2,
]
thruster_3.parameters["rotationDir"] = 1.0
thruster_3.parameters["chanMotor"] = 7
aircraft.add_thruster(thruster_3)

thruster_4 = mc.Thruster("thruster_4")
thruster_4.pose = [-0.378, 0, -0.020 + z_adjust, pi, 0.087, 0]
thruster_4.parameters["rotationDir"] = 1.0
thruster_4.parameters["chanMotor"] = 2
# thruster_4.parameters["c_prop"] = 0.07
# thruster_4.parameters["k_motor"] = 35
# thruster_4.parameters["k_t_p"] = 0.0
# thruster_4.parameters["k_omega"] = 390
thruster_4.parameters["motorType"] = 3
thruster_4.parameters["propDiam"] = 0.3556
thruster_4.parameters["engInertia"] = 648e-6
thruster_4.parameters["Kv"] = 8.17
thruster_4.parameters["Rm"] = 0.1
thruster_4.parameters["Rs"] = 0.022
thruster_4.parameters["Cells"] = 6
thruster_4.parameters["I0"] = 0.5
thruster_4.parameters["RadPSLimits"] = [0.01, 1000]
thruster_4.parameters["nCoeffPoly/polyType"] = 0
thruster_4.parameters["nCoeffPoly/polyNo"] = 5
thruster_4.parameters["nCoeffPoly/coeffs"] = [
    0.0042,
    -1.4367,
    44.4844,
    -178.0312,
    277.6847,
    -152.4062,
]
thruster_4.parameters["propPowerPoly/polyType"] = 0
thruster_4.parameters["propPowerPoly/polyNo"] = 3
thruster_4.parameters["propPowerPoly/coeffs"] = [0.04992, 0.003624, 0.05131, -0.1448]
aircraft.add_thruster(thruster_4)

# Set sensors

# Set initialization options
# aircraft.pose = [300, 0, 0.2, 0, 0, pi]
aircraft.pose = [0, 0, 0.2, 0, 0, 0]

aircraft.create_gazebo_model()
aircraft.create_last_letter_model()

world = mc.World("avyworld")
world.add_model(aircraft)
world.attach_camera(aircraft)
world.create_gazebo_world()
