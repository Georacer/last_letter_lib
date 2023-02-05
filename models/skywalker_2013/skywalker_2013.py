#!/usr/bin/env python3
import os
import pathlib
from math import pi
from pathlib import Path

import numpy as np

from last_letter_lib import model_creator as mc


# Set world and environment settings
# Set model name

aircraft_name = "skywalker_2013"
aircraft = mc.Aircraft(aircraft_name)

# Set airfoils and bodies

# Create body link
aircraft_body = mc.AircraftBody()

aircraft_body.link.set_mass(2)
aircraft_body.link.set_cog([0, 0, 0])
aircraft_body.link.set_inertia_ixx(0.8244)
aircraft_body.link.set_inertia_iyy(1.135)
aircraft_body.link.set_inertia_izz(1.759)
aircraft_body.link.set_inertia_ixz(0.1204)

mesh_path = pathlib.Path(__file__).parent.resolve() / Path("skywalker_2013.stl")
uav_shape = mc.Mesh(
    str(mesh_path),
    [0.02621, 0.02621, 0.02621],
)
visual_body = mc.Visual("body")
visual_body.set_geometry(uav_shape)
aircraft_body.link.add_visual(visual_body)

sphere = mc.Sphere(0.1)
collision_nose = mc.Collision("nose")
collision_nose.pose = [0.31, 0, 0, 0, 0, 0]
collision_nose.set_geometry(sphere)
aircraft_body.link.add_collision(collision_nose)

collision_nose.set_geometry(sphere)
collision_left_wing = mc.Collision("left_wing")
collision_left_wing.pose = [-0.34, -1.15, 0, 0, 0, 0]
collision_left_wing.set_geometry(sphere)
aircraft_body.link.add_collision(collision_left_wing)

collision_nose.set_geometry(sphere)
collision_right_wing = mc.Collision("right_wing")
collision_right_wing.pose = [-0.34, 1.15, 0, 0, 0, 0]
collision_right_wing.set_geometry(sphere)
aircraft_body.link.add_collision(collision_right_wing)

gear = mc.Sphere(0.03)
collision_left_gear = mc.Collision("left_gear")
collision_left_gear.pose = [0.13, -0.2, 0.19, 0, 0, 0]
collision_left_gear.set_geometry(gear)
aircraft_body.link.add_collision(collision_left_gear)

collision_right_gear = mc.Collision("right_gear")
collision_right_gear.pose = [0.13, 0.2, 0.19, 0, 0, 0]
collision_right_gear.set_geometry(gear)
aircraft_body.link.add_collision(collision_right_gear)

collision_right_gear = mc.Collision("rear_gear")
collision_right_gear.pose = [-0.87, 0.0, 0.03, 0, 0, 0]
collision_right_gear.set_geometry(gear)
aircraft_body.link.add_collision(collision_right_gear)

aircraft.add_airfoil(aircraft_body)


# Create motor links

thruster_0 = mc.Thruster("thruster_0")
thruster_0.pose = [-0.25, 0.0, 0, 0, 0, 0]
thruster_0.parameters["rotationDir"] = 1.0
thruster_0.parameters["chanMotor"] = 2

aircraft.add_thruster(thruster_0)

# Set sensors

# Set initialization options
aircraft.pose = [0, 0, 0.2, 0, 0, 0]

aircraft.create_gazebo_model()
aircraft.create_last_letter_model()

world = mc.World(aircraft_name)
world.add_model(aircraft)
world.attach_camera(aircraft)
world.create_gazebo_world()
