#!/usr/bin/env python
from math import pi

import model_creator as mc


# Set world and environment settings
# Set model name

aircraft = mc.Aircraft("skywalker")

# Set airfoils and bodies

# Create body link
# Parameters taken from https://github.com/AvyFly/modelling/blob/master/src/sim_settings.json
aircraft_body = mc.AircraftBody()

aircraft_body.link.set_mass(2)
aircraft_body.link.set_cog([0, 0, 0])
aircraft_body.link.set_inertia_ixx(0.8244)
aircraft_body.link.set_inertia_iyy(1.135)
aircraft_body.link.set_inertia_izz(1.759)
aircraft_body.link.set_inertia_ixz(0.1204)

uav_shape = mc.Mesh(
    "/home/george/.gazebo/models/skywalker/skywalker_2013.stl",
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

# Edit airfoil parameters for body frame

aircraft.airfoil_params["body_frd"][
    "aerodynamicsType"
] = 3  # linear-in-the-parameters drag model
aircraft.airfoil_params["body_frd"]["s"] = 0.45
aircraft.airfoil_params["body_frd"]["b"] = 1.88
aircraft.airfoil_params["body_frd"]["c"] = 0.24
aircraft.airfoil_params["body_frd"]["c_L_0"] = 0.4
aircraft.airfoil_params["body_frd"]["c_L_deltae"] = 0.0
aircraft.airfoil_params["body_frd"]["c_L_alpha"] = 6.5
aircraft.airfoil_params["body_frd"]["c_L_qn"] = 0
aircraft.airfoil_params["body_frd"]["mcoeff"] = 50
aircraft.airfoil_params["body_frd"]["oswald"] = 0.9
aircraft.airfoil_params["body_frd"]["alpha_stall"] = 0.4712
aircraft.airfoil_params["body_frd"]["c_D_qn"] = 0
aircraft.airfoil_params["body_frd"]["c_D_deltae"] = 0.0
aircraft.airfoil_params["body_frd"]["c_D_0"] = 0.09
aircraft.airfoil_params["body_frd"]["c_D_alpha"] = 0.14
aircraft.airfoil_params["body_frd"]["c_Y_0"] = 0
aircraft.airfoil_params["body_frd"]["c_Y_beta"] = -0.98
aircraft.airfoil_params["body_frd"]["c_Y_pn"] = 0
aircraft.airfoil_params["body_frd"]["c_Y_rn"] = 0
aircraft.airfoil_params["body_frd"]["c_Y_deltaa"] = 0
aircraft.airfoil_params["body_frd"][
    "c_Y_deltar"
] = -0.2  # opposite sign than c_n_deltar
aircraft.airfoil_params["body_frd"]["c_l_0"] = 0
aircraft.airfoil_params["body_frd"]["c_l_pn"] = -1.0
aircraft.airfoil_params["body_frd"]["c_l_beta"] = -0.12
aircraft.airfoil_params["body_frd"]["c_l_rn"] = 0.14
aircraft.airfoil_params["body_frd"]["c_l_deltaa"] = 0.25
aircraft.airfoil_params["body_frd"]["c_l_deltar"] = -0.037
aircraft.airfoil_params["body_frd"]["c_m_0"] = 0.01
aircraft.airfoil_params["body_frd"]["c_m_alpha"] = -1.3
aircraft.airfoil_params["body_frd"]["c_m_qn"] = -20
aircraft.airfoil_params["body_frd"]["c_m_deltae"] = 1.0
aircraft.airfoil_params["body_frd"]["c_n_0"] = 0
aircraft.airfoil_params["body_frd"]["c_n_beta"] = 0.25
aircraft.airfoil_params["body_frd"]["c_n_pn"] = 0.022
aircraft.airfoil_params["body_frd"]["c_n_rn"] = -1
aircraft.airfoil_params["body_frd"]["c_n_deltaa"] = 0.00
aircraft.airfoil_params["body_frd"]["c_n_deltar"] = 0.1  # opposite sign than c_y_deltar
aircraft.airfoil_params["body_frd"]["deltaa_max"] = 0.3491
aircraft.airfoil_params["body_frd"]["deltae_max"] = 0.3491
aircraft.airfoil_params["body_frd"]["deltar_max"] = 0.3491
aircraft.airfoil_params["body_frd"]["deltaa_max_nominal"] = 0.3491
aircraft.airfoil_params["body_frd"]["deltae_max_nominal"] = 0.3491
aircraft.airfoil_params["body_frd"]["deltar_max_nominal"] = 0.3491
aircraft.airfoil_params["body_frd"]["chanAileron"] = 0
aircraft.airfoil_params["body_frd"]["chanElevator"] = 1
aircraft.airfoil_params["body_frd"]["chanRudder"] = 3


# Create motor links

thruster_0 = mc.Thruster("thruster_0")
thruster_0.pose = [-0.25, 0.0, 0, 0, 0, 0]
thruster_0.parameters["rotationDir"] = 1.0
thruster_0.parameters["motorType"] = 1
thruster_0.parameters["s_prop"] = 1
thruster_0.parameters["c_prop"] = 0.33
thruster_0.parameters["k_motor"] = 20
thruster_0.parameters["k_t_p"] = 0
thruster_0.parameters["k_omega"] = 0
thruster_0.parameters["chanMotor"] = 2

aircraft.add_thruster(thruster_0)

# Set sensors

# Set initialization options
aircraft.pose = [0, 0, 0.2, 0, 0, 0]

aircraft.create_gazebo_model()
aircraft.create_last_letter_model()

world = mc.World("avyworld")
world.add_model(aircraft)
world.attach_camera(aircraft)
world.create_gazebo_world()
