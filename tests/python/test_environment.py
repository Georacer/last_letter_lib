#!/usr/bin/env python3
"""Tests for the environment library.

usage:
    Have pytest run the tests in this file.
"""

__authors__ = ["George Zogopoulos"]
__credits__ = []
__date__ = "Sat 29 Jan 2022"
__copyright__ = "Copyright 2022, Avy B.V."

import last_letter_lib.environment as env
from last_letter_lib.utils.math import Vector3


###############################################################################
# Create pytest fixtures

G = 9.81

###############################################################################
# Declare the tests themselves ################################################


class TestGravity:
    def test_simple_contructor(self):
        grav = env.GravitySimple(G)
        assert isinstance(grav, env.GravitySimple)

    def test_simple_norm(self):
        grav = env.GravitySimple(G)
        position = Vector3()
        g_norm = grav.g(position).norm
        assert g_norm == G

    def test_simple_vector(self):
        grav = env.GravitySimple(G)
        position = Vector3()
        g_vec = grav.g(position)
        assert g_vec == Vector3(0, 0, G)
