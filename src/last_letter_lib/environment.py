#!/usr/bin/env python3
"""Library for environmental models

This library is meant to host utilities related to environmental models.

usage:
    Import the module for use in other scripts.
"""

__authors__ = ["George Zogopoulos"]
__credits__ = []
__date__ = "Thu 20 2022"
__copyright__ = "Copyright 2022, Avy B.V."

from abc import ABC
from abc import abstractmethod
from dataclasses import dataclass

from last_letter_lib.utils.math import Vector3


@dataclass
class EnvironmentData:
    wind: Vector3 = Vector3()  # Wind vector in the inertial frame, in m/s
    rho: float = 1.225  # Air density, in kg/m^3
    pressure: float = 1013  # Atmospheric pressure, in hPa
    temperature: float = 273  # Air temperature, in degrees Kelvin
    gravity: float = 9.81  # Gravity acceleration, in m/s^2


class EnvironmentModel(ABC):
    data: EnvironmentData

    @property
    @abstractmethod
    def update(self, position: Vector3):
        """
        Update the environment model given the UAV position.

        INPUTS:
            position: UAV inertial position in the NED frame.
        """


class EnvironmentSimple(EnvironmentModel):
    """
    A constant environment model, independent of the aircraft state.
    """

    def __init__(self):
        self.data = EnvironmentData()

    def update(self, position):
        pass


class GravityModel(ABC):
    @abstractmethod
    def _calc_acceleration(self, position: Vector3) -> Vector3:
        """
        Implement the gravity model here.

        OUTPUTS:
            The gravity acceleration vector, in the inertial NED frame.
        """
        pass

    def g(self, position: Vector3) -> Vector3:
        """
        Return the gravity acceleration in the inertial NED frame.

        INPUTS:
            position: A Vector3 object with components
                x: geoid latitude
                y: geoid longitude
                z: geoid altitude
        """
        return self._calc_acceleration(position)


class GravitySimple(GravityModel):
    def __init__(self, g):
        self.__g = Vector3(0, 0, g)

    def _calc_acceleration(self, position):
        return self.__g
