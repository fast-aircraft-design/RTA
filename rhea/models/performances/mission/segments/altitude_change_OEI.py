"""Classes for acceleration/deceleration segments."""
#  This file is part of FAST : A framework for rapid Overall Aircraft Design
#  Copyright (C) 2020  ONERA & ISAE-SUPAERO
#  FAST is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.
import numpy as np
import logging
from typing import Tuple, List
import pandas as pd
from fastoad.base.flight_point import FlightPoint
from fastoad.models.performances.mission.segments.base import ManualThrustSegment
from scipy.constants import g
from fastoad.utils.physics import AtmosphereSI
import math
_LOGGER = logging.getLogger(__name__)  # Logger for this module
from fastoad.base.dict import AddKeyAttributes
import scipy.constants as const
from fastoad.models.performances.mission.segments.altitude_change import AltitudeChangeSegment

class AltitudeChangeOEI(AltitudeChangeSegment):
    """
    Computes a flight path segment where altitude is modified with acceleration.
    """

    def complete_flight_point(self, flight_point: FlightPoint):
        """
        Computes data for provided flight point.

        Assumes that it is already defined for time, altitude, mass,
        ground distance and speed (TAS, EAS, or Mach).

        :param flight_point: the flight point that will be completed in-place
        """
        
        flight_point.engine_setting = self.engine_setting

        self._complete_speed_values(flight_point)
        # Mach number is capped by self.maximum_mach
        if flight_point.mach > self.maximum_mach:
            flight_point.mach = self.maximum_mach
            flight_point.true_airspeed = flight_point.equivalent_airspeed = None
            self._complete_speed_values(flight_point)

        atm = AtmosphereSI(flight_point.altitude)
        reference_force = 0.5 * atm.density * flight_point.true_airspeed ** 2 * self.reference_area

        if self.polar:
            flight_point.CL = flight_point.mass * g / reference_force
            flight_point.CD = self.polar.cd(flight_point.CL)
        else:
            flight_point.CL = flight_point.CD = 0.0
        flight_point.drag = flight_point.CD * reference_force

        self._compute_propulsion(flight_point)
        
        lift = flight_point.CL * reference_force
    
        flight_point.slope_angle, flight_point.acceleration = self._get_gamma_and_acceleration(
            flight_point.mass, flight_point.drag, lift, flight_point.thrust)
        
        
        
        
    def _get_gamma_and_acceleration(self, mass, drag, lift, thrust) -> Tuple[float, float]:
        climb_gradient = thrust / mass / g - drag / lift
        climb_gradient_penalty = climb_gradient - 0.011
        gama = np.arctan(climb_gradient)
        gama_penalty = np.arctan(climb_gradient_penalty)
        
        gamma = (thrust - drag) / mass / g 
        return gama_penalty, 0.0

    def compute_next_flight_point(
        self, flight_points: List[FlightPoint], time_step: float
    ) -> FlightPoint:
        """
        Computes time, altitude, speed, mass and ground distance of next flight point.

        :param flight_points: previous flight points
        :param time_step: time step for computing next point
        :return: the computed next flight point
        """
        start = flight_points[0]
        previous = flight_points[-1]
        next_point = FlightPoint()

        next_point.mass = previous.mass #- self.propulsion.get_consumed_mass(previous, time_step)
        next_point.time = previous.time + time_step
        next_point.ground_distance = (
            previous.ground_distance
            + previous.true_airspeed * time_step * np.cos(previous.slope_angle)
        )
        self._compute_next_altitude(next_point, previous)

        if self.target.true_airspeed == "constant":
            next_point.true_airspeed = previous.true_airspeed
        elif self.target.equivalent_airspeed == "constant":
            next_point.equivalent_airspeed = start.equivalent_airspeed
        elif self.target.mach == "constant":
            next_point.mach = start.mach
        else:
            next_point.true_airspeed = previous.true_airspeed + time_step * previous.acceleration

        # The naming is not done in complete_flight_point for not naming the start point
        next_point.name = self.name
        return next_point

        