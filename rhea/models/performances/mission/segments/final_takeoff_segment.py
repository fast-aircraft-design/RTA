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
from rhea.models.performances.mission.segments.initial_takeoff_segment import (
    InitialTakeoffSegment,
)


class FinalTakeoffSegment(InitialTakeoffSegment):
    """
    Computes a flight path segment where altitude is modified with acceleration.


    """

    def compute_from(self, start: FlightPoint) -> pd.DataFrame:
        start = FlightPoint(start)
        self.complete_flight_point(
            start
        )  # needed to ensure all speed values are computed.

        if self.target.altitude and self.target.altitude < 0.0:
            # Target altitude will be modified along the process, so we keep track
            # of the original order in target CL, that is not used otherwise.
            self.target.CL = self.target.altitude
            self.interrupt_if_getting_further_from_target = False

        atm = AtmosphereSI(
            start.altitude, delta_t=start.DISA
        )  #########################################
        if self.target.equivalent_airspeed == "constant":
            start.true_airspeed = atm.get_true_airspeed(start.equivalent_airspeed)
        elif self.target.mach == "constant":
            start.true_airspeed = start.mach * atm.speed_of_sound

        return super().compute_from(start)

    def complete_flight_point(self, flight_point: FlightPoint):
        """
        Computes data for provided flight point.

        Assumes that it is already defined for time, altitude, mass,
        ground distance and speed (TAS, EAS, or Mach).

        :param flight_point: the flight point that will be completed in-place
        """
        flight_point.engine_setting = self.engine_setting

        atm = AtmosphereSI(
            flight_point.altitude, delta_t=flight_point.DISA
        )  #########################################
        reference_force = (
            0.5 * atm.density * flight_point.true_airspeed**2 * self.reference_area
        )

        self._complete_speed_values(flight_point)
        if flight_point.true_airspeed > self.vef:
            self.propulsion.motor_count = 1
            self.propulsion.engine_count = 1
            flight_point.engine_setting = 8  # RTO
        else:
            self.propulsion.motor_count = 2
            self.propulsion.engine_count = 2
            flight_point.engine_setting = 1  # TO
        self._compute_propulsion(flight_point)

        CT = flight_point.thrust / 2 / reference_force
        flight_point.CT = CT

        super().compute_aerodynamics(flight_point)

        lift = flight_point.CL * reference_force

        (
            flight_point.slope_angle,
            flight_point.acceleration,
        ) = self._get_gamma_and_acceleration(
            flight_point.mass,
            flight_point.drag,
            lift,
            flight_point.thrust,
            flight_point.alpha,
            flight_point.true_airspeed,
        )

    def compute_next_flight_point(
        self, flight_points: List[FlightPoint], time_step: float
    ) -> FlightPoint:
        """
        Computes time, altitude, speed, mass and ground distance of next flight point.

        :param flight_points: previous flight points
        :param time_step: time step for computing next point
        :return: the computed next flight point
        """

        next_point = super().compute_next_flight_point(flight_points, time_step)

        # alpha to be changed in order to reduce acceleration
        if flight_points[-1].acceleration > 0 and flight_points[-1].CL < self.CL_max:
            next_point.alpha = flight_points[-1].alpha + 0.1 * math.pi / 180
        else:
            next_point.alpha = flight_points[-1].alpha - 0.1 * math.pi / 180

        return next_point

    def _get_distance_to_target(self, flight_points: List[FlightPoint]) -> bool:
        self.target.slope_angle = flight_points[
            -1
        ].slope_angle  # just a way to store previous slope_angle

        if self.target.altitude:
            return self.target.altitude - flight_points[-1].altitude

    def _get_gamma_and_acceleration(
        self, mass, drag, lift, thrust, alpha, V
    ) -> Tuple[float, float]:
        if self.target.slope_angle:
            pass
        else:
            self.target.slope_angle = 0
        # print('previous gamma',self.target.slope_angle)
        d_gamma = (
            thrust * math.sin(alpha)
            + lift
            - mass * g * math.cos(self.target.slope_angle)
        ) / (mass * V)
        # print('d_gamma',d_gamma)
        if d_gamma <= 0:
            d_gamma = 0

        acceleration = (
            thrust * math.cos(alpha)
            - drag
            - mass * g * math.sin(self.target.slope_angle)
        ) / mass

        gamma = self.time_step * d_gamma + self.target.slope_angle

        # print('new gamma',gamma)
        # print('-----------')

        return gamma, acceleration
