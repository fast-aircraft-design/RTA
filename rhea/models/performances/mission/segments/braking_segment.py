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

import logging
from typing import Tuple, List
from scipy.constants import g
from fastoad.utils.physics import AtmosphereSI
import math
from fastoad.base.flight_point import FlightPoint
from fastoad.models.performances.mission.segments.base import ManualThrustSegment
import numpy as np
import scipy.constants as const

_LOGGER = logging.getLogger(__name__)  # Logger for this module
from rhea.models.performances.mission.segments.initial_takeoff_segment import (
    InitialTakeoffSegment,
)
from fastoad.base.dict import AddKeyAttributes

import pandas as pd


class BrakingSegment(InitialTakeoffSegment):
    """
    Computes the braking segment from v1 to v=0. Instead of friction coeffient it used brake_friction_coefficient.
    """

    def complete_flight_point(self, flight_point: FlightPoint):
        """
        Computes data for provided flight point.

        Assumes that it is already defined for time, altitude, mass,
        ground distance and speed (TAS, EAS, or Mach).

        :param flight_point: the flight point that will be completed in-place
        """
        flight_point.engine_setting = self.engine_setting
        self.thrust_rate = np.array([0.09])
        self.TP_power_rate = 0.05

        self._complete_speed_values(flight_point)
        # Mach number is capped by self.maximum_mach
        if flight_point.mach > self.maximum_mach:
            flight_point.mach = self.maximum_mach
            flight_point.true_airspeed = flight_point.equivalent_airspeed = None
            self._complete_speed_values(flight_point)

        atm = AtmosphereSI(flight_point.altitude, delta_t=flight_point.DISA)
        reference_force = (
            0.5 * atm.density * flight_point.true_airspeed**2 * self.reference_area
        )

        if flight_point.true_airspeed > self.vef:
            self.propulsion.engine_count = 1  # 1
            self.propulsion.motor_count = 0  # 1
            # self.TP_power_rate = 0.05

        self._compute_propulsion(flight_point)

        CT = flight_point.thrust / 2 / reference_force  #####verify the division by 2.

        flight_point.CT = CT

        self.compute_aerodynamics(flight_point)

        friction = (
            flight_point.thrust * math.sin(flight_point.alpha)
            + flight_point.mass * g
            - reference_force * flight_point.CL
        ) * self.kf_brake

        (
            flight_point.slope_angle,
            flight_point.acceleration,
        ) = self._get_gamma_and_acceleration(
            flight_point.mass,
            flight_point.drag,
            flight_point.thrust,
            friction,
            flight_point.alpha,
        )
