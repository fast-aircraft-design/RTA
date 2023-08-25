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
from fastoad.base.flight_point import FlightPoint
from fastoad.models.performances.mission.segments.base import ManualThrustSegment
from rhea.models.performances.mission.segments.initial_takeoff_segment import (
    InitialTakeoffSegment,
)
import numpy as np

_LOGGER = logging.getLogger(__name__)  # Logger for this module


class IntermediateTakeoffSegment(InitialTakeoffSegment):
    """
    Computes the intermediate take off segment where speed is modified with change rotation of angle of attack.

    The target must define a speed value among true_airspeed, equivalent_airspeed
    and mach.
    """

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

        if flight_points[-1].alpha >= (0.0 / 180.0 * np.pi):
            next_point.alpha = (
                flight_points[-1].alpha + (4.5 / 180.0 * np.pi) * time_step
            )  # one engine:4.2; two engine:6.5
        else:
            next_point.alpha = (
                flight_points[-1].alpha + (1.0 / 180.0 * np.pi) * time_step
            )
        # if flight_points[-1].alpha >=  (0./ 180. * np.pi):

        return next_point
