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
from scipy.constants import g,foot
from fastoad.utils.physics import AtmosphereSI
import math
_LOGGER = logging.getLogger(__name__)  # Logger for this module
from fastoad.base.dict import AddKeyAttributes
import scipy.constants as const
from fastoad.models.performances.mission.segments.base import RegulatedThrustSegment
from rhea.models.performances.mission.segments.altitude_change import AltitudeChangeSegment

class FixedSlopeDescent(AltitudeChangeSegment):
    """
    Class for computing descent flight segment at constant velocity and slope.


    """

    def _get_gamma_and_acceleration(self, mass, drag, thrust) -> Tuple[float, float]:
        gamma = (thrust - drag) / mass / g
        return gamma, 0.0
        #return -3./const.degree, 0.0

    '''def _compute_propulsion(self, flight_point: FlightPoint):
        flight_point.thrust = flight_point.drag + flight_point.mass*g*(-3.*const.degree)
        flight_point.EM_power_rate = self.EM_power_rate
        flight_point.thrust_is_regulated = True
        self.propulsion.compute_flight_points(flight_point)'''
        
    def _compute_propulsion(self, flight_point: FlightPoint):
        flight_point.thrust = flight_point.drag + flight_point.mass*g*(-3.*const.degree)
        flight_point.EM_power_rate = self.EM_power_rate
        flight_point.TP_power_rate = self.TP_power_rate
        flight_point.thrust_is_regulated = True
        self.propulsion.compute_flight_points(flight_point)        
        
