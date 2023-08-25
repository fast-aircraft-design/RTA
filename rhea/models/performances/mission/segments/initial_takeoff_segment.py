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
from fastoad.models.performances.mission.exceptions import (
    FastFlightSegmentIncompleteFlightPoint,
)

import numpy as np
import scipy.constants as const

_LOGGER = logging.getLogger(__name__)  # Logger for this module

from fastoad.base.dict import AddKeyAttributes
import pandas as pd

SEGMENT_KEYWORD_ARGUMENTS = {
    "CL_alpha": 0,
    "CL0": 0,
    "CL_max": 0,
    "kf": 0,
    "H_list": 0,
    "alpha_list": 0,
    "CT_list": 0,
    "DCl0_CT": 0,
    "DCl_alpha_CT": 0,
    "DCd_OEI": 0,
    "DCd_gd": 0,
    "DCl_gd": 0,
    "K_H": 0,
    "DCd_lg": 0,
    "DCl_lg": 0,
    "EM_power_rate": 0.0,
    "TP_power_rate": 1.0,
    "vef": 1000,
    "kf_brake": 0.6,
    "vr": 0,
    "vlo": 0,
    "v1": 0,
}


@AddKeyAttributes(SEGMENT_KEYWORD_ARGUMENTS)
class InitialTakeoffSegment(ManualThrustSegment):
    """
    Computes the initial take off segment where speed is modified with no change in altitude.
    The difference with the SpeedChangeSegment is in the calculation of Cz/Cd and the acceleration.

    The target must define a speed value among true_airspeed, equivalent_airspeed
    and mach.
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

        atm = AtmosphereSI(flight_point.altitude, delta_t=flight_point.DISA)
        reference_force = (
            0.5 * atm.density * flight_point.true_airspeed**2 * self.reference_area
        )
        # print(flight_point.TP_power_rate)
        if flight_point.equivalent_airspeed > self.vef:
            self.propulsion.motor_count = 1
            self.propulsion.engine_count = 1
            flight_point.engine_setting = 8  # RTO

        else:
            self.propulsion.motor_count = 2
            self.propulsion.engine_count = 2
            flight_point.engine_setting = 1  # TO

        self._compute_propulsion(flight_point)

        CT = flight_point.thrust / 2 / reference_force  #####verify the division by 2.

        flight_point.CT = CT

        self.compute_aerodynamics(flight_point)

        friction = (
            flight_point.thrust * math.sin(flight_point.alpha)
            + flight_point.mass * g
            - reference_force * flight_point.CL
        ) * self.kf

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
        # print('Lift off', -flight_point.mass - flight_point.drag*math.sin(flight_point.alpha) + flight_point.CL*reference_force*math.cos(flight_point.alpha) + flight_point.thrust*math.sin(flight_point.alpha) )

    def compute_aerodynamics(self, flight_point: FlightPoint):

        atm = AtmosphereSI(flight_point.altitude, delta_t=flight_point.DISA)
        reference_force = (
            0.5 * atm.density * flight_point.true_airspeed**2 * self.reference_area
        )
        Cl_no_effects = self.CL0 + flight_point.alpha * self.CL_alpha
        if np.array(self.DCl_gd).any():
            DCl_gd = np.interp(
                flight_point.alpha / const.degree, self.alpha_list, self.DCl_gd
            ) * np.interp(flight_point.altitude, self.H_list, self.K_H)
        else:
            DCl_gd = 0
        DCl_lg = np.interp(
            flight_point.alpha / const.degree, self.alpha_list, self.DCl_lg
        )

        CT = flight_point.CT
        if np.array(self.DCl0_CT).any():
            DCl_CT = (
                np.interp(CT, self.CT_list, self.DCl0_CT)
                + np.interp(CT, self.CT_list, self.DCl_alpha_CT) * flight_point.alpha
            )
        else:
            DCl_CT = 0

        flight_point.CL = Cl_no_effects + DCl_gd + DCl_lg + DCl_CT

        Cd_no_effects = self.polar.cd(Cl_no_effects)
        if np.array(self.DCd_gd).any():
            DCd_gd = np.interp(
                flight_point.alpha / const.degree, self.alpha_list, self.DCd_gd
            ) * np.interp(flight_point.altitude, self.H_list, self.K_H)
        else:
            DCd_gd = 0

        DCd_lg = np.interp(
            flight_point.alpha / const.degree, self.alpha_list, self.DCd_lg
        )

        DCd_OEI = np.interp(CT, self.CT_list, self.DCd_OEI)

        Cd_tot = Cd_no_effects + DCd_gd + DCd_lg
        if flight_point.true_airspeed > self.vef:
            Cd_tot = Cd_tot + 0.0046  # + DCd_OEI

        flight_point.CD = Cd_tot

        flight_point.drag = flight_point.CD * reference_force

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

        next_point.alpha = flight_points[-1].alpha
        next_point.TP_power_rate = flight_points[-1].TP_power_rate
        next_point.EM_power_rate = flight_points[-1].EM_power_rate
        next_point.DISA = flight_points[-1].DISA
        return next_point

    def _get_distance_to_target(self, flight_points: List[FlightPoint]) -> bool:
        if self.target.true_airspeed:
            return self.target.true_airspeed - flight_points[-1].true_airspeed
        elif self.target.equivalent_airspeed:
            return (
                self.target.equivalent_airspeed - flight_points[-1].equivalent_airspeed
            )
        elif self.target.mach:
            return self.target.mach - flight_points[-1].mach

    def _get_gamma_and_acceleration(
        self, mass, drag, thrust, friction, alpha
    ) -> Tuple[float, float]:

        acceleration = (thrust * math.cos(alpha) - friction - drag) / mass

        return 0.0, acceleration

    def _compute_propulsion(self, flight_point: FlightPoint):
        flight_point.thrust_rate = self.thrust_rate
        # flight_point.power_rate_electric = self.power_rate_electric
        # flight_point.power_rate_turbine = self.power_rate_turbine
        flight_point.EM_power_rate = self.EM_power_rate
        flight_point.TP_power_rate = self.TP_power_rate
        flight_point.thrust_is_regulated = False
        self.propulsion.compute_flight_points(flight_point)

    def _complete_speed_values(self, flight_point: FlightPoint):
        """
        SAME AS IN FASTOAD ---> ONLY MODIFIED TO TKE INTO ACCOUNT DISA

        Computes consistent values between TAS, EAS and Mach, assuming one of them is defined.
        """
        atm = AtmosphereSI(
            flight_point.altitude, delta_t=flight_point.DISA
        )  #########################################

        if flight_point.true_airspeed is None:
            if flight_point.mach:
                flight_point.true_airspeed = flight_point.mach * atm.speed_of_sound
            elif flight_point.equivalent_airspeed:
                flight_point.true_airspeed = atm.get_true_airspeed(
                    flight_point.equivalent_airspeed
                )
            else:
                raise FastFlightSegmentIncompleteFlightPoint(
                    "Flight point should be defined for true_airspeed, "
                    "equivalent_airspeed, or mach."
                )
        if flight_point.mach is None:
            flight_point.mach = flight_point.true_airspeed / atm.speed_of_sound

        if flight_point.equivalent_airspeed is None:
            flight_point.equivalent_airspeed = atm.get_equivalent_airspeed(
                flight_point.true_airspeed
            )
