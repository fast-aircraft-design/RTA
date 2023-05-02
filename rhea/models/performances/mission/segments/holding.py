"""Classes for simulating cruise segments."""
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


from fastoad.base.flight_point import FlightPoint
from fastoad.base.dict import AddKeyAttributes
from typing import List
from fastoad.utils.physics import AtmosphereSI
from fastoad.models.performances.mission.exceptions import FastFlightSegmentIncompleteFlightPoint
from fastoad.models.performances.mission.segments.hold import HoldSegment
from scipy.constants import g, foot

@AddKeyAttributes({"EM_power_rate": 0.0,"TP_power_rate":1.0})
class HoldingSegment(HoldSegment):
    """
    Class for computing hold flight segment with given power rates.
    """

    def _compute_propulsion(self, flight_point: FlightPoint):
        flight_point.thrust = flight_point.drag
        flight_point.EM_power_rate = self.EM_power_rate  
        flight_point.TP_power_rate = self.TP_power_rate
        flight_point.thrust_is_regulated = True
        self.propulsion.compute_flight_points(flight_point)
        
    def compute_next_flight_point(
        self, flight_points: List[FlightPoint], time_step: float
    ) -> FlightPoint:
        """
        Computes time, altitude, speed, mass and ground distance of next flight point.

        :param flight_points: previous flight points
        :param time_step: time step for computing next point
        :return: the computed next flight point
        """

        next_point = super().compute_next_flight_point(flight_points,time_step)
        
        next_point.DISA =flight_points[-1].DISA
        return next_point   
    
    def _complete_speed_values(self,flight_point: FlightPoint):
        """
        
        SAME AS IN FASTOAD ---> ONLY MODIFIED TO TAKE INTO ACCOUNT DISA
        
        
        Computes consistent values between TAS, EAS and Mach, assuming one of them is defined.
        """
        atm = AtmosphereSI(flight_point.altitude, delta_t=flight_point.DISA)#########################################

        if flight_point.true_airspeed is None:
            if flight_point.mach:
                flight_point.true_airspeed = flight_point.mach * atm.speed_of_sound
            elif flight_point.equivalent_airspeed:
                flight_point.true_airspeed = atm.get_true_airspeed(flight_point.equivalent_airspeed)
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
            
    def complete_flight_point(self, flight_point: FlightPoint):
        """
        SAME AS IN FASTOAD ---> ONLY MODIFIED TO TAKE INTO ACCOUNT DISA
        
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

        atm = AtmosphereSI(flight_point.altitude, delta_t=flight_point.DISA)#########################################
        reference_force = 0.5 * atm.density * flight_point.true_airspeed ** 2 * self.reference_area

        if self.polar:
            flight_point.CL = flight_point.mass * g / reference_force
            flight_point.CD = self.polar.cd(flight_point.CL)
        else:
            flight_point.CL = flight_point.CD = 0.0
        flight_point.drag = flight_point.CD * reference_force

        self._compute_propulsion(flight_point)
        flight_point.slope_angle, flight_point.acceleration = self._get_gamma_and_acceleration(
            flight_point.mass, flight_point.drag, flight_point.thrust
        )                                    
            