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

from fastoad.models.performances.mission.segments.altitude_change import AltitudeChangeSegment as at1


import logging
from typing import Tuple, List
from scipy.optimize import fsolve
import numpy as np
import pandas as pd
from fastoad.base.dict import AddKeyAttributes
from fastoad.base.flight_point import FlightPoint
from fastoad.utils.physics import AtmosphereSI
from scipy.constants import g, foot
from fastoad.models.performances.mission.exceptions import FastFlightSegmentIncompleteFlightPoint

_LOGGER = logging.getLogger(__name__)  # Logger for this module

@AddKeyAttributes({"EM_power_rate": 0.0,"TP_power_rate":1.0})
class AltitudeChangeSegment(at1):    
    """
    Computes a flight path segment where altitude is modified with constant speed 
    with a given thrust_rate and electric_power_rate
    
    """
    #: Using this value will tell to target the altitude with max lift/drag ratio.
    OPTIMAL_ALTITUDE = -10000.0

    #: Using this value will tell to target the nearest flight level to altitude
    # with max lift/drag ratio.
    OPTIMAL_FLIGHT_LEVEL = -20000.0
    
    def _compute_propulsion(self, flight_point: FlightPoint):
        flight_point.thrust_rate = self.thrust_rate
        flight_point.EM_power_rate = self.EM_power_rate
        flight_point.TP_power_rate = self.TP_power_rate
        flight_point.thrust_is_regulated = False
        
        # if flight_point.name=="acceleration descent":
        #     a=0
        
        self.propulsion.compute_flight_points(flight_point)
        

    def compute_from(self, start: FlightPoint) -> pd.DataFrame:
        start = FlightPoint(start)
        self.complete_flight_point(start)  # needed to ensure all speed values are computed.
        if start.engine_setting==2.:
            ceiling = self._get_max_altitude(start)
            if ceiling < self.target.altitude:
                self.target.altitude=ceiling
                print('Target altitude is greater than ceiling altitude: ceiling altitude will be used instead.',ceiling)
       
        else:
            ceiling=False
        

        # ceiling=False
        if self.target.altitude and self.target.altitude < 0.0:
            # Target altitude will be modified along the process, so we keep track
            # of the original order in target CL, that is not used otherwise.
            self.target.CL = self.target.altitude
            self.interrupt_if_getting_further_from_target = False
            
        if ceiling:
            if start.altitude>ceiling:
                print('Current altitude is greater then ceiling altitude with current power_rates')
                self.interrupt_if_getting_further_from_target = True
                with open('fail.txt', 'w') as f:
                    f.write('fail')
        
        atm = AtmosphereSI(start.altitude, delta_t=start.DISA)
        if self.target.equivalent_airspeed == "constant":
            start.true_airspeed = atm.get_true_airspeed(start.equivalent_airspeed)
        elif self.target.mach == "constant":
            start.true_airspeed = start.mach * atm.speed_of_sound

        flight_points_df = super().compute_from(start)
        if ceiling:
            flight_points_df['ceiling'] = ceiling[0]

        return flight_points_df

    def _get_max_altitude(self,start):
        
        def evaluate_residual_rc(altitude):

            flight_point = FlightPoint(
            engine_setting= self.engine_setting,
            thrust_is_regulated =start.thrust_is_regulated,
            mass=start.mass*0.997,
            equivalent_airspeed =start.equivalent_airspeed ,
            altitude=altitude,
            DISA = start.DISA
            )
            

            self.complete_flight_point(flight_point)
            rc=flight_point.slope_angle*flight_point.true_airspeed
            min_rc = 1.524 #m/s = 300ft/min
            #print(altitude)
            return rc-min_rc
        

        ceiling = fsolve(evaluate_residual_rc, 6000.,xtol=10e-2 )
        if ceiling<0:
            print('Ceiling<0')
            ceiling=np.array([457.])

        return ceiling
    
    def _get_distance_to_target(self, flight_points: List[FlightPoint]) -> bool:
        """
        SAME AS IN FASTOAD ---> ONLY MODIFIED TO TAKE INTO ACCOUNT DISA
        """        
        
        current = flight_points[-1]
        if self.target.CL:
            # Optimal altitude is based on a target Mach number, though target speed
            # may be specified as TAS or EAS. If so, Mach number has to be computed
            # for target altitude and speed.

            # First, as target speed is expected to be set to "constant" for one
            # parameter. Let's get the real value from start point.
            target_speed = FlightPoint(self.target)
            for speed_param in ["true_airspeed", "equivalent_airspeed", "mach"]:
                if isinstance(target_speed.get(speed_param), str):
                    target_speed[speed_param] = flight_points[0][speed_param]

            # Now, let's compute target Mach number
            atm = AtmosphereSI(max(self.target.altitude, current.altitude), delta_t=current.DISA)
            if target_speed.equivalent_airspeed:
                target_speed.true_airspeed = atm.get_true_airspeed(target_speed.equivalent_airspeed)
            if target_speed.true_airspeed:
                target_speed.mach = target_speed.true_airspeed / atm.speed_of_sound

            # Mach number has to be capped by self.maximum_mach
            target_mach = min(target_speed.mach, self.maximum_mach)

            # Now we compute optimal altitude
            optimal_altitude = self._get_optimal_altitude(
                current.mass, target_mach, current.altitude
            )
            if self.target.CL == self.OPTIMAL_ALTITUDE:
                self.target.altitude = optimal_altitude
            else:  # self.target.CL == self.OPTIMAL_FLIGHT_LEVEL:
                flight_level = 1000 * foot
                self.target.altitude = flight_level * np.floor(optimal_altitude / flight_level)

        if self.target.altitude:
            return self.target.altitude - current.altitude
        elif self.target.true_airspeed:
            return self.target.true_airspeed - current.true_airspeed
        elif self.target.equivalent_airspeed:
            return self.target.equivalent_airspeed - current.equivalent_airspeed
        elif self.target.mach:
            return self.target.mach - current.mach    
        
        
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
                
    
