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

from typing import List

import pandas as pd
from fastoad.base.flight_point import FlightPoint
from fastoad.models.performances.mission.polar import Polar
from fastoad.models.propulsion import IPropulsion
from rhea.models.performances.mission.segments.speed_change import SpeedChangeSegment
import copy
from scipy.constants import g
from fastoad.utils.physics import AtmosphereSI
from scipy.optimize import fsolve
from fastoad.base.dict import DynamicAttributeDict, AddKeyAttributes

class SpeedChangeClimb(SpeedChangeSegment):
    """
    Class for computing cruise flight segment and operational ceiling.

    Mach is considered constant, equal to Mach at starting point. Altitude is set **at every
    point** to get the optimum CL according to current mass.
    """
        
    def compute_from(self, start: FlightPoint) -> pd.DataFrame:
        flight_points_df = super().compute_from(start)       
        ceiling = self._get_max_altitude(flight_points_df.values[-1])
        flight_points_df['ceiling'] = ceiling[0]
        return flight_points_df


    def _get_max_altitude(self,start):
        
        def evaluate_residual_rc(altitude):

            flight_point = FlightPoint(
            engine_setting= self.engine_setting,
            thrust_is_regulated =start.thrust_is_regulated,
            mass=start.mass*0.99,
            equivalent_airspeed =start.equivalent_airspeed ,
            altitude=altitude,
            )
            
            self._complete_speed_values(flight_point)
            self._compute_propulsion(flight_point) 
            atm = AtmosphereSI(flight_point.altitude)
            P_dyn= (0.5* atm.density * flight_point.true_airspeed**2)
            Cl= flight_point.mass*g / (P_dyn* self.reference_area)
            Cd = self.polar.cd(Cl) 
            rc = 2*flight_point.thrust/(flight_point.mass*g) - Cd/Cl
            print(flight_point.thrust)
            min_rc = 1.524 #m/s = 300ft/min
            return rc-min_rc
        
        ceiling,info,ier,msg = fsolve(evaluate_residual_rc, 6000 , full_output=True)
        return ceiling
        

        
