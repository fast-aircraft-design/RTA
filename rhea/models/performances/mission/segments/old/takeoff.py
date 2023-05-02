"""Classes for Taxi sequences."""
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

from typing import Tuple
import pandas as pd
# from fastoad.models.performances.mission.segments.base import FixedDurationSegment
from fastoad.models.performances.mission.segments.base import ManualThrustSegment
from typing import Dict, List, Union
from fastoad.constants import FlightPhase, EngineSetting
from fastoad.models.propulsion import IPropulsion
from scipy.constants import foot, knot,g
from fastoad.base.flight_point import FlightPoint #v0.5.2b
from fastoad.models.performances.mission.polar import Polar
from fastoad.utils.physics import AtmosphereSI
import numpy as np

class TakeOffSegment(ManualThrustSegment):

    """
    Class for computing cruise flight segment at constant altitude.

    Mach is considered constant, equal to Mach at starting point.
    Altitude is constant.
    Target is a specified ground_distance. The target definition indicates
    the ground_distance to be covered during the segment, independently of
    the initial value.
    """

    #: Using this value will tell to target the altitude with max lift/drag ratio.
    OPTIMAL_ALTITUDE = -10000.0

    #: Using this value will tell to target the nearest flight level to altitude
    # with max lift/drag ratio.
    OPTIMAL_FLIGHT_LEVEL = -20000.0
    
    
    #################################################
        V_MCA = self.aircraft.vars_sizing_mission['V_MCA'] * math.sqrt(derate_ratio)    
    
    
        wing_area = self.aircraft.vars_geometry['wing_area']
        span = self.aircraft.vars_geometry['span']
        
        flap_angle = self.aircraft.vars_sizing_mission['flap_angle_to']
        slat_angle = self.aircraft.vars_sizing_mission['slat_angle_to']       
        Cz_0_clean = self.aircraft.vars_aerodynamics['Cz_0_AOA_takeoff']
        k_dcz = self.aircraft.vars_kfactors_aero['K_Cl_to']
        k_dczf = self.aircraft.vars_kfactors_aero['K_Clf_to']
        k_cxgd =self.aircraft.vars_kfactors_aero['K_Cxgd_to']
        Cz_ground = (Cz_0_clean +self.aerodynamics.compute_delta_cz_highlift(flap_angle,slat_angle,0.2)*k_dczf)
        Cl_alpha = self.aircraft.vars_aerodynamics['Cz_alpha_low']     
 
        time_step = 0.05
        weight = TOW
        takeoff_l = 0
        Mach = 0.000001
        V = 0
        time = 0
        alpha = -1.5/ 180. * math.pi
        Cz= Cz_ground+alpha * Cl_alpha
    
    

    
    ###################################################

    def compute_from(self, start: FlightPoint) -> pd.DataFrame:
        start = FlightPoint(start)
        self.complete_flight_point(start)  # needed to ensure all speed values are computed.

        if self.target.altitude and self.target.altitude < 0.0:
            # Target altitude will be modified along the process, so we keep track
            # of the original order in target CL, that is not used otherwise.
            self.target.CL = self.target.altitude
            self.interrupt_if_getting_further_from_target = False

        atm = AtmosphereSI(start.altitude)
        if self.target.equivalent_airspeed == "constant":
            start.true_airspeed = atm.get_true_airspeed(start.equivalent_airspeed)
        elif self.target.mach == "constant":
            start.true_airspeed = start.mach * atm.speed_of_sound

        return super().compute_from(start)

    def _get_distance_to_target(self, flight_points: List[FlightPoint]) -> bool:
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
            atm = AtmosphereSI(max(self.target.altitude, current.altitude))
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

    def _get_gamma_and_acceleration(self, mass, drag, thrust) -> Tuple[float, float]:
        gamma = (thrust - drag) / mass / g
        return gamma, 0.0


