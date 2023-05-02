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
from typing import Dict, List, Union
from fastoad.constants import FlightPhase, EngineSetting
from fastoad.models.propulsion import IPropulsion
from scipy.constants import foot, knot,g
from fastoad.base.flight_point import FlightPoint #v0.5.2b
from fastoad.models.performances.mission.polar import Polar
from fastoad.utils.physics import AtmosphereSI
import numpy as np
from fastoad.models.performances.mission.base import IFlightPart
from rhea.models.performances.mission.base import AbstractManualThrustFlightPhase
from rhea.models.performances.mission.base import AbstractManualThrustFlightPhaseExt

# from fastoad.models.performances.mission.segments.altitude_change import AltitudeChangeSegment
from rhea.models.performances.mission.segments.altitude_change_OEI import AltitudeChangeOEI

from fastoad.models.performances.mission.segments.speed_change import SpeedChangeSegment

from fastoad.models.performances.mission.flight.base import AbstractFlightSequence
import math
import copy





    
class ClimbPhaseOEI(AbstractManualThrustFlightPhaseExt):
    """
    Preset for climb phase.Filippone pag.282

    - Climbs up to 3000ft at constant EAS @MCL setting
    - Accelerates to EAS =climb_speed at constant altitude
    - Climbs up to target altitude at constant EAS
    - Accelerate to target cruise mach
    """



    def __init__(
        self,
        propulsion: IPropulsion,
        reference_area: float,
        polar: Polar,
    ):

        
        self.segment_kwargs = {
            "propulsion": propulsion,
            "reference_area": reference_area,   
            "polar": polar,
            "thrust_rate": 1.,
            "name": 'OEI_ceiling',
            "time_step": 2.,
        }        


    def compute_from(self, start: FlightPoint) -> pd.DataFrame:
        '''#self.find_optimum_climb_speed(start) #quella che massimizza RC
        flight_points= super().compute_from(start) 
        gradient_penalty = 0.011
        flight_points['RC']=flight_points.true_airspeed *(flight_points.thrust-flight_points.drag)/(flight_points.mass*g)
        flight_points['climb_gradient']=flight_points.thrust/(flight_points.mass*g)-flight_points.CD/flight_points.CL
        flight_points['slope_angle_penalty'] = np.arctan((flight_points.climb_gradient.values-gradient_penalty).astype(float))#0.011
        flight_points['RC_penalty']=flight_points.true_airspeed *flight_points.slope_angle_penalty
        flight_points['altitude_penalty'] =  self.segment_kwargs['time_step']* flight_points.RC_penalty.shift(1)
        flight_points.at[0,'altitude_penalty'] = start.altitude
        flight_points['altitude_penalty']= flight_points.altitude_penalty.cumsum()
        # flight_point_ceiling = flight_points[flight_points.RC_penalty<0.254].iloc[0] #0.254m/s=50 ft/min'''
        #changed because before it was using he available thust at the wrong altitude
        flight_points= super().compute_from(start) 
        flight_points['RC']=flight_points.true_airspeed *(flight_points.thrust-flight_points.drag)/(flight_points.mass*g)
        flight_points['climb_gradient']=flight_points.thrust/(flight_points.mass*g)-flight_points.CD/flight_points.CL
        flight_points['climb_gradient_penalty']=flight_points.climb_gradient - 0.011
        flight_points['RC_penalty']=flight_points.true_airspeed *flight_points.slope_angle
        flight_points['altitude_penalty']= flight_points.altitude
        
        return flight_points
    
    @property
    def flight_sequence(self) -> List[Union[IFlightPart, str]]:
        self.segment_kwargs["engine_setting"] = 9

        return [
            AltitudeChangeOEI(
                target=FlightPoint(equivalent_airspeed="constant", altitude=12000.0 * foot),
                **self.segment_kwargs,
            ),
        ]       
    
    
class CruisePhaseOEI(ClimbPhaseOEI):
    """
    Preset for climb phase.Filippone pag.282

    - Climbs up to 3000ft at constant EAS @MCL setting
    - Accelerates to EAS =climb_speed at constant altitude
    - Climbs up to target altitude at constant EAS
    - Accelerate to target cruise mach
    """





    def compute_from(self, start: FlightPoint) -> pd.DataFrame:

        #changed because before it was using he available thust at the wrong altitude
        flight_points= super().compute_from(start) 
        flight_points['RC']=flight_points.true_airspeed *(flight_points.thrust-flight_points.drag)/(flight_points.mass*g)
        flight_points['climb_gradient']=flight_points.thrust/(flight_points.mass*g)-flight_points.CD/flight_points.CL
        flight_points['climb_gradient_penalty']=flight_points.climb_gradient - 0.011
        flight_points['RC_penalty']=flight_points.true_airspeed *flight_points.slope_angle
        flight_points['altitude_penalty']= flight_points.altitude
        
        return flight_points
    
    @property
    def flight_sequence(self) -> List[Union[IFlightPart, str]]:
        self.segment_kwargs["engine_setting"] = 9

        return [
            AltitudeChangeOEI(
                target=FlightPoint(equivalent_airspeed="constant", altitude=12000.0 * foot),
                **self.segment_kwargs,
            ),
        ]       
        

        

