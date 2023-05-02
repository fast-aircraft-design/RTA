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
from rhea.models.performances.mission.base import AbstractManualThrustFlightPhaseExt
from rhea.models.performances.mission.segments.initial_takeoff_segment import InitialTakeoffSegment
from rhea.models.performances.mission.segments.intermediate_takeoff_segment import IntermediateTakeoffSegment
from rhea.models.performances.mission.segments.final_takeoff_segment import FinalTakeoffSegment
import math
import copy

# class TakeOffSegment(AbstractManualThrustFlightPhase):
class TakeOffSegment(AbstractManualThrustFlightPhaseExt):
    """
    Class for computing cruise flight segment at constant altitude.

    Mach is considered constant, equal to Mach at starting point.
    Altitude is constant.
    Target is a specified ground_distance. The target definition indicates
    the ground_distance to be covered during the segment, independently of
    the initial value.
    """


    def __init__(self, **kwargs):
        self.vmca = kwargs.pop("vmca")
        # self.vr = kwargs.pop("vr")
        # self.vlo = kwargs.pop("vlo")
        #self.vef=kwargs.pop("vef")
        
        self.vr = kwargs['vr']
        self.vlo = kwargs['vlo']
        
        super().__init__(**kwargs)    

        
    @property
    def flight_sequence(self) -> List[Union[IFlightPart, str]]:
        return [
            InitialTakeoffSegment(
                target=FlightPoint(equivalent_airspeed = self.vr),
                engine_setting=EngineSetting.TAKEOFF,
                # name ='1st take off segment',
                **self.segment_kwargs,
         
            ),
            IntermediateTakeoffSegment(
                target=FlightPoint(equivalent_airspeed = self.vlo),
                engine_setting=EngineSetting.TAKEOFF,
                # name='2nd take off segment',
                **self.segment_kwargs,
                
            ),
            FinalTakeoffSegment(
                target=FlightPoint(altitude=35. * foot),
                engine_setting=EngineSetting.TAKEOFF,
                # name='3rd take off segment',
                **self.segment_kwargs,
                
            ),            
            
        ]
 

            
        

