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
from scipy.constants import foot, knot, g
from fastoad.base.flight_point import FlightPoint  # v0.5.2b
from fastoad.models.performances.mission.polar import Polar
from fastoad.utils.physics import AtmosphereSI
import numpy as np
from fastoad.models.performances.mission.base import IFlightPart
from rhea.models.performances.mission.base import AbstractManualThrustFlightPhaseExt
from rhea.models.performances.mission.segments.initial_takeoff_segment import (
    InitialTakeoffSegment,
)
from rhea.models.performances.mission.segments.braking_segment import BrakingSegment

from rhea.models.performances.mission.segments.intermediate_asd_segment import (
    IntermediateAsdSegment,
)
import math
import copy


class ASDSegment(AbstractManualThrustFlightPhaseExt):
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
        super().__init__(**kwargs)

    def compute_from(self, start: FlightPoint) -> pd.DataFrame:
        # self.find_speeds(start)
        # self.segment_kwargs['name']='take off'
        flight_points = super().compute_from(start)
        return flight_points

    @property
    def flight_sequence(self) -> List[Union[IFlightPart, str]]:
        return [
            InitialTakeoffSegment(
                target=FlightPoint(equivalent_airspeed=self.segment_kwargs["v1"]),
                engine_setting=EngineSetting.TAKEOFF,
                # name ='1st  segment',
                **self.segment_kwargs,
            ),
            BrakingSegment(
                target=FlightPoint(equivalent_airspeed=np.array([0.001])),
                engine_setting=EngineSetting.TAKEOFF,
                **self.segment_kwargs,
            ),
        ]
