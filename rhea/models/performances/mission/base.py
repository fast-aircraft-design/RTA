"""Base classes for mission computation."""
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

from abc import ABC, abstractmethod
from typing import List, Union

import pandas as pd
from fastoad.base.flight_point import FlightPoint
from fastoad.models.performances.mission.polar import Polar
from fastoad.models.propulsion import IPropulsion
from fastoad.models.performances.mission.base import AbstractManualThrustFlightPhase
import numpy as np


class AbstractManualThrustFlightPhaseExt(AbstractManualThrustFlightPhase, ABC):
    """
    Base class for climb and descent phases.
    """

    def __init__(
        self,
        *,
        propulsion: IPropulsion,
        reference_area: float,
        polar: Polar,
        thrust_rate: float = 1.0,
        name="",
        time_step=None,
        v1: float = np.nan,
        vef: float = 1000,
        vr: float = np.nan,
        vlo: float = np.nan,
        CL_alpha: float = np.nan,
        CL0: float = np.nan,
        CL_max: float = np.nan,
        kf: float = np.nan,
        H_list: float = np.nan,
        alpha_list: float = np.nan,
        CT_list: float = np.nan,
        DCl0_CT: float = np.nan,
        DCl_alpha_CT: float = np.nan,
        DCd_OEI: float = np.nan,
        DCd_gd: float = np.nan,
        DCl_gd: float = np.nan,
        K_H: float = np.nan,
        DCd_lg: float = np.nan,
        DCl_lg: float = np.nan,
        EM_power_rate: float = 0.0,
        TP_power_rate: float = 1.0,
        kf_brake: float = 0.6,
    ):
        """
        Initialization is done only with keyword arguments.

        :param propulsion:
        :param reference_area:
        :param polar:
        :param thrust_rate:
        :param time_step: if provided, this time step will be applied for all segments.
        """
        self.segment_kwargs = {
            "propulsion": propulsion,
            "reference_area": reference_area,
            "polar": polar,
            "thrust_rate": thrust_rate,
            "name": name,
            "time_step": time_step,
            "v1": v1,
            "vef": vef,
            "vr": vr,
            "vlo": vlo,
            "CL_alpha": CL_alpha,
            "CL0": CL0,
            "CL_max": CL_max,
            "kf": kf,
            "H_list": H_list,
            "alpha_list": alpha_list,
            "CT_list": CT_list,
            "DCl0_CT": DCl0_CT,
            "DCl_alpha_CT": DCl_alpha_CT,
            "DCd_OEI": DCd_OEI,
            "DCd_gd": DCd_gd,
            "DCl_gd": DCl_gd,
            "K_H": K_H,
            "DCd_lg": DCd_lg,
            "DCl_lg": DCl_lg,
            "EM_power_rate": EM_power_rate,
            "TP_power_rate": TP_power_rate,
            "kf_brake": kf_brake,
        }
