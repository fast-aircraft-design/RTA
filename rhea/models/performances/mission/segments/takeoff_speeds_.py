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
from rhea.models.performances.mission.segments.intermediate_takeoff_segment import (
    IntermediateTakeoffSegment,
)
from rhea.models.performances.mission.segments.final_takeoff_segment import (
    FinalTakeoffSegment,
)
import math
import copy
from rhea.models.performances.mission.segments.takeoff_flight import TakeOffSegment
import numpy as np


class TakeOffSpeeds(TakeOffSegment):
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
        # self.vef=kwargs.pop("vef")

        self.vr = 1.05 * self.vmca
        self.vlo = 1.01 * self.vmca
        super(TakeOffSegment, self).__init__(**kwargs)

    def compute_from(self, start: FlightPoint) -> pd.DataFrame:
        vef = self.segment_kwargs["vef"]
        self.segment_kwargs["vef"] = 0.01  # speeds must be find with OEI conditon
        self.find_speeds(start)
        print("-------------------------------------------")
        print("-------------------------------------------")
        print("-------------------------------------------")
        print("-------------------------------------------")
        print("-------------------------------------------")
        print("-------------------------------------------")

        print("speeds found")
        self.segment_kwargs[
            "vef"
        ] = vef  # reset engine failure speed value for actual TOFL calculation
        if self.v2 > 0:
            flight_points = super().compute_from(start)
        else:
            flight_points = start
        return flight_points, self.vr, self.vlo, self.v2

    def find_speeds(self, start):

        ###v2 determination
        flight_point_v2 = copy.copy(start)

        flight_point_v2.altitude = start.altitude + 35.0 * foot
        # atm = AtmosphereSI(flight_point_v2.altitude)
        atm = AtmosphereSI(flight_point_v2.altitude, delta_t=flight_point_v2.DISA)
        CL_max = self.segment_kwargs["CL_max"]
        S_ref = self.segment_kwargs["reference_area"]
        factor = 1.131

        def find_v2(factor):
            Cl = CL_max / (factor**2)
            V2 = math.sqrt(2 * flight_point_v2.mass * g / (Cl * S_ref * atm.density))
            v2_min = max(1.1 * self.vmca, V2)
            flight_point_v2.true_airspeed = v2_min
            flight_point_v2.CL = Cl  # modify with actual v2_min
            ##complete flight point

            # flight_point_v2.engine_setting = EngineSetting.TAKEOFF
            flight_point_v2.engine_setting = 8  # RTO

            segment = self.flight_sequence[0]

            segment._complete_speed_values(flight_point_v2)

            segment.propulsion.engine_count = 1.0  # OEI
            segment.propulsion.motor_count = 1.0

            segment._compute_propulsion(flight_point_v2)

            # thrust_OEI = flight_point_v2.thrust/2
            thrust_OEI = flight_point_v2.thrust

            CT = thrust_OEI / (
                0.5 * S_ref * atm.density * flight_point_v2.true_airspeed**2
            )
            flight_point_v2.CT = CT
            DCd_OEI = np.interp(
                flight_point_v2.CT,
                self.segment_kwargs["CT_list"],
                self.segment_kwargs["DCd_OEI"],
            )
            Cd = (
                segment.polar.cd(Cl) + DCd_OEI
            )  # negleting effects (because we don't have alpha at this point)
            climb_rate = (
                thrust_OEI / (flight_point_v2.mass * g) - Cd / Cl
            ) * 100  # =tan(gamma)*100

            segment.propulsion.engine_count = (
                2  # reset AEO condition for next calculations
            )
            segment.propulsion.motor_count = (
                2  # reset AEO condition for next calculations
            )
            # print(flight_point_v2.TPshaft_power, flight_point_v2.thrust ,climb_rate, v2_min)
            # Tot_av_power = segment.propulsion.engine.Elec_nom_power *segment.EM_power_rate+segment.propulsion.engine.RTO_power*segment.TP_power_rate
            Tot_av_power = 0
            return climb_rate, v2_min, Tot_av_power

        climb_rate, v2_min, Tot_av_power = find_v2(factor)
        if climb_rate >= 2.4:
            self.v2_min = v2_min
        else:
            if math.isnan(climb_rate):
                print("CT is nan in takeoff")
            while True:
                factor += 0.01
                climb_rate, v2_min, Tot_av_power = find_v2(factor)
                if climb_rate >= 2.4:
                    self.v2_min = v2_min
                    break
                elif climb_rate < 0:
                    self.v2_min = 0
                    break

        if self.v2_min > 0 and Tot_av_power > 1812000:

            ###vr, vlo determination
            # alpha_max = 13.5 / 180 * math.pi

            # self.segment_kwargs['propulsion'].engine_count = np.array([2.])

            VR = self.vmca * 1.05
            # VR >= v1
            flight_point_vr = copy.copy(start)
            # atm = AtmosphereSI(flight_point_vr.altitude)
            atm = AtmosphereSI(flight_point_vr.altitude, delta_t=flight_point_vr.DISA)
            rho = atm.density
            v_test = 1.02 * VR  # random initial speed used  to determine vlo
            while True:
                # print('VR', VR)
                while True:
                    flight_point_vr.true_airspeed = VR
                    if flight_point_vr.DISA == None:
                        print("deltaisa is none")
                    # self.segment_kwargs['vef']=VR
                    # print(IntermediateTakeoffSegment.propulsion.engine_count)
                    flight_points_Inter = IntermediateTakeoffSegment(
                        target=FlightPoint(equivalent_airspeed=v_test),
                        engine_setting=EngineSetting.TAKEOFF,
                        # name='vlo determination',
                        **self.segment_kwargs,
                    ).compute_from(flight_point_vr)
                    # print('')

                    flight_points_Inter["lift_off"] = (
                        -flight_points_Inter.mass * g
                        - flight_points_Inter.drag
                        * np.sin(flight_points_Inter.alpha.astype(float))
                        + (
                            flight_points_Inter.CL
                            * 0.5
                            * rho
                            * flight_points_Inter.true_airspeed**2
                            * S_ref
                        )
                        * np.cos(flight_points_Inter.alpha.astype(float))
                        + flight_points_Inter.thrust
                        * np.sin(flight_points_Inter.alpha.astype(float))
                    )

                    # find the flght_point which corresponds to vlo
                    try:
                        flight_point_vlo = flight_points_Inter[
                            flight_points_Inter.lift_off > 0
                        ].iloc[0]
                        if flight_point_vlo.CL > CL_max:
                            VR += 0.5
                            # print('CL',flight_point_vlo.CL)
                        else:
                            self.vr = VR
                            self.vlo = flight_point_vlo.true_airspeed
                            # print('VLO',self.vlo)
                            break
                    except:
                        # print('no vlo')
                        v_test += 0.5

                # verify that v2 obtained with vr and vlo is greter than minimum v2
                if self.vlo > self.v2_min:
                    flight_points_Final = FinalTakeoffSegment(
                        target=FlightPoint(altitude=35.0 * foot),
                        engine_setting=EngineSetting.TAKEOFF,
                        # name='v2 determination',
                        **self.segment_kwargs,
                    ).compute_from(FlightPoint(flight_point_vlo))
                    v2 = flight_points_Final.iloc[-1].true_airspeed
                    self.v2 = v2
                    # print('final V2',v2)
                    break
                else:
                    # self.segment_kwargs['name']='v2 determination'

                    flight_points_Final = FinalTakeoffSegment(
                        target=FlightPoint(altitude=35.0 * foot),
                        engine_setting=EngineSetting.TAKEOFF,
                        # name='v2 determination',
                        **self.segment_kwargs,
                    ).compute_from(FlightPoint(flight_point_vlo))
                    v2 = flight_points_Final.iloc[-1].true_airspeed
                    if v2 > self.v2_min:
                        self.v2 = v2
                        # print('final V2',v2)
                        break
                    else:
                        # print('V2',v2)
                        VR += 0.5

        else:
            self.vlo = 0
            self.vr = 0
            self.v2 = 0
            print(
                "Warning: Take-off cannot be performed with available TO power",
                Tot_av_power / 1000,
                "KW",
            )
