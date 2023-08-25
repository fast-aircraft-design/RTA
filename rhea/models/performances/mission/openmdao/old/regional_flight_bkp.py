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

import numpy as np
import openmdao.api as om
import pandas as pd
from fastoad.constants import FlightPhase
from fastoad import BundleLoader
from rhea.models.aerodynamics.constants import (
    POLAR_POINT_COUNT,
    CT_POINT_COUNT,
    ALPHA_POINT_COUNT,
    H_POINT_COUNT,
)
from fastoad.models.performances.breguet import Breguet
from fastoad.models.performances.mission.segments.hold import HoldSegment
from fastoad.models.performances.mission.segments.taxi import TaxiSegment
from rhea.models.performances.mission.segments.takeoff_flight import TakeOffSegment
from rhea.models.performances.mission.segments.OEI_ceiling import ClimbPhaseOEI

# from fastoad.models.propulsion import EngineSet
from models.propulsion.fuel_engine.turboprop_engine.base import TPEngineSet
from models.propulsion.fuel_engine.hybrid_engine.base import HybridEngineSet

from scipy.constants import foot, nautical_mile, knot, hour, g

from fastoad.models.performances.mission.flight.base import RangedFlight
from ..flight.standard_regional_flight import StandardFlight, DiversionFlight

# from fastoad.models.performances.mission.flight_point import FlightPoint #v0.50a
from fastoad.base.flight_point import FlightPoint
from fastoad.models.performances.mission.polar import Polar
from fastoad.base.dict import AddKeyAttributes
from fastoad.utils.physics import Atmosphere
from fastoad.constants import EngineSetting

AddKeyAttributes(["alpha", "max_mach"])(FlightPoint)


class SizingFlight_RHEA(om.ExplicitComponent):
    """
    Simulates a complete flight sizing mission with diversion.
    """

    def __init__(self, **kwargs):
        """
        Computes thrust, SFC and thrust rate by direct call to engine model.

        Options:
          - propulsion_id: (mandatory) the identifier of the propulsion wrapper.
          - out_file: if provided, a csv file will be written at provided path with all computed
                      flight points. If path is relative, it will be resolved from working
                      directory
        """
        super().__init__(**kwargs)
        self.flight_points = None
        self._engine_wrapper = None
        # self._engine_wrapper2 = None

    def initialize(self):
        self.options.declare("propulsion_id", default="", types=str)
        self.options.declare("out_file", default="", types=str)

    def setup(self):
        self._engine_wrapper = BundleLoader().instantiate_component(
            self.options["propulsion_id"]
        )
        self._engine_wrapper.setup(self)

        # self._engine_wrapper2 = BundleLoader().instantiate_component('rhea.wrapper.propulsion.TP_engine_L1')
        # self._engine_wrapper2.setup(self)
        # Inputs -----------------------------------------------------------------------------------
        self.add_input("data:TLAR:cruise_mach", np.nan)
        self.add_input("data:TLAR:range", np.nan, units="m")

        # if 'PH' in self.options["propulsion_id"]:
        #     self.add_input("data:geometry:propulsion:motor:count",  np.nan)

        self.add_input("data:geometry:propulsion:engine:count", np.nan)

        self.add_input("data:geometry:wing:area", np.nan, units="m**2")

        self.add_input(
            "data:aerodynamics:aircraft:cruise:CL", np.nan, shape=POLAR_POINT_COUNT
        )
        self.add_input(
            "data:aerodynamics:aircraft:cruise:CD", np.nan, shape=POLAR_POINT_COUNT
        )
        self.add_input(
            "data:aerodynamics:aircraft:low_speed:CL", np.nan, shape=POLAR_POINT_COUNT
        )
        self.add_input(
            "data:aerodynamics:aircraft:low_speed:CD", np.nan, shape=POLAR_POINT_COUNT
        )
        self.add_input(
            "data:aerodynamics:aircraft:takeoff:CL", np.nan, shape=POLAR_POINT_COUNT
        )
        self.add_input(
            "data:aerodynamics:aircraft:takeoff:CD", np.nan, shape=POLAR_POINT_COUNT
        )
        self.add_input(
            "data:aerodynamics:aircraft:takeoff:CL_alpha", np.nan, units="1/rad"
        )
        self.add_input("data:aerodynamics:aircraft:takeoff:CL0", np.nan)
        self.add_input("data:aerodynamics:aircraft:takeoff:CL_max", np.nan)
        self.add_input(
            "data:aerodynamics:aircraft:low_speed:H",
            np.nan,
            units="m",
            shape=H_POINT_COUNT,
        )
        self.add_input(
            "data:aerodynamics:aircraft:low_speed:alpha",
            np.nan,
            units="deg",
            shape=ALPHA_POINT_COUNT,
        )
        self.add_input(
            "data:aerodynamics:aircraft:low_speed:CT", np.nan, shape=CT_POINT_COUNT
        )
        # self.add_input("data:aerodynamics:aircraft:low_speed:DCD_ext", np.nan)
        # self.add_input("data:aerodynamics:aircraft:low_speed:DCD_feather", np.nan)

        self.add_input(
            "data:aerodynamics:aircraft:takeoff:CT_effect:DCL0",
            0,
            copy_shape="data:aerodynamics:aircraft:low_speed:CT",
        )
        self.add_input(
            "data:aerodynamics:aircraft:takeoff:CT_effect:DCL_alpha",
            0,
            units="1/rad",
            copy_shape="data:aerodynamics:aircraft:low_speed:CT",
        )
        self.add_input(
            "data:aerodynamics:aircraft:takeoff:OEI_effect:DCD",
            np.nan,
            copy_shape="data:aerodynamics:aircraft:low_speed:CT",
        )
        self.add_input(
            "data:aerodynamics:aircraft:takeoff:ground_effect:DCD",
            0,
            copy_shape="data:aerodynamics:aircraft:low_speed:alpha",
        )
        self.add_input(
            "data:aerodynamics:aircraft:takeoff:ground_effect:DCL",
            0,
            copy_shape="data:aerodynamics:aircraft:low_speed:alpha",
        )
        self.add_input(
            "data:aerodynamics:aircraft:takeoff:ground_effect:K_H",
            0,
            copy_shape="data:aerodynamics:aircraft:low_speed:H",
        )
        self.add_input(
            "data:aerodynamics:aircraft:takeoff:lg_effect:DCD",
            np.nan,
            copy_shape="data:aerodynamics:aircraft:low_speed:alpha",
        )
        self.add_input(
            "data:aerodynamics:aircraft:takeoff:lg_effect:DCL",
            np.nan,
            copy_shape="data:aerodynamics:aircraft:low_speed:alpha",
        )

        self.add_input("data:weight:aircraft:MTOW", np.nan, units="kg")

        self.add_input("data:mission:sizing:taxi_out:fuel", np.nan, units="kg")

        self.add_input("data:mission:sizing:takeoff:V_MCA", np.nan, units="m/s")

        self.add_input(
            "data:mission:sizing:takeoff:Friction_coefficient_no_brake", np.nan
        )

        self.add_input("data:mission:sizing:takeoff:altitude", np.nan, units="m")
        self.add_input("data:mission:sizing:takeoff:fuel", np.nan, units="kg")
        # self.add_output("data:mission:sizing:takeoff:fuel", np.nan, units="kg")
        self.add_input("data:mission:sizing:takeoff:thrust_rate", np.nan)

        self.add_input("data:mission:sizing:climb:thrust_rate", np.nan)
        self.add_input("data:mission:sizing:climb:speed", np.nan, units="m/s")
        self.add_input(
            "data:mission:sizing:main_route:cruise:altitude", np.nan, units="ft"
        )
        self.add_input("data:mission:sizing:descent:thrust_rate", np.nan)
        self.add_input("data:mission:sizing:descent:speed", np.nan, units="m/s")

        self.add_input("data:mission:sizing:diversion:distance", np.nan, units="m")
        self.add_input("data:mission:sizing:diversion:altitude", np.nan, units="ft")
        self.add_input("data:mission:sizing:diversion:mach", np.nan)
        self.add_input("data:mission:sizing:holding:duration", np.nan, units="s")

        self.add_input("data:mission:sizing:landing:fuel", np.nan, units="kg")
        self.add_input("data:mission:sizing:taxi_in:fuel", np.nan, units="kg")

        self.add_input("data:mission:sizing:taxi_in:duration", np.nan, units="s")
        self.add_input("data:mission:sizing:taxi_in:speed", np.nan, units="m/s")
        self.add_input("data:mission:sizing:taxi_in:thrust_rate", np.nan)

        # Inputs needed in order to use the sizing output file as input .xml for the DOC mission evaluation-----------------------------------------------------------------------------------
        self.add_input("data:mission:DOC:cruise:mach", np.nan)
        self.add_input("data:mission:DOC:range", np.nan, units="m")
        self.add_input("data:weight:aircraft:OWE", np.nan, units="kg")
        self.add_input("data:mission:DOC:payload", np.nan, units="kg")

        self.add_input("data:mission:DOC:TOW", np.nan, units="kg")

        self.add_input("data:mission:DOC:taxi_out:fuel", np.nan, units="kg")

        # self.add_input("data:mission:DOC:takeoff:V2", np.nan, units="m/s")

        self.add_input("data:mission:DOC:takeoff:altitude", np.nan, units="m")
        self.add_input("data:mission:DOC:takeoff:fuel", np.nan, units="kg")

        self.add_input("data:mission:DOC:climb:thrust_rate", np.nan)
        self.add_input("data:mission:DOC:climb:speed", np.nan, units="m/s")
        self.add_input(
            "data:mission:DOC:main_route:cruise:altitude", np.nan, units="ft"
        )
        self.add_input("data:mission:DOC:descent:thrust_rate", np.nan)
        self.add_input("data:mission:DOC:descent:speed", np.nan, units="m/s")

        self.add_input("data:mission:DOC:diversion:distance", np.nan, units="m")
        self.add_input("data:mission:DOC:diversion:altitude", np.nan, units="ft")
        self.add_input("data:mission:DOC:diversion:mach", np.nan)
        self.add_input("data:mission:DOC:holding:duration", np.nan, units="s")

        self.add_input("data:mission:DOC:landing:fuel", np.nan, units="kg")
        self.add_input("data:mission:DOC:taxi_in:fuel", np.nan, units="kg")

        self.add_input("data:mission:DOC:taxi_in:duration", np.nan, units="s")
        self.add_input("data:mission:DOC:taxi_in:speed", np.nan, units="m/s")
        self.add_input("data:mission:DOC:taxi_in:thrust_rate", np.nan)

        # Outputs ----------------------------------------------------------------------------------
        self.add_output("data:mission:sizing:takeoff:distance", units="m", val=0.0)
        self.add_output("data:mission:sizing:takeoff:V_rotate", units="m/s", val=0.0)
        self.add_output("data:mission:sizing:takeoff:V_liftoff", units="m/s", val=0.0)
        self.add_output("data:mission:sizing:takeoff:V_2", units="m/s", val=0.0)

        self.add_output("data:mission:sizing:OEI_ceiling:altitude", units="m")

        self.add_output("data:mission:sizing:cruise:max_mach")

        self.add_output("data:mission:sizing:initial_climb:fuel", units="kg")
        self.add_output("data:mission:sizing:initial_climb:H2", units="kg", val=0)

        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:H2_flow_rate",
            units="kg/s",
            val=0,
        )

        self.add_output("data:mission:sizing:main_route:climb:fuel", units="kg")
        self.add_output("data:mission:sizing:main_route:cruise:fuel", units="kg")
        self.add_output("data:mission:sizing:main_route:descent:fuel", units="kg")
        self.add_output("data:mission:sizing:main_route:climb:H2", units="kg", val=0.0)
        self.add_output("data:mission:sizing:main_route:cruise:H2", units="kg", val=0.0)
        self.add_output(
            "data:mission:sizing:main_route:descent:H2", units="kg", val=0.0
        )

        self.add_output("data:mission:sizing:initial_climb:distance", units="m")
        self.add_output("data:mission:sizing:main_route:climb:distance", units="m")
        self.add_output("data:mission:sizing:main_route:cruise:distance", units="m")
        self.add_output("data:mission:sizing:main_route:descent:distance", units="m")

        self.add_output("data:mission:sizing:initial_climb:duration", units="s")
        self.add_output("data:mission:sizing:main_route:climb:duration", units="s")

        self.add_output("data:mission:sizing:main_route:climb:TTC", units="s")

        self.add_output("data:mission:sizing:main_route:cruise:duration", units="s")
        self.add_output("data:mission:sizing:main_route:descent:duration", units="s")

        self.add_output("data:mission:sizing:diversion:climb:fuel", units="kg")
        self.add_output("data:mission:sizing:diversion:cruise:fuel", units="kg")
        self.add_output("data:mission:sizing:diversion:descent:fuel", units="kg")
        self.add_output("data:mission:sizing:diversion:climb:H2", units="kg", val=0.0)
        self.add_output("data:mission:sizing:diversion:cruise:H2", units="kg", val=0.0)
        self.add_output("data:mission:sizing:diversion:descent:H2", units="kg", val=0.0)

        self.add_output("data:mission:sizing:diversion:climb:distance", units="m")
        self.add_output("data:mission:sizing:diversion:cruise:distance", units="m")
        self.add_output("data:mission:sizing:diversion:descent:distance", units="m")

        self.add_output("data:mission:sizing:diversion:climb:duration", units="s")
        self.add_output("data:mission:sizing:diversion:cruise:duration", units="s")
        self.add_output("data:mission:sizing:diversion:descent:duration", units="s")

        self.add_output("data:mission:sizing:holding:fuel", units="kg")
        self.add_output("data:mission:sizing:holding:H2", units="kg", val=0.0)
        # self.add_output("data:mission:sizing:taxi_in:fuel", units="kg")

        self.add_output("data:mission:sizing:ZFW", units="kg")
        self.add_output("data:mission:sizing:fuel", units="kg")
        self.add_output("data:mission:sizing:H2", units="kg", val=0.0)
        self.add_output("data:mission:sizing:BAT_energy", units="J", val=0.0)
        self.add_output("data:mission:sizing:trip_fuel", units="kg")
        self.add_output("data:mission:sizing:trip_H2", units="kg", val=0.0)
        self.add_output("data:mission:sizing:block_fuel", units="kg")
        self.declare_partials(["*"], ["*"])

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        loop_input = inputs["data:weight:aircraft:MTOW"]
        mission_type = "sizing"
        try:
            self.compute_mission(inputs, outputs, loop_input, mission_type)
        except IndexError:
            self.compute_breguet(inputs, outputs, loop_input, mission_type)

    def compute_breguet(self, inputs, outputs, loop_input, mission_type):

        if "PH" or "FH" in self.options["propulsion_id"]:
            # propulsion_model = HybridEngineSet(
            #     self._engine_wrapper.get_model(inputs), inputs["data:geometry:propulsion:engine:count"],inputs["data:geometry:propulsion:motor:count"]
            # )
            propulsion_model = HybridEngineSet(
                self._engine_wrapper.get_model(inputs),
                inputs["data:geometry:propulsion:engine:count"],
            )
        else:
            propulsion_model = TPEngineSet(
                self._engine_wrapper.get_model(inputs),
                inputs["data:geometry:propulsion:engine:count"],
            )

        high_speed_polar = Polar(
            inputs["data:aerodynamics:aircraft:cruise:CL"],
            inputs["data:aerodynamics:aircraft:cruise:CD"],
        )

        if mission_type == "sizing":
            flight_distance = inputs["data:TLAR:range"]
            cruise_mach = inputs["data:TLAR:cruise_mach"]
            TOW_input = loop_input
        elif mission_type == "DOC":
            flight_distance = inputs["data:mission:DOC:range"]
            cruise_mach = inputs["data:mission:DOC:cruise:mach"]
            TOW_input = loop_input
        # elif mission_type=='payload_range':
        #     flight_distance= loop_inputs[0]
        #     cruise_mach= inputs["data:TLAR:cruise_mach"]
        #     diversion_mach = inputs["data:mission:sizing:diversion:mach"]
        #     TOW_input=loop_inputs[1]
        #     mission_type='DOC'

        breguet = Breguet(
            propulsion_model,
            max(
                10.0,
                high_speed_polar.optimal_cl
                / high_speed_polar.cd(high_speed_polar.optimal_cl),
            ),
            cruise_mach,
            10000.0,
        )

        breguet.compute(
            TOW_input,
            flight_distance,
        )

        outputs["data:mission:" + mission_type + ":ZFW"] = breguet.zfw
        outputs["data:mission:" + mission_type + ":fuel"] = breguet.mission_fuel

    def compute_mission(self, inputs, outputs, loop_inputs, mission_type):

        if mission_type == "sizing":
            flight_distance = inputs["data:TLAR:range"]
            cruise_mach = inputs["data:TLAR:cruise_mach"]
            diversion_mach = inputs["data:mission:sizing:diversion:mach"]
            TOW_input = loop_inputs
        elif mission_type == "DOC":
            flight_distance = inputs["data:mission:DOC:range"]
            cruise_mach = inputs["data:mission:DOC:cruise:mach"]
            diversion_mach = inputs["data:mission:DOC:diversion:mach"]
            TOW_input = loop_inputs
        elif mission_type == "payload_range":
            flight_distance = loop_inputs[0]
            cruise_mach = inputs["data:TLAR:cruise_mach"]
            diversion_mach = inputs["data:mission:sizing:diversion:mach"]
            TOW_input = loop_inputs[1]
            mission_type = "DOC"

        # if 'PHFC' in self.options["propulsion_id"]:
        if "PH" or "FH" in self.options["propulsion_id"]:
            # propulsion_model = HybridEngineSet(
            #     self._engine_wrapper.get_model(inputs), inputs["data:geometry:propulsion:engine:count"],inputs["data:geometry:propulsion:motor:count"]
            # )
            # propulsion_model_OEI = HybridEngineSet(
            #     self._engine_wrapper.get_model(inputs), inputs["data:geometry:propulsion:engine:count"]-1,inputs["data:geometry:propulsion:motor:count"]
            # )
            propulsion_model = HybridEngineSet(
                self._engine_wrapper.get_model(inputs),
                inputs["data:geometry:propulsion:engine:count"],
            )
            propulsion_model_OEI = HybridEngineSet(
                self._engine_wrapper.get_model(inputs),
                inputs["data:geometry:propulsion:engine:count"] - 1,
            )
        else:
            propulsion_model = TPEngineSet(
                self._engine_wrapper.get_model(inputs),
                inputs["data:geometry:propulsion:engine:count"],
            )
            propulsion_model_OEI = TPEngineSet(
                self._engine_wrapper.get_model(inputs),
                inputs["data:geometry:propulsion:engine:count"] - 1,
            )

        reference_area = inputs["data:geometry:wing:area"]
        # cruise_mach = inputs["data:mission:"+mission_type+":cruise:mach"]
        # flight_distance = inputs["data:mission:"+mission_type+":range"]
        cruise_altitude = inputs[
            "data:mission:" + mission_type + ":main_route:cruise:altitude"
        ]
        diversion_cruise_altitude = inputs[
            "data:mission:" + mission_type + ":diversion:altitude"
        ]
        climb_speed = inputs["data:mission:" + mission_type + ":climb:speed"]
        descent_speed = inputs["data:mission:" + mission_type + ":descent:speed"]

        TOargs = {
            "vmca": inputs["data:mission:sizing:takeoff:V_MCA"],
            "CL_alpha": inputs["data:aerodynamics:aircraft:takeoff:CL_alpha"],
            "CL0": inputs["data:aerodynamics:aircraft:takeoff:CL0"],
            "CL_max": inputs["data:aerodynamics:aircraft:takeoff:CL_max"],
            "H_list": inputs["data:aerodynamics:aircraft:low_speed:H"],
            "alpha_list": inputs["data:aerodynamics:aircraft:low_speed:alpha"],
            "CT_list": inputs["data:aerodynamics:aircraft:low_speed:CT"],
            # 'DCd_ext':inputs["data:aerodynamics:aircraft:low_speed:DCD_ext"],
            # 'DCd_feather':inputs["data:aerodynamics:aircraft:low_speed:DCD_feather"],
            "DCl0_CT": inputs["data:aerodynamics:aircraft:takeoff:CT_effect:DCL0"],
            "DCl_alpha_CT": inputs[
                "data:aerodynamics:aircraft:takeoff:CT_effect:DCL_alpha"
            ],
            "DCd_OEI": inputs["data:aerodynamics:aircraft:takeoff:OEI_effect:DCD"],
            "DCd_gd": inputs["data:aerodynamics:aircraft:takeoff:ground_effect:DCD"],
            "DCl_gd": inputs["data:aerodynamics:aircraft:takeoff:ground_effect:DCL"],
            "K_H": inputs["data:aerodynamics:aircraft:takeoff:ground_effect:K_H"],
            "DCd_lg": inputs["data:aerodynamics:aircraft:takeoff:lg_effect:DCD"],
            "DCl_lg": inputs["data:aerodynamics:aircraft:takeoff:lg_effect:DCL"],
            "kf": inputs["data:mission:sizing:takeoff:Friction_coefficient_no_brake"],
        }
        thrust_rates = {
            FlightPhase.CLIMB: inputs[
                "data:mission:" + mission_type + ":climb:thrust_rate"
            ],
            FlightPhase.DESCENT: inputs[
                "data:mission:" + mission_type + ":descent:thrust_rate"
            ],
        }
        high_speed_polar = Polar(
            inputs["data:aerodynamics:aircraft:cruise:CL"],
            inputs["data:aerodynamics:aircraft:cruise:CD"],
        )
        low_speed_climb_polar = Polar(
            inputs["data:aerodynamics:aircraft:low_speed:CL"],
            inputs["data:aerodynamics:aircraft:low_speed:CD"],
        )
        take_off_polar = Polar(
            inputs["data:aerodynamics:aircraft:takeoff:CL"],
            inputs["data:aerodynamics:aircraft:takeoff:CD"],
        )

        print("start main mission", TOW_input)

        # Take off segment =====================================================
        alpha_initial = -1.0 / 180 * np.pi
        initial_takeoff = FlightPoint(
            mass=TOW_input - inputs["data:mission:" + mission_type + ":taxi_out:fuel"],
            true_airspeed=0.001,
            altitude=inputs["data:mission:" + mission_type + ":takeoff:altitude"],
            ground_distance=0.0,
            alpha=alpha_initial,
        )

        take_off_calculator = TakeOffSegment(
            **TOargs,
            polar=take_off_polar,
            reference_area=reference_area,
            propulsion=propulsion_model,
            thrust_rate=inputs["data:mission:sizing:takeoff:thrust_rate"],
            name="take off",
            time_step=0.01,
        )

        take_off_flight_points = take_off_calculator.compute_from(initial_takeoff)

        print("     Take off completed")
        delta_time = [
            take_off_flight_points.time.iloc[1] - take_off_flight_points.time.iloc[0]
        ] + [
            take_off_flight_points.time.iloc[i + 1]
            - take_off_flight_points.time.iloc[i]
            for i in range(len(take_off_flight_points) - 1)
        ]
        take_off_flight_points["delta_time"] = np.array(delta_time, dtype=object)
        fuel_delta_mass = (
            take_off_flight_points.psfc
            * take_off_flight_points.TPshaft_power
            * take_off_flight_points.delta_time
        )
        take_off_flight_points["fuel_mass"] = fuel_delta_mass.cumsum()

        end_of_takeoff = FlightPoint(
            take_off_flight_points.loc[take_off_flight_points.name == "take off"].iloc[
                -1
            ]
        )

        # outputs["data:mission:"+mission_type+":takeoff:fuel"] = (
        #     end_of_takeoff.fuel_mass
        # )

        outputs[
            "data:mission:" + mission_type + ":takeoff:distance"
        ] = end_of_takeoff.ground_distance

        outputs["data:mission:" + mission_type + ":takeoff:V_rotate"] = FlightPoint(
            take_off_flight_points.loc[
                take_off_flight_points.alpha == alpha_initial
            ].iloc[-1]
        ).true_airspeed
        outputs["data:mission:" + mission_type + ":takeoff:V_liftoff"] = FlightPoint(
            take_off_flight_points.loc[
                take_off_flight_points.altitude
                == float(inputs["data:mission:" + mission_type + ":takeoff:altitude"])
            ].iloc[-1]
        ).true_airspeed
        outputs[
            "data:mission:" + mission_type + ":takeoff:V_2"
        ] = end_of_takeoff.true_airspeed

        # OEI ceiling flight =====================================================
        if mission_type == "sizing":
            altitude_OEI = (
                inputs["data:mission:" + mission_type + ":takeoff:altitude"]
                + 1500 * foot
            )
            density_OEI = Atmosphere(altitude_OEI, altitude_in_feet=False).density

            VSR = (
                2
                * end_of_takeoff.mass
                * g
                / (
                    inputs["data:aerodynamics:aircraft:takeoff:CL_max"]
                    * reference_area
                    * density_OEI
                )
            ) ** 0.5
            print(end_of_takeoff.true_airspeed, 1.18 * VSR)

            init_OEI_climb = FlightPoint(
                mass=TOW_input
                - inputs["data:mission:" + mission_type + ":takeoff:fuel"]
                - inputs["data:mission:" + mission_type + ":taxi_out:fuel"],
                equivalent_airspeed=1.18 * VSR,  # 135*knot,
                altitude=altitude_OEI,
                ground_distance=0.0,
            )
            low_speed_climb_polar_OEI = Polar(
                inputs["data:aerodynamics:aircraft:low_speed:CL"],
                inputs[
                    "data:aerodynamics:aircraft:low_speed:CD"
                ],  # +0.0046,#+0.002,   #0.002for AIAA study
            )

            OEI_calculator = ClimbPhaseOEI(
                polar=low_speed_climb_polar_OEI,
                reference_area=reference_area,
                propulsion=propulsion_model_OEI,
            )
            OEI_flight_points = OEI_calculator.compute_from(init_OEI_climb)
            try:
                flight_point_ceiling = OEI_flight_points[
                    OEI_flight_points.RC_penalty < 0.254
                ].iloc[
                    0
                ]  # 0.254m/s=50 ft/min
                ceiling_altitude = flight_point_ceiling.altitude_penalty
            except:
                print("Ceiling higher than 12000ft")
                ceiling_altitude = np.array([3657.6])
            outputs["data:mission:sizing:OEI_ceiling:altitude"] = ceiling_altitude
            print("ceiling_altitude", ceiling_altitude)

            print("     OEI flight completed")
        # Main mission flight =====================================================

        base_flight_calculator = RangedFlight(
            StandardFlight(
                propulsion=propulsion_model,
                reference_area=reference_area,
                low_speed_climb_polar=low_speed_climb_polar,
                high_speed_polar=high_speed_polar,
                cruise_mach=cruise_mach,
                thrust_rates=thrust_rates,
                climb_target_altitude=cruise_altitude * foot,
                climb_speed=climb_speed,
                descent_speed=descent_speed,
                # time_step=1,
            ),
            flight_distance,
        )

        # end_of_takeoff = FlightPoint(
        #     mass=TOW_input-inputs["data:mission:"+mission_type+":taxi_out:fuel"],
        #     true_airspeed=60,#inputs["data:mission:"+mission_type+":takeoff:V2"],
        #     altitude=inputs["data:mission:"+mission_type+":takeoff:altitude"] + 35 * foot,
        #     ground_distance=0.0,
        #     fuel_mass=10,
        #     time=60,
        # )

        flight_points = base_flight_calculator.compute_from(end_of_takeoff)

        print("END main mission", TOW_input)

        # updated start flight point for every parameters
        # end_of_takeoff = FlightPoint(flight_points.iloc[0])

        delta_time = [flight_points.time.iloc[1] - flight_points.time.iloc[0]] + [
            flight_points.time.iloc[i + 1] - flight_points.time.iloc[i]
            for i in range(len(flight_points) - 1)
        ]
        flight_points["delta_time"] = np.array(delta_time, dtype=object)
        fuel_delta_mass = (
            flight_points.psfc * flight_points.TPshaft_power * flight_points.delta_time
        )
        flight_points["fuel_mass"] = fuel_delta_mass.cumsum()

        if "PHFC" in self.options["propulsion_id"]:
            H2_delta_mass = flight_points.delta_time * flight_points.H2_fc
            flight_points["H2_mass"] = H2_delta_mass.cumsum()
            outputs["data:propulsion:electric_systems:fuel_cell:H2_flow_rate"] = (
                max(flight_points.H2_fc.values) / propulsion_model.motor_count
            )

        elif "PHEB" in self.options["propulsion_id"]:
            BAT_delta_energy = flight_points.delta_time * flight_points.BAT_ec  # W*s=J
            BAT_energy = BAT_delta_energy.cumsum()
            flight_points["BAT_energy"] = BAT_energy

        # Get flight points for each end of phase

        end_of_initial_climb = FlightPoint(
            flight_points.loc[flight_points.name == "initial climb"].iloc[-1]
        )

        # top_of_climb is the flight point where cruise altitude is reached
        top_of_climb = FlightPoint(
            flight_points[
                (flight_points.name == "climb")
                & (flight_points.altitude <= float(cruise_altitude * foot))
            ].iloc[-1]
        )
        # end_of_climb is the flight point where cruise altitude and cruise mach is reached
        end_of_climb = FlightPoint(
            flight_points.loc[flight_points.name == "climb"].iloc[-1]
        )
        # d2 = df[(df['l_ext']==l_ext) & (df['item']==item) & (df['wn']==wn) & (df['wd']==1)]
        end_of_cruise = FlightPoint(
            flight_points.loc[flight_points.name == "cruise"].iloc[-1]
        )

        end_of_descent = FlightPoint(
            flight_points.loc[flight_points.name == "descent"].iloc[-1]
        )

        # Set OpenMDAO outputs
        # outputs["data:mission:"+mission_type+":initial_climb:fuel"] = (
        #     end_of_initial_climb.fuel_mass
        # )
        outputs[
            "data:mission:" + mission_type + ":cruise:max_mach"
        ] = end_of_climb.max_mach

        outputs["data:mission:" + mission_type + ":initial_climb:fuel"] = (
            end_of_initial_climb.fuel_mass - end_of_takeoff.fuel_mass
        )

        outputs["data:mission:" + mission_type + ":main_route:climb:fuel"] = (
            end_of_climb.fuel_mass - end_of_initial_climb.fuel_mass
        )
        outputs["data:mission:" + mission_type + ":main_route:cruise:fuel"] = (
            end_of_cruise.fuel_mass - end_of_climb.fuel_mass
        )

        outputs["data:mission:" + mission_type + ":main_route:descent:fuel"] = (
            end_of_descent.fuel_mass - end_of_cruise.fuel_mass
        )

        if "PHFC" in self.options["propulsion_id"]:

            outputs[
                "data:mission:" + mission_type + ":initial_climb:H2"
            ] = end_of_initial_climb.H2_mass
            outputs["data:mission:" + mission_type + ":main_route:climb:H2"] = (
                end_of_climb.H2_mass - end_of_initial_climb.H2_mass
            )
            outputs["data:mission:" + mission_type + ":main_route:cruise:H2"] = (
                end_of_cruise.H2_mass - end_of_climb.H2_mass
            )
            outputs["data:mission:" + mission_type + ":main_route:descent:H2"] = (
                end_of_descent.H2_mass - end_of_cruise.H2_mass
            )

        outputs["data:mission:" + mission_type + ":initial_climb:distance"] = (
            end_of_initial_climb.ground_distance - end_of_takeoff.ground_distance
        )
        outputs["data:mission:" + mission_type + ":main_route:climb:distance"] = (
            end_of_climb.ground_distance - end_of_initial_climb.ground_distance
        )
        outputs["data:mission:" + mission_type + ":main_route:cruise:distance"] = (
            end_of_cruise.ground_distance - end_of_climb.ground_distance
        )

        outputs["data:mission:" + mission_type + ":main_route:descent:distance"] = (
            end_of_descent.ground_distance - end_of_cruise.ground_distance
        )
        outputs["data:mission:" + mission_type + ":initial_climb:duration"] = (
            end_of_initial_climb.time - end_of_takeoff.time
        )
        outputs["data:mission:" + mission_type + ":main_route:climb:duration"] = (
            end_of_climb.time - end_of_initial_climb.time
        )

        outputs["data:mission:" + mission_type + ":main_route:climb:TTC"] = (
            top_of_climb.time - end_of_initial_climb.time
        )

        outputs["data:mission:" + mission_type + ":main_route:cruise:duration"] = (
            end_of_cruise.time - end_of_climb.time
        )

        outputs["data:mission:" + mission_type + ":main_route:descent:duration"] = (
            end_of_descent.time - end_of_cruise.time
        )

        # Diversion flight =====================================================
        hybrid_diversion = False

        # if hybrid_diversion:
        #     propulsion_model = HybridEngineSet(
        #         self._engine_wrapper.get_model(inputs), inputs["data:geometry:propulsion:engine:count"],inputs["data:geometry:propulsion:motor:count"]
        #     )
        # else:
        #     propulsion_model = TPEngineSet(self._engine_wrapper.get_model(inputs), inputs["data:geometry:propulsion:engine:count"])
        diversion_distance = inputs[
            "data:mission:" + mission_type + ":diversion:distance"
        ]

        # tipo qui gli puoi dare come mpdello quello classico turboprop invece dell'PHFC_engine

        diversion_flight_calculator = RangedFlight(
            DiversionFlight(
                propulsion=propulsion_model,
                reference_area=reference_area,
                low_speed_climb_polar=low_speed_climb_polar,
                high_speed_polar=high_speed_polar,
                cruise_mach=diversion_mach,
                thrust_rates=thrust_rates,
                climb_target_altitude=diversion_cruise_altitude * foot,
                climb_speed=climb_speed,
                descent_speed=[95.0],
            ),
            diversion_distance,
        )
        diversion_flight_points = diversion_flight_calculator.compute_from(
            end_of_descent
        )

        # Complete flight_points with H2 and fuel cumulative masses

        delta_time = [
            diversion_flight_points.time.iloc[1] - diversion_flight_points.time.iloc[0]
        ] + [
            diversion_flight_points.time.iloc[i + 1]
            - diversion_flight_points.time.iloc[i]
            for i in range(len(diversion_flight_points) - 1)
        ]
        diversion_flight_points["delta_time"] = np.array(delta_time)
        fuel_delta_mass = (
            diversion_flight_points.psfc
            * diversion_flight_points.TPshaft_power
            * diversion_flight_points.delta_time
        )
        diversion_flight_points["fuel_mass"] = fuel_delta_mass.cumsum()

        if hybrid_diversion:
            H2_delta_mass = (
                diversion_flight_points.delta_time * diversion_flight_points.H2_fc
            )
            diversion_flight_points["H2_mass"] = H2_delta_mass.cumsum()
        if "PHEB" in self.options["propulsion_id"]:
            BAT_delta_energy_div = (
                diversion_flight_points.delta_time * diversion_flight_points.BAT_ec
            )  # W*s=J
            BAT_energy_div = BAT_delta_energy_div.cumsum()
            diversion_flight_points["BAT_energy"] = BAT_energy_div

        # Get flight points for each end of phase
        end_of_diversion_climb = FlightPoint(
            diversion_flight_points.loc[
                diversion_flight_points.name == "diversion climb"
            ].iloc[-1]
        )
        end_of_diversion_cruise = FlightPoint(
            diversion_flight_points.loc[
                diversion_flight_points.name == "diversion cruise"
            ].iloc[-1]
        )
        end_of_diversion_descent = FlightPoint(
            diversion_flight_points.loc[
                diversion_flight_points.name == "diversion descent"
            ].iloc[-1]
        )

        # # rename phases because all flight points will be concatenated later.
        # diversion_flight_points.loc[
        #     diversion_flight_points.name == "climb"
        # ].name = "diversion climb"
        # diversion_flight_points.loc[
        #     diversion_flight_points.name == "cruise"
        # ].name = "diversion cruise"
        # diversion_flight_points.loc[
        #     diversion_flight_points.name == "descent"
        # ].name = "diversion descent"

        # Set OpenMDAO outputs

        outputs[
            "data:mission:" + mission_type + ":diversion:climb:fuel"
        ] = end_of_diversion_climb.fuel_mass
        outputs["data:mission:" + mission_type + ":diversion:cruise:fuel"] = (
            end_of_diversion_cruise.fuel_mass - end_of_diversion_climb.fuel_mass
        )
        outputs["data:mission:" + mission_type + ":diversion:descent:fuel"] = (
            end_of_diversion_descent.fuel_mass - end_of_diversion_cruise.fuel_mass
        )

        if hybrid_diversion:

            outputs[
                "data:mission:" + mission_type + ":diversion:climb:H2"
            ] = end_of_diversion_climb.H2_mass
            outputs["data:mission:" + mission_type + ":diversion:cruise:H2"] = (
                end_of_diversion_cruise.H2_mass - end_of_diversion_climb.H2_mass
            )
            outputs["data:mission:" + mission_type + ":diversion:descent:H2"] = (
                end_of_diversion_descent.H2_mass - end_of_diversion_cruise.H2_mass
            )

        # Set OpenMDAO outputs

        outputs["data:mission:" + mission_type + ":diversion:climb:distance"] = (
            end_of_diversion_climb.ground_distance - end_of_descent.ground_distance
        )
        outputs["data:mission:" + mission_type + ":diversion:cruise:distance"] = (
            end_of_diversion_cruise.ground_distance
            - end_of_diversion_climb.ground_distance
        )
        outputs["data:mission:" + mission_type + ":diversion:descent:distance"] = (
            end_of_diversion_descent.ground_distance
            - end_of_diversion_cruise.ground_distance
        )
        outputs["data:mission:" + mission_type + ":diversion:climb:duration"] = (
            end_of_diversion_climb.time - end_of_descent.time
        )
        outputs["data:mission:" + mission_type + ":diversion:cruise:duration"] = (
            end_of_diversion_cruise.time - end_of_diversion_climb.time
        )
        outputs["data:mission:" + mission_type + ":diversion:descent:duration"] = (
            end_of_diversion_descent.time - end_of_diversion_cruise.time
        )

        # Holding ==============================================================
        hybrid_holding = False

        # if hybrid_holding:
        #     propulsion_model = HybridEngineSet(
        #         self._engine_wrapper.get_model(inputs), inputs["data:geometry:propulsion:engine:count"],inputs["data:geometry:propulsion:motor:count"]
        #     )
        # else:
        #     propulsion_model = TPEngineSet(self._engine_wrapper.get_model(inputs), inputs["data:geometry:propulsion:engine:count"])

        holding_duration = inputs["data:mission:" + mission_type + ":holding:duration"]

        holding_calculator = HoldSegment(
            target=FlightPoint(time=holding_duration),
            propulsion=propulsion_model,
            reference_area=reference_area,
            polar=high_speed_polar,
            name="holding",
            engine_setting=EngineSetting.CRUISE,
        )

        holding_flight_points = holding_calculator.compute_from(
            end_of_diversion_descent
        )
        # holding_flight_points = holding_calculator.compute_from(end_of_descent)

        # Complete flight_points with H2 and fuel cumulative masses
        delta_time = [
            holding_flight_points.time.iloc[1] - holding_flight_points.time.iloc[0]
        ] + [
            holding_flight_points.time.iloc[i + 1] - holding_flight_points.time.iloc[i]
            for i in range(len(holding_flight_points) - 1)
        ]
        holding_flight_points["delta_time"] = np.array(delta_time)
        fuel_delta_mass = (
            holding_flight_points.psfc
            * holding_flight_points.TPshaft_power
            * holding_flight_points.delta_time
        )
        holding_flight_points["fuel_mass"] = fuel_delta_mass.cumsum()

        if hybrid_holding:
            H2_delta_mass = (
                holding_flight_points.delta_time * holding_flight_points.H2_fc
            )
            holding_flight_points["H2_mass"] = H2_delta_mass.cumsum()

        end_of_holding = FlightPoint(holding_flight_points.iloc[-1])

        outputs[
            "data:mission:" + mission_type + ":holding:fuel"
        ] = end_of_holding.fuel_mass

        if hybrid_holding:

            outputs[
                "data:mission:" + mission_type + ":holding:H2"
            ] = end_of_holding.H2_mass

        # Approach and landing ==============================================================
        """end_of_landing= FlightPoint(
            mass=end_of_holding.mass- inputs["data:mission:"+mission_type+":landing:fuel"],
            true_airspeed=0.,
            altitude=0.,
            ground_distance=end_of_holding.ground_distance+1000., 
        )"""

        # Taxi-in ==============================================================
        """taxi_in_duration = inputs["data:mission:"+mission_type+":taxi_in:duration"]
        taxi_in_thrust_rate = inputs["data:mission:"+mission_type+":taxi_in:thrust_rate"]




        taxi_in_calculator = TaxiSegment(
            target=FlightPoint(time=taxi_in_duration),
            propulsion=propulsion_model,
            thrust_rate=taxi_in_thrust_rate,
            name="taxi-in",
        )
        start_of_taxi_in = FlightPoint(end_of_landing)
        start_of_taxi_in.true_airspeed = inputs["data:mission:"+mission_type+":taxi_in:speed"]
        taxi_in_flight_points = taxi_in_calculator.compute_from(end_of_landing)

        end_of_taxi_in = FlightPoint(taxi_in_flight_points.iloc[-1])
        outputs["data:mission:"+mission_type+":taxi_in:fuel"] = end_of_landing.mass - end_of_taxi_in.mass"""
        """end_of_taxi_in= FlightPoint(
            mass=end_of_landing.mass- inputs["data:mission:"+mission_type+":taxi_in:fuel"],
            altitude=0.,
            ground_distance=end_of_landing.ground_distance, 
        )"""
        # Final ================================================================
        allowances = (
            inputs["data:mission:" + mission_type + ":takeoff:fuel"]
            + inputs["data:mission:" + mission_type + ":taxi_out:fuel"]
            + inputs["data:mission:" + mission_type + ":landing:fuel"]
            + inputs["data:mission:" + mission_type + ":taxi_in:fuel"]
        )
        fuel_route = end_of_descent.fuel_mass
        H2_route = sum(
            filter(
                None,
                [
                    end_of_holding.H2_mass,
                    end_of_diversion_descent.H2_mass,
                    end_of_descent.H2_mass,
                ],
            )
        )
        # BAT_route = sum(filter(None,[end_of_descent.BAT_energy, end_of_diversion_descent.BAT_energy])) #Joule

        # H2_route = sum(filter(None, [end_of_descent.H2_mass]))
        BAT_route = sum(filter(None, [end_of_descent.BAT_energy]))  # Joule

        allowances_H2 = 0.15 * H2_route  # h2 to keep cryotemp at the end of the mission

        outputs["data:mission:" + mission_type + ":trip_fuel"] = fuel_route
        outputs["data:mission:" + mission_type + ":block_fuel"] = (
            fuel_route + allowances
        )
        outputs["data:mission:" + mission_type + ":ZFW"] = (
            end_of_holding.mass
            - inputs["data:mission:" + mission_type + ":landing:fuel"]
            - inputs["data:mission:" + mission_type + ":taxi_in:fuel"]
            - 0.05 * fuel_route
            - allowances_H2
        )
        outputs["data:mission:" + mission_type + ":fuel"] = (
            end_of_holding.fuel_mass
            + end_of_diversion_descent.fuel_mass
            + end_of_descent.fuel_mass
            + allowances
            + 0.05 * fuel_route
        )
        # outputs["data:mission:"+mission_type+":ZFW"] = end_of_descent.mass- inputs["data:mission:"+mission_type+":landing:fuel"] - inputs["data:mission:"+mission_type+":taxi_in:fuel"]- 0.05 * fuel_route -0.35*H2_route
        # outputs["data:mission:"+mission_type+":fuel"] = end_of_descent.fuel_mass+ allowances + 0.05 * fuel_route

        if "PHFC" in self.options["propulsion_id"]:
            # outputs["data:mission:"+mission_type+":H2"] = H2_route*1.35
            # outputs["data:mission:"+mission_type+":trip_H2"]=end_of_descent.H2_mass*1.35
            outputs["data:mission:" + mission_type + ":H2"] = H2_route + allowances_H2
            outputs["data:mission:" + mission_type + ":trip_H2"] = (
                end_of_descent.H2_mass + allowances_H2
            )
        elif "PHEB" in self.options["propulsion_id"]:
            outputs["data:mission:" + mission_type + ":BAT_energy"] = BAT_route

        # elif 'TP' in self.options["propulsion_id"]:

        #     outputs["data:mission:"+mission_type+":trip_fuel"]=fuel_route
        #     outputs["data:mission:"+mission_type+":block_fuel"]=TOW_input - end_of_descent.mass +  inputs["data:mission:"+mission_type+":landing:fuel"]+inputs["data:mission:"+mission_type+":taxi_in:fuel"]
        #     outputs["data:mission:"+mission_type+":ZFW"] = end_of_holding.mass- inputs["data:mission:"+mission_type+":landing:fuel"] - inputs["data:mission:"+mission_type+":taxi_in:fuel"]- 0.05 * fuel_route
        #     outputs["data:mission:"+mission_type+":fuel"] = (
        #         TOW_input - outputs["data:mission:"+mission_type+":ZFW"]
        #     )

        if "L1" in self.options["propulsion_id"]:
            if mission_type == "sizing":
                OEI_flight_points.drop(
                    columns=["TP_total_temperature", "TP_total_pressure"]
                ).applymap(lambda x: np.asscalar(np.asarray(x))).to_csv(
                    self.options["out_file"][:-4] + "_OEI.csv"
                )

            self.flight_points = (
                pd.concat(
                    [
                        take_off_flight_points.drop(
                            columns=["TP_total_temperature", "TP_total_pressure"]
                        ),
                        flight_points.drop(
                            columns=["TP_total_temperature", "TP_total_pressure"]
                        ),
                        diversion_flight_points.drop(
                            columns=["TP_total_temperature", "TP_total_pressure"]
                        ),
                        holding_flight_points.drop(
                            columns=["TP_total_temperature", "TP_total_pressure"]
                        ),
                        # taxi_in_flight_points,
                    ]
                )
                .reset_index(drop=True)
                .applymap(lambda x: np.asscalar(np.asarray(x)))
            )

            thermo_data = pd.concat(
                [
                    take_off_flight_points[
                        ["TP_total_temperature", "TP_total_pressure"]
                    ],
                    flight_points[["TP_total_temperature", "TP_total_pressure"]],
                    diversion_flight_points[
                        ["TP_total_temperature", "TP_total_pressure"]
                    ],
                    holding_flight_points[
                        ["TP_total_temperature", "TP_total_pressure"]
                    ],
                    # taxi_in_flight_points,
                ]
            ).reset_index(drop=True)
            if self.options["out_file"]:
                self.flight_points.to_csv(self.options["out_file"])
                thermo_data.to_csv(self.options["out_file"][:-4] + "_thermo.csv")

        else:
            if mission_type == "sizing":
                OEI_flight_points.applymap(lambda x: np.asscalar(np.asarray(x))).to_csv(
                    self.options["out_file"][:-4] + "_OEI.csv"
                )

            self.flight_points = (
                pd.concat(
                    [
                        take_off_flight_points,
                        flight_points,
                        diversion_flight_points,
                        holding_flight_points,
                        # taxi_in_flight_points,
                    ]
                )
                .reset_index(drop=True)
                .applymap(lambda x: np.asscalar(np.asarray(x)))
            )

            if self.options["out_file"]:
                self.flight_points.to_csv(self.options["out_file"])
