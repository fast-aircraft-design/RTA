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
from rhea.models.propulsion.fuel_engine.constants import POWER_RATE_COUNT
from fastoad.models.performances.breguet import Breguet

# from fastoad.models.performances.mission.segments.hold import HoldSegment
from rhea.models.performances.mission.segments.holding import HoldingSegment

from fastoad.models.performances.mission.segments.taxi import TaxiSegment
from rhea.models.performances.mission.segments.takeoff_flight import TakeOffSegment
from rhea.models.performances.mission.segments.OEI_ceiling import ClimbPhaseOEI

# from fastoad.models.propulsion import EngineSet
from models.propulsion.fuel_engine.turboprop_engine.base import (
    TPEngineSet,
    TPH2EngineSet,
)
from models.propulsion.fuel_engine.hybrid_engine.base import HybridEngineSet

from scipy.constants import foot, nautical_mile, knot, hour, g

from rhea.models.performances.mission.flight.base import RangedFlight
from ..flight.standard_regional_flight import StandardFlight, DiversionFlight

# from fastoad.models.performances.mission.flight_point import FlightPoint #v0.50a
from fastoad.base.flight_point import FlightPoint
from fastoad.models.performances.mission.polar import Polar
from fastoad.base.dict import AddKeyAttributes, AddKeyAttribute
from fastoad.utils.physics import Atmosphere
from fastoad.constants import EngineSetting
from fastoad.models.performances.mission.segments.base import (
    SEGMENT_KEYWORD_ARGUMENTS,
)  # v0.5.2b

AddKeyAttributes(["alpha", "max_mach", "ceiling", "DISA"])(FlightPoint)


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
        self.options.declare("prop_fid", default="ADT", types=str)

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
        # self.add_input("data:TLAR:NPAX", val=1.)
        # self.add_input("setup:conv",  np.nan)

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

        self.add_input("data:weight:aircraft:MTOW", np.nan, units="kg")

        self.add_input("data:mission:sizing:taxi_out:fuel", np.nan, units="kg")
        self.add_input("data:mission:sizing:taxi_out:duration", 360.0, units="s")
        self.add_input("data:mission:sizing:takeoff:duration", 60.0, units="s")
        self.add_input("data:mission:sizing:landing:duration", 120.0, units="s")
        self.add_input("data:mission:sizing:taxi_in:duration", 240.0, units="s")

        self.add_input("data:mission:sizing:taxi_out:H2", 0.0, units="kg")
        self.add_input("data:mission:sizing:DISA", 0)

        self.add_input("data:mission:sizing:takeoff:altitude", np.nan, units="m")
        self.add_input("data:mission:sizing:takeoff:fuel", np.nan, units="kg")
        self.add_input("data:mission:sizing:takeoff:H2", 0.0, units="kg")
        self.add_input("data:mission:sizing:takeoff:V_2", units="m/s", val=60.0)
        self.add_input("data:mission:sizing:takeoff:TP_power_rate", val=1.0)
        self.add_input("data:mission:sizing:takeoff:EM_power_rate", val=0.0)

        self.add_input("data:mission:sizing:initial_climb:EM_power_rate", 0.0)
        self.add_input("data:mission:sizing:initial_climb:TP_power_rate", 1.0)

        self.add_input("data:mission:sizing:climb:thrust_rate", np.nan)
        self.add_input(
            "data:mission:sizing:climb:EM_power_rate", 0.0, shape=POWER_RATE_COUNT
        )
        self.add_input(
            "data:mission:sizing:climb:TP_power_rate", 1.0, shape=POWER_RATE_COUNT
        )

        self.add_input("data:mission:sizing:diversion:climb:EM_power_rate", 0.0)
        self.add_input("data:mission:sizing:diversion:climb:TP_power_rate", 1.0)

        self.add_input("data:mission:sizing:climb:speed", np.nan, units="m/s")
        self.add_input(
            "data:mission:sizing:main_route:cruise:altitude", np.nan, units="ft"
        )
        self.add_input("data:mission:sizing:cruise:mach", 0.45)
        self.add_input("data:mission:sizing:cruise:EM_power_rate", 0.0)
        self.add_input("data:mission:sizing:cruise:TP_power_rate", 1.0)

        self.add_input("data:mission:sizing:diversion:cruise:EM_power_rate", 0.0)
        self.add_input("data:mission:sizing:diversion:cruise:TP_power_rate", 1.0)

        self.add_input("data:mission:sizing:descent:thrust_rate", np.nan)
        self.add_input("data:mission:sizing:descent:speed", np.nan, units="m/s")

        self.add_input("data:mission:sizing:diversion:H2_allowances", 0.0)
        self.add_input("data:mission:sizing:diversion:distance", np.nan, units="m")
        self.add_input("data:mission:sizing:diversion:altitude", np.nan, units="ft")
        self.add_input("data:mission:sizing:diversion:mach", np.nan)
        self.add_input("data:mission:sizing:holding:duration", np.nan, units="s")

        self.add_input("data:mission:sizing:holding:EM_power_rate", 0.0)
        self.add_input("data:mission:sizing:holding:TP_power_rate", 1.0)

        self.add_input("data:mission:sizing:landing:fuel", np.nan, units="kg")
        self.add_input("data:mission:sizing:taxi_in:fuel", np.nan, units="kg")
        self.add_input("data:mission:sizing:landing:H2", 0.0, units="kg")
        self.add_input("data:mission:sizing:taxi_in:H2", 0.0, units="kg")

        self.add_input("data:mission:sizing:taxi_in:speed", np.nan, units="m/s")
        self.add_input("data:mission:sizing:taxi_in:thrust_rate", np.nan)

        # Inputs needed in order to use the sizing output file as input .xml for the DOC mission evaluation-----------------------------------------------------------------------------------
        self.add_input("data:mission:DOC:cruise:mach", np.nan)
        self.add_input("data:mission:DOC:range", np.nan, units="m")
        # self.add_input("data:weight:aircraft:OWE", np.nan, units="kg")
        self.add_input("data:mission:DOC:payload", np.nan, units="kg")

        self.add_input("data:mission:DOC:TOW", np.nan, units="kg")

        self.add_input("data:mission:DOC:taxi_out:fuel", np.nan, units="kg")
        self.add_input("data:mission:DOC:taxi_out:H2", 0.0, units="kg")

        self.add_input("data:mission:DOC:takeoff:altitude", np.nan, units="m")
        self.add_input("data:mission:DOC:takeoff:fuel", np.nan, units="kg")
        self.add_input("data:mission:DOC:takeoff:H2", 0.0, units="kg")

        # self.add_input("data:mission:DOC:takeoff:V_2", units="m/s",val=60.)
        # self.add_input("data:mission:DOC:takeoff:duration", units="s",val=60.)

        self.add_input("data:mission:DOC:climb:thrust_rate", np.nan)
        self.add_input("data:mission:DOC:climb:speed", np.nan, units="m/s")
        self.add_input(
            "data:mission:DOC:main_route:cruise:altitude", np.nan, units="ft"
        )
        self.add_input("data:mission:DOC:descent:thrust_rate", np.nan)
        self.add_input("data:mission:DOC:descent:speed", np.nan, units="m/s")

        self.add_input("data:mission:DOC:diversion:H2_allowances", 0.0)
        self.add_input("data:mission:DOC:diversion:distance", np.nan, units="m")
        self.add_input("data:mission:DOC:diversion:altitude", np.nan, units="ft")
        self.add_input("data:mission:DOC:diversion:mach", np.nan)
        self.add_input("data:mission:DOC:holding:duration", np.nan, units="s")

        self.add_input("data:mission:DOC:landing:fuel", np.nan, units="kg")
        self.add_input("data:mission:DOC:taxi_in:fuel", np.nan, units="kg")
        self.add_input("data:mission:DOC:landing:H2", 0.0, units="kg")
        self.add_input("data:mission:DOC:taxi_in:H2", 0.0, units="kg")

        self.add_input("data:mission:DOC:taxi_in:duration", np.nan, units="s")
        self.add_input("data:mission:DOC:taxi_in:speed", np.nan, units="m/s")
        self.add_input("data:mission:DOC:taxi_in:thrust_rate", np.nan)

        # Outputs ----------------------------------------------------------------------------------

        self.add_output("data:mission:sizing:cruise:max_mach")
        self.add_output("data:mission:sizing:climb:operational_ceiling", 0, units="m")
        self.add_output(
            "data:mission:sizing:diversion:climb:operational_ceiling", 0, units="m"
        )

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

        self.add_output("data:mission:sizing:diversion:climb:fuel", units="kg", val=0.0)
        self.add_output(
            "data:mission:sizing:diversion:cruise:fuel", units="kg", val=0.0
        )
        self.add_output(
            "data:mission:sizing:diversion:descent:fuel", units="kg", val=0.0
        )
        self.add_output("data:mission:sizing:diversion:climb:H2", units="kg", val=0.0)
        self.add_output("data:mission:sizing:diversion:cruise:H2", units="kg", val=0.0)
        self.add_output("data:mission:sizing:diversion:descent:H2", units="kg", val=0.0)

        self.add_output(
            "data:mission:sizing:diversion:climb:distance", units="m", val=0.0
        )
        self.add_output(
            "data:mission:sizing:diversion:cruise:distance", units="m", val=0.0
        )
        self.add_output(
            "data:mission:sizing:diversion:descent:distance", units="m", val=0.0
        )

        self.add_output(
            "data:mission:sizing:diversion:climb:duration", units="s", val=0.0
        )
        self.add_output(
            "data:mission:sizing:diversion:cruise:duration", units="s", val=0.0
        )
        self.add_output(
            "data:mission:sizing:diversion:descent:duration", units="s", val=0.0
        )

        self.add_output("data:mission:sizing:holding:fuel", units="kg")
        self.add_output("data:mission:sizing:holding:H2", units="kg", val=0.0)

        self.add_output("data:mission:sizing:ZFW", units="kg")
        self.add_output("data:mission:sizing:fuel", units="kg")
        self.add_output("data:mission:sizing:H2", units="kg", val=0.0)
        self.add_output("data:mission:sizing:BAT_energy", units="J", val=0.0)
        self.add_output("data:mission:sizing:trip_fuel", units="kg")
        self.add_output("data:mission:sizing:trip_H2", units="kg", val=0.0)
        self.add_output("data:mission:sizing:block_fuel", units="kg")
        self.add_output("data:mission:sizing:block_time", units="s")

        self.declare_partials(["*"], ["*"])

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        loop_input = inputs["data:weight:aircraft:MTOW"]
        mission_type = "sizing"
        try:
            self.compute_mission(inputs, outputs, loop_input, mission_type)
        except IndexError:
            self.compute_breguet(inputs, outputs, loop_input, mission_type)

    def compute_breguet(self, inputs, outputs, loop_input, mission_type):

        if (
            "PH" in self.options["propulsion_id"]
            or "FH" in self.options["propulsion_id"]
        ):
            # propulsion_model = HybridEngineSet(
            #     self._engine_wrapper.get_model(inputs), inputs["data:geometry:propulsion:engine:count"],inputs["data:geometry:propulsion:motor:count"]
            # )
            propulsion_model = HybridEngineSet(
                self._engine_wrapper.get_model(inputs),
                inputs["data:geometry:propulsion:engine:count"],
            )
        elif "H2" in self.options["propulsion_id"]:
            propulsion_model = TPH2EngineSet(
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
            cruise_mach = inputs["data:mission:sizing:cruise:mach"]
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
            cruise_mach = inputs["data:mission:sizing:cruise:mach"]
            diversion_mach = inputs["data:mission:sizing:diversion:mach"]
            TOW_input = loop_inputs
        elif mission_type == "DOC":
            flight_distance = inputs["data:mission:DOC:range"]
            cruise_mach = inputs["data:mission:DOC:cruise:mach"]
            diversion_mach = inputs["data:mission:DOC:diversion:mach"]
            TOW_input = loop_inputs
        elif mission_type == "payload_range":
            flight_distance = loop_inputs[0]
            cruise_mach = inputs["data:mission:sizing:cruise:mach"]
            diversion_mach = inputs["data:mission:sizing:diversion:mach"]
            TOW_input = loop_inputs[1]
            mission_type = "DOC"

        # if 'PHFC' in self.options["propulsion_id"]:
        if (
            "PH" in self.options["propulsion_id"]
            or "FH" in self.options["propulsion_id"]
        ):
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
        elif "H2" in self.options["propulsion_id"]:
            propulsion_model = TPH2EngineSet(
                self._engine_wrapper.get_model(inputs),
                inputs["data:geometry:propulsion:engine:count"],
            )
            propulsion_model_OEI = TPH2EngineSet(
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

        cruise_altitude = inputs[
            "data:mission:" + mission_type + ":main_route:cruise:altitude"
        ]
        diversion_cruise_altitude = inputs[
            "data:mission:" + mission_type + ":diversion:altitude"
        ]
        climb_speed = inputs["data:mission:" + mission_type + ":climb:speed"]
        descent_speed = inputs["data:mission:" + mission_type + ":descent:speed"]

        thrust_rates = {
            FlightPhase.CLIMB: inputs[
                "data:mission:" + mission_type + ":climb:thrust_rate"
            ],
            FlightPhase.DESCENT: inputs[
                "data:mission:" + mission_type + ":descent:thrust_rate"
            ],
        }

        electric_power_rates = {
            FlightPhase.INITIAL_CLIMB: inputs[
                "data:mission:" + mission_type + ":initial_climb:EM_power_rate"
            ],
            FlightPhase.CLIMB: inputs[
                "data:mission:" + mission_type + ":climb:EM_power_rate"
            ],
            FlightPhase.CRUISE: inputs[
                "data:mission:" + mission_type + ":cruise:EM_power_rate"
            ],
        }

        turbine_power_rates = {
            FlightPhase.INITIAL_CLIMB: inputs[
                "data:mission:" + mission_type + ":initial_climb:TP_power_rate"
            ],
            FlightPhase.CLIMB: inputs[
                "data:mission:" + mission_type + ":climb:TP_power_rate"
            ],
            FlightPhase.CRUISE: inputs[
                "data:mission:" + mission_type + ":cruise:TP_power_rate"
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

        print(
            "start main mission", TOW_input
        )  # , int(propulsion_model.engine.Elec_nom_power),int(propulsion_model.engine.RTO_power) )

        # Main mission flight =====================================================

        base_flight_calculator = RangedFlight(
            StandardFlight(
                propulsion=propulsion_model,
                reference_area=reference_area,
                low_speed_climb_polar=low_speed_climb_polar,
                high_speed_polar=high_speed_polar,
                cruise_mach=cruise_mach,
                thrust_rates=thrust_rates,
                electric_power_rates=electric_power_rates,
                turbine_power_rates=turbine_power_rates,
                climb_target_altitude=cruise_altitude * foot,
                climb_speed=climb_speed,
                descent_speed=descent_speed,
                # time_step=1,
            ),
            flight_distance,
        )

        end_of_takeoff = FlightPoint(
            mass=TOW_input
            - inputs["data:mission:" + mission_type + ":taxi_out:fuel"]
            - inputs["data:mission:" + mission_type + ":takeoff:fuel"],
            true_airspeed=60,  # inputs["data:mission:"+mission_type+":takeoff:V_2"],
            altitude=inputs["data:mission:" + mission_type + ":takeoff:altitude"]
            + 35 * foot,
            ground_distance=0.0,
            fuel_mass=10,  # inputs["data:mission:"+mission_type+":takeoff:fuel"],
            time=60,  # inputs["data:mission:"+mission_type+":takeoff:duration"],
            DISA=inputs["data:mission:" + mission_type + ":DISA"],
        )

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

        if "FC" in self.options["propulsion_id"]:
            H2_delta_mass = flight_points.delta_time * flight_points.H2_fc
            flight_points["H2_mass"] = H2_delta_mass.cumsum()
            outputs["data:propulsion:electric_systems:fuel_cell:H2_flow_rate"] = (
                max(flight_points.H2_fc.values) / propulsion_model.motor_count
            )

        if "H2" in self.options["propulsion_id"]:
            H2_delta_mass = flight_points.delta_time * flight_points.H2_fc
            flight_points["H2_mass"] = H2_delta_mass.cumsum()
            outputs["data:propulsion:electric_systems:fuel_cell:H2_flow_rate"] = (
                max(flight_points.H2_fc.values) / propulsion_model.engine_count
            )

        elif "EB" in self.options["propulsion_id"]:
            BAT_delta_energy = flight_points.delta_time * flight_points.BAT_ec  # W*s=J
            BAT_energy = BAT_delta_energy.cumsum()
            flight_points["BAT_energy"] = BAT_energy

        # Get flight points for each end of phase

        end_of_initial_climb = FlightPoint(
            flight_points.loc[flight_points.name == "initial climb"].iloc[-1]
        )

        # top_of_climb is the flight point where cruise altitude is reached

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

        top_of_climb = FlightPoint(
            flight_points[
                (flight_points.name == "climb")
                & (flight_points.altitude < float(end_of_cruise.altitude))
            ].iloc[-1]
        )
        # Set OpenMDAO outputs
        # outputs["data:mission:"+mission_type+":initial_climb:fuel"] = (
        #     end_of_initial_climb.fuel_mass
        # )
        outputs[
            "data:mission:" + mission_type + ":cruise:max_mach"
        ] = end_of_climb.max_mach

        ###following lines needed in case last climb segment cannot reach target and the calculated ceiling is the one of the previous climb segment
        outputs[
            "data:mission:" + mission_type + ":climb:operational_ceiling"
        ] = top_of_climb.ceiling
        if end_of_cruise.altitude < cruise_altitude * foot:
            outputs[
                "data:mission:" + mission_type + ":climb:operational_ceiling"
            ] = end_of_cruise.altitude
        ##########################

        outputs[
            "data:mission:" + mission_type + ":initial_climb:fuel"
        ] = end_of_initial_climb.fuel_mass  # - end_of_takeoff.fuel_mass

        outputs["data:mission:" + mission_type + ":main_route:climb:fuel"] = (
            end_of_climb.fuel_mass - end_of_initial_climb.fuel_mass
        )
        outputs["data:mission:" + mission_type + ":main_route:cruise:fuel"] = (
            end_of_cruise.fuel_mass - end_of_climb.fuel_mass
        )

        outputs["data:mission:" + mission_type + ":main_route:descent:fuel"] = (
            end_of_descent.fuel_mass - end_of_cruise.fuel_mass
        )

        if (
            "FC" in self.options["propulsion_id"]
            or "H2" in self.options["propulsion_id"]
        ):

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
        diversion_electric_power_rates = {
            FlightPhase.CLIMB: inputs[
                "data:mission:" + mission_type + ":diversion:climb:EM_power_rate"
            ],
            FlightPhase.CRUISE: inputs[
                "data:mission:" + mission_type + ":diversion:cruise:EM_power_rate"
            ],
        }

        diversion_turbine_power_rates = {
            FlightPhase.CLIMB: inputs[
                "data:mission:" + mission_type + ":diversion:climb:TP_power_rate"
            ],
            FlightPhase.CRUISE: inputs[
                "data:mission:" + mission_type + ":diversion:cruise:TP_power_rate"
            ],
        }
        if (
            "FHFC" in self.options["propulsion_id"]
            or "H2" in self.options["propulsion_id"]
        ):
            hybrid_diversion = True

        # if hybrid_diversion:
        #     propulsion_model = HybridEngineSet(
        #         self._engine_wrapper.get_model(inputs), inputs["data:geometry:propulsion:engine:count"],inputs["data:geometry:propulsion:motor:count"]
        #     )
        # else:
        #     propulsion_model = TPEngineSet(self._engine_wrapper.get_model(inputs), inputs["data:geometry:propulsion:engine:count"])

        diversion_distance = inputs[
            "data:mission:" + mission_type + ":diversion:distance"
        ]
        if diversion_distance > 0:
            diversion_flight_calculator = RangedFlight(
                DiversionFlight(
                    propulsion=propulsion_model,
                    reference_area=reference_area,
                    low_speed_climb_polar=low_speed_climb_polar,
                    high_speed_polar=high_speed_polar,
                    cruise_mach=diversion_mach,
                    thrust_rates=thrust_rates,
                    electric_power_rates=diversion_electric_power_rates,
                    turbine_power_rates=diversion_turbine_power_rates,
                    climb_target_altitude=diversion_cruise_altitude * foot,
                    climb_speed=climb_speed,
                    descent_speed=[95.0],
                ),
                diversion_distance,
            )

            end_of_descent.equivalent_airspeed = climb_speed  # set manually otherwise performs climbs at end of descent speed!!
            diversion_flight_points = diversion_flight_calculator.compute_from(
                end_of_descent
            )

            # Complete flight_points with H2 and fuel cumulative masses

            delta_time = [
                diversion_flight_points.time.iloc[1]
                - diversion_flight_points.time.iloc[0]
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
            if "EB" in self.options["propulsion_id"]:
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
            top_of_diversion_climb = FlightPoint(
                diversion_flight_points[
                    (diversion_flight_points.name == "diversion climb")
                    & (
                        diversion_flight_points.altitude
                        < float(end_of_diversion_cruise.altitude)
                    )
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

            outputs[
                "data:mission:" + mission_type + ":diversion:climb:operational_ceiling"
            ] = top_of_diversion_climb.ceiling
            if end_of_diversion_cruise.altitude < diversion_cruise_altitude * foot:
                outputs[
                    "data:mission:"
                    + mission_type
                    + ":diversion:climb:operational_ceiling"
                ] = end_of_diversion_cruise.altitude
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

        else:
            end_of_diversion_descent = FlightPoint(
                mass=end_of_descent.mass,
                # true_airspeed=80,#inputs["data:mission:"+mission_type+":takeoff:V_2"],
                altitude=end_of_descent.altitude,
                ground_distance=end_of_descent.ground_distance,
                fuel_mass=0.0,  # inputs["data:mission:"+mission_type+":takeoff:fuel"],
                time=end_of_descent.time,  # inputs["data:mission:"+mission_type+":takeoff:duration"],
            )
            diversion_flight_points = flight_points[0:0]

        # Holding ==============================================================
        hybrid_holding = False

        if (
            "FHFC" in self.options["propulsion_id"]
            or "H2" in self.options["propulsion_id"]
        ):
            hybrid_holding = True

        # if hybrid_holding:
        #     propulsion_model = HybridEngineSet(
        #         self._engine_wrapper.get_model(inputs), inputs["data:geometry:propulsion:engine:count"],inputs["data:geometry:propulsion:motor:count"]
        #     )
        # else:
        #     propulsion_model = TPEngineSet(self._engine_wrapper.get_model(inputs), inputs["data:geometry:propulsion:engine:count"])

        holding_duration = inputs["data:mission:" + mission_type + ":holding:duration"]

        if holding_duration > 0:

            holding_calculator = HoldingSegment(
                target=FlightPoint(time=holding_duration),
                propulsion=propulsion_model,
                reference_area=reference_area,
                polar=high_speed_polar,
                name="holding",
                engine_setting=EngineSetting.CRUISE,
            )

            end_of_diversion_descent.equivalent_airspeed = 80.0
            atmosphere = Atmosphere(
                end_of_diversion_descent.altitude, altitude_in_feet=False
            )
            v_tas_hold = atmosphere.get_true_airspeed(
                end_of_diversion_descent.equivalent_airspeed
            )
            end_of_diversion_descent.true_airspeed = v_tas_hold
            a = atmosphere.speed_of_sound
            mach_hold = v_tas_hold / a
            end_of_diversion_descent.mach = mach_hold

            holding_flight_points = holding_calculator.compute_from(
                end_of_diversion_descent
            )
            # holding_flight_points = holding_calculator.compute_from(end_of_descent)

            # Complete flight_points with H2 and fuel cumulative masses
            delta_time = [
                holding_flight_points.time.iloc[1] - holding_flight_points.time.iloc[0]
            ] + [
                holding_flight_points.time.iloc[i + 1]
                - holding_flight_points.time.iloc[i]
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
        else:
            end_of_holding = FlightPoint(
                mass=end_of_diversion_descent.mass,
                # true_airspeed=80,#inputs["data:mission:"+mission_type+":takeoff:V_2"],
                altitude=end_of_diversion_descent.altitude,
                ground_distance=end_of_diversion_descent.ground_distance,
                fuel_mass=0.0,  # inputs["data:mission:"+mission_type+":takeoff:fuel"],
                time=end_of_diversion_descent.time,  # inputs["data:mission:"+mission_type+":takeoff:duration"],
            )
            holding_flight_points = flight_points[0:0]

        # Final ================================================================
        allowances = (
            inputs["data:mission:" + mission_type + ":takeoff:fuel"]
            + inputs["data:mission:" + mission_type + ":taxi_out:fuel"]
            + inputs["data:mission:" + mission_type + ":landing:fuel"]
            + inputs["data:mission:" + mission_type + ":taxi_in:fuel"]
        )
        allowances_time = (
            inputs["data:mission:" + mission_type + ":takeoff:duration"]
            + inputs["data:mission:" + mission_type + ":taxi_out:duration"]
            + inputs["data:mission:" + mission_type + ":landing:duration"]
            + inputs["data:mission:" + mission_type + ":taxi_in:duration"]
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

        allowances_H2 = (
            0.15 * H2_route
            + inputs["data:mission:" + mission_type + ":takeoff:H2"]
            + inputs["data:mission:" + mission_type + ":taxi_out:H2"]
            + inputs["data:mission:" + mission_type + ":landing:H2"]
            + inputs["data:mission:" + mission_type + ":taxi_in:H2"]
        )  # h2 to keep cryotemp at the end of the mission
        if inputs["data:mission:" + mission_type + ":diversion:H2_allowances"] == 1:
            diversion_allowances_H2 = outputs[
                "data:mission:" + mission_type + ":main_route:climb:H2"
            ]  # inputs["data:mission:"+mission_type+":diversion:H2_allowances"]
        else:
            diversion_allowances_H2 = 0
        # if 'FHFC' in self.options["propulsion_id"]:
        #     allowances_H2+=allowances/6

        outputs["data:mission:" + mission_type + ":trip_fuel"] = fuel_route
        outputs["data:mission:" + mission_type + ":block_fuel"] = (
            fuel_route + allowances
        )
        outputs["data:mission:" + mission_type + ":block_time"] = (
            end_of_descent.time - end_of_initial_climb.time + allowances_time
        )
        # if mission_type=='sizing':
        #     NPAX = inputs["data:TLAR:NPAX"]
        #     outputs["data:mission:sizing:block_fuel_pax"] = outputs["data:mission:"+mission_type+":block_fuel"]/NPAX
        outputs["data:mission:" + mission_type + ":ZFW"] = (
            end_of_holding.mass
            - inputs["data:mission:" + mission_type + ":landing:fuel"]
            - inputs["data:mission:" + mission_type + ":taxi_in:fuel"]
            - 0.05 * fuel_route
            - allowances_H2
            - diversion_allowances_H2
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

        if (
            "FC" in self.options["propulsion_id"]
            or "H2" in self.options["propulsion_id"]
        ):
            # outputs["data:mission:"+mission_type+":H2"] = H2_route*1.35
            # outputs["data:mission:"+mission_type+":trip_H2"]=end_of_descent.H2_mass*1.35
            outputs["data:mission:" + mission_type + ":H2"] = (
                H2_route + allowances_H2 + diversion_allowances_H2
            )
            outputs[
                "data:mission:" + mission_type + ":trip_H2"
            ] = end_of_descent.H2_mass  # + allowances_H2
        elif "EB" in self.options["propulsion_id"]:
            outputs["data:mission:" + mission_type + ":BAT_energy"] = BAT_route

        # elif 'TP' in self.options["propulsion_id"]:

        #     outputs["data:mission:"+mission_type+":trip_fuel"]=fuel_route
        #     outputs["data:mission:"+mission_type+":block_fuel"]=TOW_input - end_of_descent.mass +  inputs["data:mission:"+mission_type+":landing:fuel"]+inputs["data:mission:"+mission_type+":taxi_in:fuel"]
        #     outputs["data:mission:"+mission_type+":ZFW"] = end_of_holding.mass- inputs["data:mission:"+mission_type+":landing:fuel"] - inputs["data:mission:"+mission_type+":taxi_in:fuel"]- 0.05 * fuel_route
        #     outputs["data:mission:"+mission_type+":fuel"] = (
        #         TOW_input - outputs["data:mission:"+mission_type+":ZFW"]
        #     )

        if (
            "L1" in self.options["propulsion_id"]
            and "PH" in self.options["propulsion_id"]
        ):
            # if mission_type=='sizing':
            #     OEI_flight_points.drop(columns=['TP_total_temperature','TP_total_pressure']).applymap(lambda x: np.asscalar(np.asarray(x))).to_csv(self.options["out_file"][:-4]+'_OEI.csv')

            self.flight_points = (
                pd.concat(
                    [  # take_off_flight_points.drop(columns=['TP_total_temperature','TP_total_pressure']),
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
                [  # take_off_flight_points[['TP_total_temperature','TP_total_pressure']],
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
            # if mission_type=='sizing':
            #     OEI_flight_points.applymap(lambda x: np.asscalar(np.asarray(x))).to_csv(self.options["out_file"][:-4]+'_OEI.csv')

            self.flight_points = (
                pd.concat(
                    [  # take_off_flight_points,
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
