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
from rhea.models.propulsion.fuel_engine.constants import POWER_RATE_COUNT

from fastoad import BundleLoader
from rhea.models.aerodynamics.constants import (
    POLAR_POINT_COUNT,
    CT_POINT_COUNT,
    ALPHA_POINT_COUNT,
    H_POINT_COUNT,
)
from scipy.optimize import root_scalar

from models.performances.mission.openmdao.regional_flight import SizingFlight_RHEA


class DOCFlight_RHEA(SizingFlight_RHEA):
    """
    Simulates a complete flight operational mission with diversion.
    """

    def setup(self):
        self._engine_wrapper = BundleLoader().instantiate_component(
            self.options["propulsion_id"]
        )
        self._engine_wrapper.setup(self)

        # Inputs -----------------------------------------------------------------------------------
        self.add_input("data:mission:DOC:cruise:mach", np.nan)
        self.add_input("data:mission:DOC:range", np.nan, units="m")
        self.add_input("data:mission:DOC:payload", np.nan, units="kg")
        self.add_input("data:weight:aircraft:OWE", np.nan, units="kg")
        self.add_input("data:mission:DOC:DISA", 0)

        # if 'PHFC' in self.options["propulsion_id"]:
        #     self.add_input("data:geometry:propulsion:motor:count",  np.nan)

        self.add_input("data:geometry:propulsion:engine:count", 2)
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
            np.nan,
            copy_shape="data:aerodynamics:aircraft:low_speed:CT",
        )
        self.add_input(
            "data:aerodynamics:aircraft:takeoff:CT_effect:DCL_alpha",
            np.nan,
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
            np.nan,
            copy_shape="data:aerodynamics:aircraft:low_speed:alpha",
        )
        self.add_input(
            "data:aerodynamics:aircraft:takeoff:ground_effect:DCL",
            np.nan,
            copy_shape="data:aerodynamics:aircraft:low_speed:alpha",
        )
        self.add_input(
            "data:aerodynamics:aircraft:takeoff:ground_effect:K_H",
            np.nan,
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

        self.add_input("data:mission:sizing:takeoff:V_MCA", 0, units="m/s")
        self.add_input("data:mission:sizing:takeoff:Friction_coefficient_no_brake", 0)
        self.add_input("data:mission:sizing:takeoff:thrust_rate", 0)

        self.add_input("data:mission:DOC:takeoff:EM_power_rate", 0.0)
        self.add_input("data:mission:DOC:takeoff:TP_power_rate", 1.0)
        self.add_input("data:mission:DOC:initial_climb:EM_power_rate", 0.0)
        self.add_input("data:mission:DOC:initial_climb:TP_power_rate", 1.0)
        self.add_input(
            "data:mission:DOC:climb:EM_power_rate", 0.0, shape=POWER_RATE_COUNT
        )
        self.add_input(
            "data:mission:DOC:climb:TP_power_rate", 1.0, shape=POWER_RATE_COUNT
        )
        self.add_input("data:mission:DOC:cruise:EM_power_rate", 0.0)
        self.add_input("data:mission:DOC:cruise:TP_power_rate", 1.0)

        self.add_input("data:mission:DOC:diversion:climb:EM_power_rate", 0.0)
        self.add_input("data:mission:DOC:diversion:climb:TP_power_rate", 1.0)
        self.add_input("data:mission:DOC:diversion:cruise:EM_power_rate", 0.0)
        self.add_input("data:mission:DOC:diversion:cruise:TP_power_rate", 1.0)
        self.add_input("data:mission:DOC:diversion:H2_allowances", 0.0)

        self.add_input("data:weight:aircraft:MTOW", np.nan, units="kg")

        self.add_input("data:mission:DOC:taxi_out:fuel", np.nan, units="kg")
        self.add_input("data:mission:DOC:taxi_out:H2", 0.0, units="kg")
        self.add_input("data:mission:DOC:taxi_out:duration", 360.0, units="s")
        self.add_input("data:mission:DOC:takeoff:duration", 60.0, units="s")
        self.add_input("data:mission:DOC:landing:duration", 120.0, units="s")
        self.add_input("data:mission:DOC:taxi_in:duration", 240.0, units="s")
        # self.add_input("data:mission:DOC:takeoff:V2", np.nan, units="m/s")

        self.add_input("data:mission:DOC:takeoff:altitude", np.nan, units="m")
        self.add_input("data:mission:DOC:takeoff:fuel", np.nan, units="kg")
        self.add_input("data:mission:DOC:takeoff:H2", 0.0, units="kg")

        self.add_input("data:mission:DOC:climb:thrust_rate", np.nan)
        self.add_input("data:mission:DOC:climb:speed", np.nan, units="m/s")
        self.add_input(
            "data:mission:DOC:main_route:cruise:altitude", np.nan, units="ft"
        )
        self.add_input("data:mission:DOC:descent:thrust_rate", np.nan)
        self.add_input("data:mission:DOC:descent:speed", np.nan, units="m/s")
        # self.add_input("data:mission:DOC:takeoff:V_2", units="m/s",val=60.)
        # self.add_input("data:mission:DOC:takeoff:duration", units="s",val=60.)

        self.add_input("data:mission:DOC:diversion:mach", np.nan)
        self.add_input("data:mission:DOC:diversion:distance", np.nan, units="m")
        self.add_input("data:mission:DOC:diversion:altitude", np.nan, units="ft")
        self.add_input("data:mission:DOC:holding:duration", np.nan, units="s")

        self.add_input("data:mission:DOC:landing:fuel", np.nan, units="kg")
        self.add_input("data:mission:DOC:taxi_in:fuel", np.nan, units="kg")
        self.add_input("data:mission:DOC:landing:H2", 0.0, units="kg")
        self.add_input("data:mission:DOC:taxi_in:H2", 0.0, units="kg")

        self.add_input("data:mission:DOC:taxi_in:speed", np.nan, units="m/s")
        self.add_input("data:mission:DOC:taxi_in:thrust_rate", np.nan)

        # Outputs ----------------------------------------------------------------------------------
        self.add_output("data:mission:DOC:TOW", units="kg")
        self.add_output("data:mission:DOC:takeoff:distance", units="m")
        self.add_output("data:mission:DOC:takeoff:V_rotate", units="m/s")
        self.add_output("data:mission:DOC:takeoff:V_liftoff", units="m/s")
        self.add_output("data:mission:DOC:takeoff:V_2", units="m/s")

        self.add_output("data:mission:DOC:climb:operational_ceiling", 0, units="m")
        self.add_output(
            "data:mission:DOC:diversion:climb:operational_ceiling", 0, units="m"
        )

        self.add_output("data:mission:DOC:cruise:max_mach")
        self.add_output("data:mission:DOC:initial_climb:fuel", units="kg")
        self.add_output("data:mission:DOC:initial_climb:H2", units="kg", val=0)
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:H2_flow_rate",
            units="kg/s",
            val=0,
        )

        self.add_output("data:mission:DOC:main_route:climb:fuel", units="kg")
        self.add_output("data:mission:DOC:main_route:cruise:fuel", units="kg")
        self.add_output("data:mission:DOC:main_route:descent:fuel", units="kg")
        self.add_output("data:mission:DOC:main_route:climb:H2", units="kg", val=0.0)
        self.add_output("data:mission:DOC:main_route:cruise:H2", units="kg", val=0.0)
        self.add_output("data:mission:DOC:main_route:descent:H2", units="kg", val=0.0)

        self.add_output("data:mission:DOC:initial_climb:distance", units="m")
        self.add_output("data:mission:DOC:main_route:climb:distance", units="m")
        self.add_output("data:mission:DOC:main_route:cruise:distance", units="m")
        self.add_output("data:mission:DOC:main_route:descent:distance", units="m")

        self.add_output("data:mission:DOC:initial_climb:duration", units="s")
        self.add_output("data:mission:DOC:main_route:climb:duration", units="s")
        self.add_output("data:mission:DOC:main_route:climb:TTC", units="s")
        self.add_output("data:mission:DOC:main_route:cruise:duration", units="s")
        self.add_output("data:mission:DOC:main_route:descent:duration", units="s")

        self.add_output("data:mission:DOC:diversion:climb:fuel", units="kg", val=0.0)
        self.add_output("data:mission:DOC:diversion:cruise:fuel", units="kg", val=0.0)
        self.add_output("data:mission:DOC:diversion:descent:fuel", units="kg", val=0.0)
        self.add_output("data:mission:DOC:diversion:climb:H2", units="kg", val=0.0)
        self.add_output("data:mission:DOC:diversion:cruise:H2", units="kg", val=0.0)
        self.add_output("data:mission:DOC:diversion:descent:H2", units="kg", val=0.0)

        self.add_output("data:mission:DOC:diversion:climb:distance", units="m", val=0.0)
        self.add_output(
            "data:mission:DOC:diversion:cruise:distance", units="m", val=0.0
        )
        self.add_output(
            "data:mission:DOC:diversion:descent:distance", units="m", val=0.0
        )

        self.add_output("data:mission:DOC:diversion:climb:duration", units="s", val=0.0)
        self.add_output(
            "data:mission:DOC:diversion:cruise:duration", units="s", val=0.0
        )
        self.add_output(
            "data:mission:DOC:diversion:descent:duration", units="s", val=0.0
        )

        self.add_output("data:mission:DOC:holding:fuel", units="kg")
        self.add_output("data:mission:DOC:holding:H2", units="kg", val=0.0)

        # self.add_output("data:mission:DOC:taxi_in:fuel", units="kg")

        self.add_output("data:mission:DOC:ZFW", units="kg")
        self.add_output("data:mission:DOC:fuel", units="kg")
        self.add_output("data:mission:DOC:H2", units="kg", val=0.0)
        self.add_output("data:mission:DOC:BAT_energy", units="J", val=0.0)
        self.add_output("data:mission:DOC:trip_fuel", units="kg")
        self.add_output("data:mission:DOC:trip_H2", units="kg", val=0.0)
        self.add_output("data:mission:DOC:block_fuel", units="kg")
        self.add_output("data:mission:DOC:block_time", units="s")
        self.add_output("mdao:conv", val=np.nan)

        self.declare_partials(["*"], ["*"])

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        try:
            tol = 10e-5
            payload = inputs["data:mission:DOC:payload"]
            OWE = inputs["data:weight:aircraft:OWE"]
            ZFW_target = OWE + payload
            mission_type = "DOC"
            TOW_input = inputs["data:weight:aircraft:MTOW"]

            def compute_point(TOW_input):
                loop_inputs = TOW_input
                self.compute_mission(inputs, outputs, loop_inputs, mission_type)
                return ZFW_target - outputs["data:mission:DOC:ZFW"]

            sol = root_scalar(
                compute_point,
                x0=0.9 * TOW_input,
                x1=0.7 * TOW_input,
                rtol=tol,
                method="secant",
            )
            outputs["data:mission:DOC:TOW"] = sol.root
        except IndexError:
            self.compute_breguet(inputs, outputs, TOW_input, mission_type)

    """def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        try:
            delta_ZFW=100.
            tol = 1e-1
            TOW_input = inputs["data:weight:aircraft:MTOW"]
            mission_type= 'DOC'
            while abs(delta_ZFW)>tol:
                
                self.compute_mission(inputs, outputs,TOW_input,mission_type)
                
                delta_ZFW,TOW_input=self.compute_residuals(inputs, outputs)
            outputs["data:mission:DOC:TOW"]=TOW_input

        except IndexError:
            self.compute_breguet(inputs, outputs,TOW_input)
        
       
        
    def compute_residuals(self, inputs, outputs):        
        payload=inputs["data:mission:DOC:payload"]
        OWE=inputs["data:weight:aircraft:OWE"]        
        delta_ZFW=payload+OWE-outputs["data:mission:DOC:ZFW"] 
        new_TOW= payload+OWE+outputs["data:mission:DOC:fuel"]
        return delta_ZFW,new_TOW"""
