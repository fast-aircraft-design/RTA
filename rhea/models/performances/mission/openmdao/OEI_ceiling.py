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
from fastoad import BundleLoader
from rhea.models.aerodynamics.constants import (
    POLAR_POINT_COUNT,
    CT_POINT_COUNT,
    ALPHA_POINT_COUNT,
    H_POINT_COUNT,
)
from rhea.models.propulsion.fuel_engine.constants import POWER_RATE_COUNT


# from fastoad.models.propulsion import EngineSet
from models.propulsion.fuel_engine.turboprop_engine.base import (
    TPEngineSet,
    TPH2EngineSet,
)
from models.propulsion.fuel_engine.hybrid_engine.base import HybridEngineSet

from scipy.constants import foot, nautical_mile, knot, hour, g

from scipy.optimize import fsolve

# from fastoad.models.performances.mission.flight_point import FlightPoint #v0.50a
from fastoad.models.performances.mission.polar import Polar
from fastoad.utils.physics import Atmosphere

from models.propulsion.fuel_engine.turboprop_engine.engine_components.Propeller import (
    Propeller,
)


class OEI_ceiling(om.ExplicitComponent):
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
        self.options.declare("prop_fid", default="", types=str)

    def setup(self):
        self._engine_wrapper = BundleLoader().instantiate_component(
            self.options["propulsion_id"]
        )
        self._engine_wrapper.setup(self)

        # Inputs -----------------------------------------------------------------------------------

        self.add_input("data:geometry:wing:area", np.nan, units="m**2")

        self.add_input(
            "data:aerodynamics:aircraft:cruise:CL", np.nan, shape=POLAR_POINT_COUNT
        )
        self.add_input(
            "data:aerodynamics:aircraft:cruise:CD", np.nan, shape=POLAR_POINT_COUNT
        )
        self.add_input(
            "data:mission:sizing:climb:EM_power_rate", 0.0, shape=POWER_RATE_COUNT
        )

        # self.add_input("data:aerodynamics:aircraft:low_speed:DCD_ext", np.nan)
        # self.add_input("data:aerodynamics:aircraft:low_speed:DCD_feather", np.nan)
        self.add_input(
            "data:aerodynamics:aircraft:low_speed:CT", np.nan, shape=CT_POINT_COUNT
        )
        self.add_input(
            "data:aerodynamics:aircraft:low_speed:OEI_effect:DCD",
            np.nan,
            copy_shape="data:aerodynamics:aircraft:low_speed:CT",
        )

        self.add_input("data:weight:aircraft:MTOW", np.nan, units="kg")
        self.add_input("data:geometry:propulsion:engine:count", np.nan)

        # Outputs ----------------------------------------------------------------------------------
        self.add_output("data:mission:sizing:OEI:net_ceiling", units="m", val=0.0)
        self.add_output("data:mission:sizing:OEI:V_TAS", units="m/s", val=0.0)
        self.add_output("data:mission:sizing:OEI:mass", units="kg", val=0.0)

        self.declare_partials(["*"], ["*"])

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):

        if (
            "PH" in self.options["propulsion_id"]
            or "FH" in self.options["propulsion_id"]
        ):
            propulsion_model_OEI = HybridEngineSet(
                self._engine_wrapper.get_model(inputs),
                inputs["data:geometry:propulsion:engine:count"] - 1,
            )
        elif "PW100_H2" in self.options["propulsion_id"]:
            propulsion_model_OEI = TPH2EngineSet(
                self._engine_wrapper.get_model(inputs),
                inputs["data:geometry:propulsion:engine:count"] - 1,
            )

        else:
            propulsion_model_OEI = TPEngineSet(
                self._engine_wrapper.get_model(inputs),
                inputs["data:geometry:propulsion:engine:count"] - 1,
            )

        reference_area = inputs["data:geometry:wing:area"]
        DCd_OEI = inputs["data:aerodynamics:aircraft:low_speed:OEI_effect:DCD"]
        CT_list = inputs["data:aerodynamics:aircraft:low_speed:CT"]

        self.em_power_rate = min(
            inputs["data:mission:sizing:climb:EM_power_rate"].min() * 2, 1
        )
        # EM_power_rate =

        high_speed_polar = Polar(
            inputs["data:aerodynamics:aircraft:cruise:CL"],
            inputs["data:aerodynamics:aircraft:cruise:CD"],
        )

        mass = inputs["data:weight:aircraft:MTOW"] - 300

        def func(alt, m):
            weight = m * 9.81
            atmosphere = Atmosphere(alt, altitude_in_feet=True)
            density = atmosphere.density

            V_TAS = self.find_best_climb_speed(
                high_speed_polar, reference_area, atmosphere, weight
            )

            mach = V_TAS / atmosphere.speed_of_sound
            CL = weight / (0.5 * density * reference_area * V_TAS**2)
            FN = self.evaluate_thrust(
                propulsion_model_OEI.engine, atmosphere, mach
            )  # *1.04 #altitude in feet
            CT = FN / (0.5 * density * reference_area * V_TAS**2)

            dCd_OEI = np.interp(CT, CT_list, DCd_OEI)

            ceiling = FN / weight - (high_speed_polar.cd(CL) + dCd_OEI) / CL - 0.011

            # print(alt,V_TAS,CL,high_speed_polar.cd(CL),dCd_OEI,FN,mach,CT)
            return ceiling

        ceiling = fsolve(
            func, 10000.0, args=mass
        )  # inputs["data:weight:aircraft:MTOW"])
        V_TAS = self.find_best_climb_speed(
            high_speed_polar,
            reference_area,
            Atmosphere(ceiling, altitude_in_feet=True),
            inputs["data:weight:aircraft:MTOW"] * 9.81,
        )

        outputs["data:mission:sizing:OEI:net_ceiling"] = ceiling * foot
        outputs["data:mission:sizing:OEI:V_TAS"] = V_TAS
        outputs["data:mission:sizing:OEI:mass"] = mass

    def evaluate_thrust(self, propulsion_model_OEI, atmosphere, mach):
        phase = 9.0

        if (
            "PH" in self.options["propulsion_id"]
            or "FH" in self.options["propulsion_id"]
        ):
            # max_shaft_power,max_thermo_power,max_motor_power,max_available_power,gearbox_limit_power,max_TPshaft_power  = propulsion_model_OEI.max_power(atmosphere, mach, phase)
            # max_thrust, eta = Propeller().select('power_to_thrust',self.options["prop_fid"],propulsion_model_OEI,atmosphere, mach,phase, max_shaft_power)
            # max_TPthrust, eta =Propeller().select('power_to_thrust',self.options["prop_fid"],propulsion_model_OEI,atmosphere, mach,phase, max_TPshaft_power)
            # FR = propulsion_model_OEI.turbine.compute_engine_point(atmosphere, mach, phase,T_prop=max_TPthrust,shaft_power=max_TPshaft_power)
            motor_power = (
                propulsion_model_OEI.Elec_nom_power * self.em_power_rate
            )  # -->diversion climb is the sizing point(No H2 available)
            propulsion_model_OEI = propulsion_model_OEI.turbine
        else:
            motor_power = 0

        (
            max_shaft_power,
            max_thermo_power,
            gearbox_limit_power,
        ) = propulsion_model_OEI.max_power(atmosphere, mach, phase)
        tot_shaft_power = max_shaft_power + motor_power
        max_thrust, eta = Propeller().select(
            "power_to_thrust",
            self.options["prop_fid"],
            propulsion_model_OEI,
            atmosphere,
            mach,
            phase,
            tot_shaft_power,
        )
        FR = propulsion_model_OEI.compute_engine_point(
            atmosphere, mach, phase, T_prop=max_thrust, shaft_power=max_shaft_power
        )

        FN = FR + max_thrust
        return FN

    def find_best_climb_speed(
        self, high_speed_polar, reference_area, atmosphere, weight
    ):
        CL = high_speed_polar.optimal_cl
        density = atmosphere.density
        V_opt = (2 * weight / (CL * reference_area * density)) ** 0.5

        return V_opt
