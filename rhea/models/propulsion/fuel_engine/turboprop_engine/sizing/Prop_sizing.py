"""
Weight computation (mass and CG)
"""
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
from openmdao.core.explicitcomponent import ExplicitComponent
from scipy import constants
from fastoad.module_management.constants import ModelDomain
from fastoad.module_management.service_registry import RegisterOpenMDAOSystem


@RegisterOpenMDAOSystem(
    "rhea.propulsion.propeller_sizing", domain=ModelDomain.PROPULSION
)
class Prop_sizing(ExplicitComponent):
    """
    Performs sizing of the propeller based on input max power and disk loading.
    """

    def setup(self):
        self.add_input(
            "data:propulsion:propeller:disk_loading", np.nan, units="kW/m**2"
        )
        self.add_input("data:propulsion:propeller:max_power", np.nan, units="kW")

        self.add_output("data:geometry:propulsion:propeller:diameter", units="m")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs):
        disk_loading = inputs["data:propulsion:propeller:disk_loading"]
        max_power = inputs["data:propulsion:propeller:max_power"]

        # evaluate propeller diameter
        d = (4 * max_power / (constants.pi * disk_loading)) ** 0.5

        # #evaluate max rotational speed
        # altitude =0
        # mach=0
        # atmosphere = Atmosphere(altitude, altitude_in_feet=False)
        # a = atmosphere.speed_of_sound
        # omega= 2*a/d*((0.7**2-mach**2)**0.5) #rad/s
        # rpm_max=60/(2*constants.pi)*omega

        outputs["data:geometry:propulsion:propeller:diameter"] = d
