"""
    Estimation of tanks center of gravity
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


class ComputeOthersCG_RHEA(ExplicitComponent):
    # TODO: Document equations. Cite sources
    """ Other components center of gravities estimation"""


    def setup(self):
        self.add_input("data:geometry:wing:MAC:at25percent:x", val=np.nan, units="m")
        self.add_input("data:geometry:wing:MAC:length", val=np.nan, units="m")
        self.add_input("data:geometry:wing:MAC:leading_edge:x:local", val=np.nan, units="m")
        self.add_input("data:geometry:wing:MAC:y", val=np.nan, units="m")
        self.add_input("data:geometry:wing:root:chord", val=np.nan, units="m")
        self.add_input("data:geometry:wing:kink:chord", val=np.nan, units="m")
        self.add_input("data:geometry:wing:root:y", val=np.nan, units="m")
        self.add_input("data:geometry:wing:kink:leading_edge:x:local", val=np.nan, units="m")
        self.add_input("data:geometry:wing:kink:y", val=np.nan, units="m")

        self.add_output("data:weight:systems:flight_controls:CG:x", units="m")
        self.add_output("data:weight:systems:de-icing:CG:x", units="m")

        self.declare_partials("data:weight:systems:flight_controls:CG:x", "*", method="fd")        
        self.declare_partials(
            "data:weight:systems:de-icing:CG:x",
            ["data:geometry:wing:MAC:at25percent:x", "data:geometry:wing:MAC:length"],
            method="fd",
        )        
        
    def compute(self, inputs, outputs):
        
        fa_length = inputs["data:geometry:wing:MAC:at25percent:x"]
        l0_wing = inputs["data:geometry:wing:MAC:length"]
        x0_wing = inputs["data:geometry:wing:MAC:leading_edge:x:local"]
        y0_wing = inputs["data:geometry:wing:MAC:y"]
        l2_wing = inputs["data:geometry:wing:root:chord"]
        l3_wing = inputs["data:geometry:wing:kink:chord"]
        y2_wing = inputs["data:geometry:wing:root:y"]
        x3_wing = inputs["data:geometry:wing:kink:leading_edge:x:local"]
        y3_wing = inputs["data:geometry:wing:kink:y"]

        x_leading_edge = x3_wing * (y0_wing - y2_wing) / (y3_wing - y2_wing)
        l_cg_control = l2_wing + (y0_wing - y2_wing) / (y3_wing - y2_wing) * (l3_wing - l2_wing)
        x_cg_control = x_leading_edge + l_cg_control
        x_cg_control_absolute = fa_length - 0.25 * l0_wing - x0_wing + x_cg_control

        x_cg_c23 = fa_length - 0.15 * l0_wing
        
        outputs["data:weight:systems:flight_controls:CG:x"] = x_cg_control_absolute        
        outputs["data:weight:systems:de-icing:CG:x"] = x_cg_c23

