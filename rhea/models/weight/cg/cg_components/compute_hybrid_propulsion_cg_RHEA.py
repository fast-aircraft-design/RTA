
"""
    Estimation of propulsion center of gravity
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


class ComputeHybridPropulsionCG_RHEA(ExplicitComponent):
    """ Propulsion center of gravity estimation as a function of wing position"""

    def __init__(self,hybrid):
        super().__init__()
        self.hybrid=hybrid
        
    def setup(self):

        self.add_output("data:weight:propulsion:electric_systems:fuel_cell:CG:x", val=10, units="m")
        self.add_output("data:weight:propulsion:electric_systems:battery:CG:x", val=10, units="m")
        self.add_output("data:weight:propulsion:electric_systems:power_electronics:CG:x", val=10, units="m")
        self.add_output("data:weight:propulsion:electric_systems:motor:CG:x", val=10, units="m")
        self.add_output("data:weight:propulsion:electric_systems:H2_storage:CG:x", val=10, units="m")

            
           





