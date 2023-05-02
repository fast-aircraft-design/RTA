"""
Computation of wing area
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
import openmdao.api as om


class CompleteMissionOutputs(om.ExplicitComponent):
    """
    Completes propulsion inputs starting from P_non and RTO_power
    """

    def setup(self):


        self.add_input("data:TLAR:NPAX", val=1.)
        self.add_input("data:mission:sizing:block_fuel", units="kg")
        self.add_input("data:weight:aircraft:payload", units="kg")
        self.add_output("data:mission:sizing:block_fuel_pax")
        self.add_output("data:mission:sizing:block_fuel_pax_gross")


    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):     
        NPAX =inputs["data:TLAR:NPAX"]
        block_fuel =inputs["data:mission:sizing:block_fuel"]
        max_pax_gross =inputs["data:weight:aircraft:payload"]/95 #payload instead of max_payload
        
        outputs["data:mission:sizing:block_fuel_pax"] = block_fuel/NPAX     
        outputs["data:mission:sizing:block_fuel_pax_gross"] = block_fuel/max_pax_gross                


