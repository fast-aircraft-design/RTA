"""
Estimation of communication systems weight
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



class CommunicationSystemWeight(ExplicitComponent):
    """
    Weight estimation for communication systems

    This includes:

    - Alarms
    - Interphone
    - Flight recorder
    - Transceiver VHF/COMM

    """

    def setup(self):

        self.add_input("tuning:weight:systems:communications:mass:k", val=1.0)
        self.add_input("tuning:weight:systems:communications:mass:offset", val=0.0, units="kg")

        self.add_output("data:weight:systems:communications:mass", units="kg")

        self.declare_partials("*", "*", method="fd")

    # pylint: disable=too-many-locals
    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        k = inputs["tuning:weight:systems:communications:mass:k"]
        offset = inputs["tuning:weight:systems:communications:mass:offset"]

        # Mass of communication system
        mass_comm = 80 
        outputs["data:weight:systems:communications:mass"] = k * mass_comm + offset
