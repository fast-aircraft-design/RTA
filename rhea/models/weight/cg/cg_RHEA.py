"""
    FAST - Copyright (c) 2016 ONERA ISAE
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

from rhea.models.weight.cg.cg_components import ComputeGlobalCG_RHEA
from fastoad.models.weight.cg.cg_components import ComputeVTcg
from fastoad.models.weight.cg.cg_components import ComputeWingCG
from fastoad.models.weight.cg.cg_components import ComputeHTcg
from rhea.models.weight.cg.cg_components import ComputeTanksCG_RHEA
from rhea.models.weight.cg.cg_components import ComputePropulsionCG_RHEA
from rhea.models.weight.cg.cg_components import  ComputeOthersCG_RHEA
from rhea.models.weight.cg.cg_components import ComputeHybridPropulsionCG_RHEA

class CG_RHEA(om.Group):
    """ Model that computes the global center of gravity """
    def __init__(self,hybrid):
        super().__init__()
        self.hybrid=hybrid
        
    def setup(self):
        self.add_subsystem("ht_cg", ComputeHTcg(), promotes=["*"])
        self.add_subsystem("vt_cg", ComputeVTcg(), promotes=["*"])
        self.add_subsystem("compute_cg_wing", ComputeWingCG(), promotes=["*"])
        self.add_subsystem("compute_cg_tanks", ComputeTanksCG_RHEA(), promotes=["*"])       
        self.add_subsystem("compute_cg_propulsion", ComputePropulsionCG_RHEA(), promotes=["*"])
        self.add_subsystem("compute_cg_hybrid_propulsion", ComputeHybridPropulsionCG_RHEA(self.hybrid), promotes=["*"])
        self.add_subsystem("compute_cg_others", ComputeOthersCG_RHEA(), promotes=["*"])        
        self.add_subsystem("compute_cg", ComputeGlobalCG_RHEA(), promotes=["*"])
        self.add_subsystem("aircraft", ComputeAircraftCG(), promotes=["*"])



class ComputeAircraftCG(om.ExplicitComponent):
    """ Compute position of aircraft CG from CG ratio """

    def setup(self):
        self.add_input("data:weight:aircraft:CG:aft:MAC_position", val=np.nan)
        self.add_input("data:geometry:wing:MAC:at25percent:x", val=np.nan, units="m")
        self.add_input("data:geometry:wing:MAC:length", val=np.nan, units="m")

        self.add_output("data:weight:aircraft:CG:aft:x", units="m")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs):
        cg_ratio = inputs["data:weight:aircraft:CG:aft:MAC_position"]
        l0_wing = inputs["data:geometry:wing:MAC:length"]
        mac_position = inputs["data:geometry:wing:MAC:at25percent:x"]

        outputs["data:weight:aircraft:CG:aft:x"] = (
            mac_position - 0.25 * l0_wing + cg_ratio * l0_wing
        )
