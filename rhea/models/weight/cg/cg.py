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
from fastoad.module_management.service_registry import RegisterSubmodel
from .cg_components import ComputeFlightControlCG
from .cg_components import ComputeGlobalCG
# from fastoad_cs25.models.weight.cg.cg_components import ComputeVTcg
# from fastoad_cs25.models.weight.cg.cg_components import ComputeWingCG
# from fastoad_cs25.models.weight.cg.cg_components import ComputeHTcg
from .cg_components import ComputeTanksCG_RHEA
from .cg_components import ComputePropulsionCG_RHEA
from .cg_components import ComputeOthersCG
# from fastoad_cs25.models.weight.cg.cg_components import UpdateMLG
from fastoad_cs25.models.weight.cg.constants import (
    SERVICE_AIRCRAFT_CG,
    SERVICE_FLIGHT_CONTROLS_CG,
    SERVICE_GLOBAL_CG,
    SERVICE_HORIZONTAL_TAIL_CG,
    SERVICE_MLG_CG,
    SERVICE_OTHERS_CG,
    SERVICE_TANKS_CG,
    SERVICE_VERTICAL_TAIL_CG,
    SERVICE_WING_CG,
)


class CG(om.Group):
    """ Model that computes the global center of gravity """
    def __init__(self,hybrid):
        super().__init__()
        self.hybrid=hybrid
        
    def setup(self):
        self.add_subsystem("ht_cg", RegisterSubmodel.get_submodel(SERVICE_HORIZONTAL_TAIL_CG), promotes=["*"])
        self.add_subsystem("vt_cg", RegisterSubmodel.get_submodel(SERVICE_VERTICAL_TAIL_CG), promotes=["*"])
        self.add_subsystem("compute_cg_wing", RegisterSubmodel.get_submodel(SERVICE_WING_CG), promotes=["*"])
        self.add_subsystem("compute_cg_flight_controls", ComputeFlightControlCG(), promotes=["*"])        
        self.add_subsystem("compute_cg_tanks", ComputeTanksCG_RHEA(), promotes=["*"])       
        self.add_subsystem("compute_cg_propulsion", ComputePropulsionCG_RHEA(), promotes=["*"])
        self.add_subsystem("compute_cg_others", ComputeOthersCG(), promotes=["*"])        
        self.add_subsystem("compute_cg", ComputeGlobalCG(), promotes=["*"])
        self.add_subsystem("update_mlg", RegisterSubmodel.get_submodel(SERVICE_MLG_CG), promotes=["*"])

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
