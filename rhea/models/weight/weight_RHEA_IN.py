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

import openmdao.api as om
#from fastoad.models.options import OpenMdaoOptionDispatcherGroup
#from fastoad.models.weight.cg.cg import ComputeAircraftCG
from fastoad.module_management.service_registry import RegisterOpenMDAOSystem
from rhea.models.weight.cg.cg_RHEA_IN import CG_RHEA
from rhea.models.weight.mass_breakdown import MassBreakdown_RHEA
from fastoad.module_management.constants import ModelDomain

@RegisterOpenMDAOSystem("rhea.weight.IN", domain=ModelDomain.WEIGHT)
class Weight_RHEA_IN(om.Group):
    """
    Computes aircraft CG from CG ratio
    """
    def initialize(self):
        self.options.declare("hybrid", types=bool, default=False)
        self.options.declare("payload_from_npax", types=bool, default=False)
        self.options.declare("out_file", default="", types=str)
    def setup(self):
        
        #self.add_subsystem("aircraft", ComputeAircraftCG(), promotes=["*"])
        
        self.add_subsystem("mass_breakdown", MassBreakdown_RHEA(hybrid=self.options["hybrid"],payload_from_npax=self.options["payload_from_npax"],out_file=self.options["out_file"]), promotes=["*"])
        self.add_subsystem("cg", CG_RHEA(hybrid=self.options["hybrid"]), promotes=["*"])