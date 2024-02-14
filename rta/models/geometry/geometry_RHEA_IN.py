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

import openmdao.api as om

from fastoad_cs25.models.geometry.compute_aero_center import ComputeAeroCenter
from fastoad_cs25.models.geometry.geom_components.compute_wetted_area import (
    ComputeWettedArea,
)
from fastoad_cs25.models.geometry.geom_components.fuselage.compute_cnbeta_fuselage import (
    ComputeCnBetaFuselage,
)
from rta.models.geometry.geom_components import ComputeGeometry_RHEA
from fastoad.module_management.constants import ModelDomain
from fastoad.module_management.service_registry import (
    RegisterOpenMDAOSystem,
    RegisterSubmodel,
)


@RegisterOpenMDAOSystem("rta.geometry.RHEA_IN", domain=ModelDomain.GEOMETRY)
class Geometry_IN(om.Group):
    """
    This module computes total aircraft wet area, cnbetafuselage and aero center.
    """

    def setup(self):
        self.add_subsystem("geometry", ComputeGeometry_RHEA(), promotes=["*"])
        self.add_subsystem("fuselage_cnbeta", ComputeCnBetaFuselage(), promotes=["*"])
        self.add_subsystem("compute_total_area", ComputeWettedArea(), promotes=["*"])
        self.add_subsystem("compute_aero_center", ComputeAeroCenter(), promotes=["*"])
