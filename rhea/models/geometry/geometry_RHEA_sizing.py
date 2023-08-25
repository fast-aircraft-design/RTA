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
from fastoad.module_management.service_registry import RegisterOpenMDAOSystem, RegisterSubmodel
from fastoad_cs25.models.geometry.compute_aero_center import ComputeAeroCenter
# from fastoad_cs25.models.geometry.geom_components import ComputeTotalArea
# from fastoad_cs25.models.geometry.geom_components.vt import ComputeVerticalTailGeometry
# from fastoad_cs25.models.geometry.geom_components.ht import ComputeHorizontalTailGeometry

from fastoad_cs25.models.geometry.constants import (
    SERVICE_AIRCRAFT_AERODYNAMIC_CENTER,
    SERVICE_AIRCRAFT_WETTED_AREA,
    SERVICE_FUSELAGE_GEOMETRY_BASIC,
    SERVICE_FUSELAGE_GEOMETRY_WITH_CABIN_SIZING,
    SERVICE_HORIZONTAL_TAIL_GEOMETRY,
    SERVICE_NACELLE_PYLON_GEOMETRY,
    SERVICE_VERTICAL_TAIL_GEOMETRY,
    SERVICE_WING_GEOMETRY,
)
from fastoad.module_management.constants import ModelDomain
# from fastoad_cs25.models.geometry.geom_components.fuselage import ComputeCnBetaFuselage
# from rhea.models.geometry.geom_components import ComputeGeometry_RHEA

from .geom_components.fuselage.compute_fuselage import (
    ComputeFuselageGeometryBasic,
    ComputeFuselageGeometryCabinSizing,
)

from .geom_components.nacelle.compute_nacelle import (
    ComputeNacelleGeometry,
)

from .geom_components.wing.compute_wing_RHEA import ComputeWingGeometry_RHEA
from fastoad_cs25.models.constants import CABIN_SIZING_OPTION


@RegisterOpenMDAOSystem("rhea.geometry.sizing", domain=ModelDomain.GEOMETRY)
class Geometry_sizing(om.Group):
    """
    Computes geometric characteristics of the (tube-wing) aircraft:
      - fuselage size is computed from payload requirements
      - wing dimensions are computed from global parameters (area, taper ratio...)
      - tail planes are dimensioned from HQ requirements

    This module also computes centers of gravity and static margin
    """

    def initialize(self):
        self.options.declare(CABIN_SIZING_OPTION, types=bool, default=True)

    def setup(self):

        if self.options[CABIN_SIZING_OPTION]:
            self.add_subsystem(
                "compute_fuselage", ComputeFuselageGeometryCabinSizing(), promotes=["*"]
            )
        else:
            self.add_subsystem("compute_fuselage", ComputeFuselageGeometryBasic(), promotes=["*"])

        self.add_subsystem("compute_wing", ComputeWingGeometry_RHEA(), promotes=["*"])
        self.add_subsystem(
            "compute_engine_nacelle", ComputeNacelleGeometry(), promotes=["*"]
        )
        self.add_subsystem("compute_ht",
                           RegisterSubmodel.get_submodel(SERVICE_HORIZONTAL_TAIL_GEOMETRY), promotes=["*"])

        self.add_subsystem("compute_vt", RegisterSubmodel.get_submodel(SERVICE_VERTICAL_TAIL_GEOMETRY), promotes=["*"])
        self.add_subsystem("compute_total_area",
                           RegisterSubmodel.get_submodel(SERVICE_AIRCRAFT_WETTED_AREA), promotes=["*"])

        self.add_subsystem("compute_aero_center", ComputeAeroCenter(), promotes=["*"])
