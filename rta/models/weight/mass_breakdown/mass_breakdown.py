"""Main components for mass breakdown."""
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

from fastoad_cs25.models.constants import PAYLOAD_FROM_NPAX

from .a_airframe import WingWeight, NacellesWeight

from fastoad_cs25.models.weight.mass_breakdown.a_airframe import (
    FuselageWeight,
    EmpennageWeight,
    # FlightControlsWeight,
    LandingGearWeight,
    # PylonsWeight,
    # PaintWeight,
)
from .b_propulsion.fuel_lines_weight import (
    FuelLinesWeight,
)
from .c_systems import (
    ECSWeight,
    AutoFlightSystemWeight,
    CommunicationSystemWeight,
    ElectricalPowerSystemWeight,
    FlightFurnishingWeight,
    FireSystemWeight,
    FlightControlsSystemWeight,
    HydraulicPowerSystemWeight,
    DeiceSystemWeight,
    NavigationSystemWeight,
    APUWeight,
)
from fastoad_cs25.models.weight.mass_breakdown.cs25 import Loads
from .d_furniture import (
    LightsWeight,
    WaterWeight,
    OxygenWeight,
    FurnishingWeight,
    InsulationWeight,
    SeatsCrewWeight,
    InteriorIntegrationWeight,
)

from .e_operational import OperationalEquipmentsWeight, OperationalItemsWeight

from fastoad_cs25.models.weight.mass_breakdown.e_crew import CrewWeight
from fastoad_cs25.models.weight.mass_breakdown.payload import ComputePayload
from fastoad_cs25.models.weight.mass_breakdown.update_mlw_and_mzfw import (
    UpdateMLWandMZFW,
)
from .b_propulsion.turboprop_weight import TurbopropWeight


class MTOWComputation(om.AddSubtractComp):
    """
    Computes MTOW from OWE, design payload and consumed fuel in sizing mission.
    """

    def setup(self):
        self.add_equation(
            "data:weight:aircraft:MTOW",
            [
                "data:weight:aircraft:OWE",
                "data:weight:aircraft:payload",
                "data:mission:sizing:fuel",
            ],
            units="kg",
        )


class MassBreakdown(om.Group):
    """
    Computes analytically the mass of each part of the aircraft, and the resulting sum,
    the Overall Weight Empty (OWE).

    Some models depend on MZFW (Max Zero Fuel Weight), MLW (Max Landing Weight) and
    MTOW (Max TakeOff Weight), which depend on OWE.

    This model cycles for having consistent OWE, MZFW and MLW.

    Options:
    - payload_from_npax: If True (default), payload masses will be computed from NPAX, if False
                         design payload mass and maximum payload mass must be provided.
    """

    def initialize(self):
        self.options.declare(PAYLOAD_FROM_NPAX, types=bool, default=True)

    def setup(self):
        if self.options[PAYLOAD_FROM_NPAX]:
            self.add_subsystem("payload", ComputePayload(), promotes=["*"])
        self.add_subsystem("owe", OperatingWeightEmpty(), promotes=["*"])
        self.add_subsystem("update_mzfw_and_mlw", UpdateMLWandMZFW(), promotes=["*"])

        # Solvers setup
        self.nonlinear_solver = om.NonlinearBlockGS()
        self.nonlinear_solver.options["iprint"] = 0
        self.nonlinear_solver.options["maxiter"] = 50
        # self.nonlinear_solver.options["rtol"] = 1e-2

        self.linear_solver = om.LinearBlockGS()
        self.linear_solver.options["iprint"] = 0
        # self.linear_solver.options["rtol"] = 1e-2


class AirframeWeight(om.Group):
    """
    Computes mass of airframe.
    """

    def setup(self):
        # Airframe
        self.add_subsystem("loads", Loads(), promotes=["*"])
        self.add_subsystem("ATA57", WingWeight(), promotes=["*"])
        self.add_subsystem("ATA53", FuselageWeight(), promotes=["*"])
        self.add_subsystem("ATA55", EmpennageWeight(), promotes=["*"])
        self.add_subsystem("ATA32", LandingGearWeight(), promotes=["*"])
        self.add_subsystem("ATA54", NacellesWeight(), promotes=["*"])

        weight_sum = om.AddSubtractComp()
        weight_sum.add_equation(
            "data:weight:airframe:mass",
            [
                "data:weight:airframe:wing:mass",
                "data:weight:airframe:fuselage:mass",
                "data:weight:airframe:horizontal_tail:mass",
                "data:weight:airframe:vertical_tail:mass",
                "data:weight:airframe:landing_gear:main:mass",
                "data:weight:airframe:landing_gear:front:mass",
                "data:weight:airframe:nacelle_struts:mass",
            ],
            units="kg",
            desc="Mass of airframe",
        )

        self.add_subsystem(
            "airframe_weight_sum",
            weight_sum,
            promotes=["*"],
        )


class PropulsionWeight(om.Group):
    """
    Computes mass of propulsion.
    """

    def setup(self):
        # Engine have to be computed before pylons
        self.add_subsystem("ATA61_72_73", TurbopropWeight(), promotes=["*"])
        self.add_subsystem("ATA28", FuelLinesWeight(), promotes=["*"])

        # self.add_subsystem("unconsumables_weight", UnconsumablesWeight(), promotes=["*"])

        weight_sum = om.AddSubtractComp()
        weight_sum.add_equation(
            "data:weight:propulsion:mass",
            [
                "data:weight:propulsion:engine:mass",
                "data:weight:propulsion:propeller:mass",
                "data:weight:propulsion:engine_controls_instrumentation:mass",
                "data:weight:propulsion:fuel_system:mass",
            ],
            units="kg",
            desc="Mass of the propulsion system",
        )

        self.add_subsystem(
            "propulsion_weight_sum",
            weight_sum,
            promotes=["*"],
        )


class SystemsWeight(om.Group):
    """
    Computes mass of systems.
    """

    def setup(self):
        self.add_subsystem("ATA21", ECSWeight(), promotes=["*"])

        self.add_subsystem("ATA22", AutoFlightSystemWeight(), promotes=["*"])
        self.add_subsystem("ATA23", CommunicationSystemWeight(), promotes=["*"])

        self.add_subsystem("ATA24", ElectricalPowerSystemWeight(), promotes=["*"])
        self.add_subsystem("ATA2510", FlightFurnishingWeight(), promotes=["*"])

        self.add_subsystem("ATA26", FireSystemWeight(), promotes=["*"])
        self.add_subsystem("ATA27", FlightControlsSystemWeight(), promotes=["*"])

        self.add_subsystem("ATA29", HydraulicPowerSystemWeight(), promotes=["*"])
        self.add_subsystem("ATA30", DeiceSystemWeight(), promotes=["*"])

        self.add_subsystem("ATA34", NavigationSystemWeight(), promotes=["*"])
        self.add_subsystem("ATA49", APUWeight(), promotes=["*"])

        weight_sum = om.AddSubtractComp()
        weight_sum.add_equation(
            "data:weight:systems:mass",
            [
                "data:weight:systems:auxiliary_power_unit:mass",
                "data:weight:systems:electric_systems:electric_generation:mass",
                "data:weight:systems:electric_systems:electric_common_installation:mass",
                "data:weight:systems:hydraulic_systems:mass",
                "data:weight:systems:fire_protection:mass",
                "data:weight:systems:flight_furnishing:mass",
                "data:weight:systems:automatic_flight_system:mass",
                "data:weight:systems:communications:mass",
                "data:weight:systems:ECS:mass",
                "data:weight:systems:de-icing:mass",
                "data:weight:systems:navigation:mass",
                "data:weight:systems:flight_controls:mass",
            ],
            units="kg",
            desc="Mass of aircraft systems",
        )

        self.add_subsystem(
            "systems_weight_sum",
            weight_sum,
            promotes=["*"],
        )


class FurnitureWeight(om.Group):
    """
    Computes mass of furniture.
    """

    def setup(self):
        self.add_subsystem("ATA33", LightsWeight(), promotes=["*"])
        self.add_subsystem("ATA38", WaterWeight(), promotes=["*"])
        self.add_subsystem("ATA35", OxygenWeight(), promotes=["*"])
        self.add_subsystem("ATA25", FurnishingWeight(), promotes=["*"])
        self.add_subsystem("ATA2580", InsulationWeight(), promotes=["*"])
        self.add_subsystem("ATA2510", SeatsCrewWeight(), promotes=["*"])
        self.add_subsystem("ATA5345_5347", InteriorIntegrationWeight(), promotes=["*"])

        weight_sum = om.AddSubtractComp()
        weight_sum.add_equation(
            "data:weight:furniture:mass",
            [
                "data:weight:furniture:furnishing:mass",
                "data:weight:furniture:water:mass",
                "data:weight:furniture:interior_integration:mass",
                "data:weight:furniture:insulation:mass",
                "data:weight:furniture:cabin_lighting:mass",
                "data:weight:furniture:seats_crew_accommodation:mass",
                "data:weight:furniture:oxygen:mass",
            ],
            units="kg",
            desc="Mass of aircraft furniture",
        )

        self.add_subsystem(
            "furniture_weight_sum",
            weight_sum,
            promotes=["*"],
        )


class OperationalWeight(om.Group):
    """
    Computes mass of operational items and equipments.
    """

    def setup(self):
        self.add_subsystem("Op_items", OperationalItemsWeight(), promotes=["*"])
        self.add_subsystem(
            "Op_equipment", OperationalEquipmentsWeight(), promotes=["*"]
        )

        weight_sum = om.AddSubtractComp()
        weight_sum.add_equation(
            "data:weight:operational:mass",
            [
                "data:weight:operational:items:passenger_seats:mass",
                "data:weight:operational:items:unusable_fuel:mass",
                "data:weight:operational:items:documents_toolkit:mass",
                "data:weight:operational:items:galley_structure:mass",
                "data:weight:operational:equipment:others:mass",
                "data:weight:operational:equipment:crew:mass",
            ],
            units="kg",
            desc="Mass of aircraft operational items and equipments",
        )

        self.add_subsystem(
            "operational_weight_sum",
            weight_sum,
            promotes=["*"],
        )


class OperatingWeightEmpty(om.Group):
    """Operating Empty Weight (OEW) estimation.

    This group aggregates weight from all components of the aircraft.
    """

    def setup(self):
        # Propulsion should be done before airframe, because it drives pylon mass.
        self.add_subsystem("propulsion_weight", PropulsionWeight(), promotes=["*"])
        self.add_subsystem("airframe_weight", AirframeWeight(), promotes=["*"])
        self.add_subsystem("systems_weight", SystemsWeight(), promotes=["*"])
        self.add_subsystem("furniture_weight", FurnitureWeight(), promotes=["*"])
        self.add_subsystem("operational_weight", OperationalWeight(), promotes=["*"])
        self.add_subsystem("crew_weight", CrewWeight(), promotes=["*"])

        weight_sum = om.AddSubtractComp()
        weight_sum.add_equation(
            "data:weight:aircraft:OWE",
            [
                "data:weight:airframe:mass",
                "data:weight:propulsion:mass",
                "data:weight:systems:mass",
                "data:weight:furniture:mass",
                "data:weight:operational:mass",
                "data:weight:aircraft_empty:contingency",
            ],
            units="kg",
            desc="OWE",
        )

        self.add_subsystem("OWE_sum", weight_sum, promotes=["*"])
