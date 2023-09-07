"""
Main components for mass breakdown
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

from fastoad_cs25.models.constants import PAYLOAD_FROM_NPAX
from fastoad_cs25.models.weight.mass_breakdown.e_crew import CrewWeight
from fastoad_cs25.models.weight.mass_breakdown.payload import ComputePayload
from .b_propulsion.turboprop_weight import TurbopropWeight
from .b_propulsion.fuel_lines_weight import (
    FuelLinesWeight,
)
import numpy as np


class MassBreakdown_RHEA(om.Group):
    """
    Each component mass of the conventional a/c configuration is taken as input.
    Mass models re only used to evaluate the modified power-plant system (including all electricaland thermal
    components needed for its correct functoning).

    Computes analytically the resulting sum of the systems mass, propulsion mass, operating mss,
    airframe mass and the Overall Weight Empty (OWE).

    Evaluates the remaining available payload mass.
    """

    def __init__(self, hybrid, payload_from_npax, out_file):
        super().__init__()
        self.hybrid = hybrid
        self.payload_from_npax = payload_from_npax
        self.out_file = out_file

    # def initialize(self):
    # self.options.declare(PAYLOAD_FROM_NPAX, types=bool, default=True)

    # self.options.declare("hybrid",types=bool)

    def setup(self):

        self.add_subsystem(
            "owe",
            OperatingWeightEmpty(self.hybrid, self.payload_from_npax, self.out_file),
            promotes=["*"],
        )
        # if self.payload_from_npax:
        #     self.add_subsystem("payload", ComputePayload(), promotes=["*"])
        # else:
        #     self.add_subsystem("payload", ComputePayloadfromOWE(), promotes=["*"])
        # self.add_subsystem("owe", OperatingWeightEmpty(self.hybrid), promotes=["*"])


class ComputePayloadfromOWE(om.ExplicitComponent):
    """Computes payload from OWE"""

    def setup(self):
        self.add_input(
            "settings:weight:aircraft:design_mass_per_seat",
            units="kg",
            desc="Design value of mass per seat",
        )
        self.add_input(
            "settings:weight:aircraft:payload:design_mass_per_passenger",
            units="kg",
            desc="Design value of mass per passenger",
        )
        # self.add_input("data:weight:aircraft:OWE_no_seats", units="kg")
        self.add_input("data:weight:airframe:mass", units="kg")
        self.add_input("data:weight:propulsion:mass", units="kg")
        self.add_input("data:weight:systems:mass", units="kg")
        self.add_input("data:weight:furniture:mass", units="kg")
        self.add_input("data:weight:operational_no_seats:mass", units="kg")
        self.add_input("data:weight:aircraft_empty:contingency", units="kg")

        self.add_input("data:mission:sizing:ZFW", units="kg")
        self.add_input("data:geometry:cabin:NPAX1")
        self.add_input("data:geometry:cabin:seats:economical:count_by_row")

        self.add_input("data:geometry:fuselage:PAX_length", units="m")
        self.add_input("data:geometry:fuselage:available_tail_length", 5, units="m")
        self.add_input("data:propulsion:electric_systems:H2_storage:length", units="m")

        self.add_output("data:TLAR:NPAX")
        self.add_output("data:weight:aircraft:payload", units="kg")
        self.add_output("data:weight:aircraft:max_payload", units="kg")
        self.add_output(
            "data:weight:operational:items:passenger_seats:mass", units="kg"
        )

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        OWE_no_seats = (
            inputs["data:weight:airframe:mass"]
            + inputs["data:weight:propulsion:mass"]
            + inputs["data:weight:systems:mass"]
            + inputs["data:weight:furniture:mass"]
            + inputs["data:weight:operational_no_seats:mass"]
            + inputs["data:weight:aircraft_empty:contingency"]
        )
        H2_tank_length = inputs["data:propulsion:electric_systems:H2_storage:length"]
        tailcone_length = inputs["data:geometry:fuselage:available_tail_length"]
        count_by_row = inputs["data:geometry:cabin:seats:economical:count_by_row"]
        row_length = inputs["data:geometry:fuselage:PAX_length"] / (
            inputs["data:geometry:cabin:NPAX1"] / count_by_row
        )
        ZFW = inputs["data:mission:sizing:ZFW"]
        max_pax = inputs["data:geometry:cabin:NPAX1"]
        mass_per_seat = inputs["settings:weight:aircraft:design_mass_per_seat"]

        mass_per_pax = inputs[
            "settings:weight:aircraft:payload:design_mass_per_passenger"
        ]

        max_payload = ZFW - OWE_no_seats
        n_pax = int(max_payload / (mass_per_pax + mass_per_seat))

        available_length = tailcone_length + row_length * int(
            max_pax / count_by_row - n_pax / count_by_row
        )
        while available_length < H2_tank_length:
            n_pax -= 1
            available_length = tailcone_length + row_length * int(
                max_pax / count_by_row - n_pax / count_by_row
            )

        if n_pax > max_pax:
            n_pax = max_pax
        #     else:
        #         n_pax+=-1
        mass_seats = mass_per_seat * n_pax

        outputs["data:weight:aircraft:payload"] = n_pax * mass_per_pax
        outputs["data:weight:aircraft:max_payload"] = ZFW - (OWE_no_seats + mass_seats)
        outputs["data:TLAR:NPAX"] = n_pax
        outputs["data:weight:operational:items:passenger_seats:mass"] = mass_seats


class OperatingWeightEmpty(om.Group):
    """Operating Empty Weight (OEW) estimation

    This group aggregates weight from all components of the aircraft.
    """

    def __init__(self, hybrid, payload_from_npax, out_file):
        super().__init__()
        self.hybrid = hybrid
        self.payload_from_npax = payload_from_npax
        self.out_file = out_file

    # def initialize(self):
    #     self.options.declare("hybrid",types=bool, default=True)

    def setup(self):

        # Crew
        self.add_subsystem("crew_weight", CrewWeight(), promotes=["*"])

        # Make additions
        self.add_subsystem(
            "airframe_weight_sum",
            om.AddSubtractComp(
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
            ),
            promotes=["*"],
        )

        # Propulsion
        self.add_subsystem(
            "turboprop_weight", TurbopropWeight(), promotes=["*"]
        )  # to add once we do the sizing of the turboprop
        # if self.hybrid:
        self.add_subsystem("ATA28", FuelLinesWeight(), promotes=["*"])

        self.add_subsystem(
            "propulsion_weight_sum",
            om.AddSubtractComp(
                "data:weight:propulsion:mass",
                [
                    "data:weight:propulsion:engine:mass",
                    "data:weight:propulsion:propeller:mass",
                    "data:weight:propulsion:engine_controls_instrumentation:mass",
                    "data:weight:propulsion:fuel_system:mass",
                    "data:weight:propulsion:electric_systems:fuel_cell:mass",
                    "data:weight:propulsion:electric_systems:motor:mass",
                    "data:weight:propulsion:electric_systems:power_electronics:mass",
                    "data:weight:propulsion:electric_systems:battery:mass",
                    "data:weight:propulsion:electric_systems:H2_storage:mass",
                    "data:weight:propulsion:electric_systems:cooling:mass",
                    "data:weight:propulsion:electric_systems:cables:mass",
                    "data:weight:propulsion:electric_systems:H2_distribution:mass",
                ],
                units="kg",
                desc="Mass of the propulsion system",
            ),
            promotes=["*"],
        )

        # else:
        #     self.add_subsystem(
        #         "propulsion_weight_sum",
        #         om.AddSubtractComp(
        #             "data:weight:propulsion:mass",
        #             [
        #             "data:weight:propulsion:engine:mass",
        #             "data:weight:propulsion:propeller:mass",
        #             "data:weight:propulsion:engine_controls_instrumentation:mass",
        #             "data:weight:propulsion:fuel_system:mass",
        #             ],
        #             units="kg",
        #             desc="Mass of the propulsion system",
        #         ),
        #         promotes=["*"],
        #     )

        self.add_subsystem(
            "systems_weight_sum",
            om.AddSubtractComp(
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
            ),
            promotes=["*"],
        )

        self.add_subsystem(
            "furniture_weight_sum",
            om.AddSubtractComp(
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
            ),
            promotes=["*"],
        )

        self.add_subsystem(
            "operational_weight_sum_no_seats",
            om.AddSubtractComp(
                "data:weight:operational_no_seats:mass",
                [
                    "data:weight:operational:items:unusable_fuel:mass",
                    "data:weight:operational:items:documents_toolkit:mass",
                    "data:weight:operational:items:galley_structure:mass",
                    "data:weight:operational:equipment:others:mass",
                    "data:weight:operational:equipment:crew:mass",
                ],
                units="kg",
                desc="Mass of aircraft operational items and equipments without passenger seats",
            ),
            promotes=["*"],
        )

        if self.payload_from_npax:
            self.add_subsystem("payload", ComputePayload(), promotes=["*"])
        else:
            self.add_subsystem("payload", ComputePayloadfromOWE(), promotes=["*"])

        self.add_subsystem(
            "operational_weight_sum",
            om.AddSubtractComp(
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
            ),
            promotes=["*"],
        )

        self.add_subsystem(
            "OWE_sum",
            om.AddSubtractComp(
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
            ),
            promotes=["*"],
        )
