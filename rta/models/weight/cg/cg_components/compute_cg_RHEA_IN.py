"""
    Estimation of components center of gravity
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


class ComputeCG_RHEA_IN(ExplicitComponent):
    """Components center of gravity taken as inputs"""

    def setup(self):

        self.add_input("data:weight:airframe:wing:CG:x", val=np.nan, units="m")
        self.add_input("data:weight:airframe:fuselage:CG:x", val=np.nan, units="m")
        self.add_input(
            "data:weight:airframe:horizontal_tail:CG:x", val=np.nan, units="m"
        )
        self.add_input("data:weight:airframe:vertical_tail:CG:x", val=np.nan, units="m")
        self.add_input(
            "data:weight:airframe:landing_gear:main:CG:x", val=np.nan, units="m"
        )
        self.add_input(
            "data:weight:airframe:landing_gear:front:CG:x", val=np.nan, units="m"
        )
        self.add_input(
            "data:weight:airframe:nacelle:CG:x", val=np.nan, units="m"
        )

        self.add_input("data:weight:propulsion:engine:CG:x", val=np.nan, units="m")
        self.add_input("data:weight:propulsion:propeller:CG:x", val=np.nan, units="m")
        self.add_input(
            "data:weight:propulsion:engine_controls_instrumentation:CG:x",
            val=np.nan,
            units="m",
        )
        self.add_input("data:weight:propulsion:fuel_system:CG:x", val=np.nan, units="m")

        self.add_input(
            "data:weight:systems:auxiliary_power_unit:CG:x", val=np.nan, units="m"
        )
        self.add_input(
            "data:weight:systems:electric_systems:electric_generation:CG:x",
            val=np.nan,
            units="m",
        )
        self.add_input(
            "data:weight:systems:electric_systems:electric_common_installation:CG:x",
            val=np.nan,
            units="m",
        )
        self.add_input(
            "data:weight:systems:hydraulic_systems:CG:x", val=np.nan, units="m"
        )
        self.add_input(
            "data:weight:systems:fire_protection:CG:x", val=np.nan, units="m"
        )
        self.add_input(
            "data:weight:systems:flight_furnishing:CG:x", val=np.nan, units="m"
        )
        self.add_input(
            "data:weight:systems:automatic_flight_system:CG:x", val=np.nan, units="m"
        )
        self.add_input("data:weight:systems:communications:CG:x", val=np.nan, units="m")
        self.add_input("data:weight:systems:ECS:CG:x", val=np.nan, units="m")
        self.add_input("data:weight:systems:de-icing:CG:x", val=np.nan, units="m")
        self.add_input("data:weight:systems:navigation:CG:x", val=np.nan, units="m")
        self.add_input(
            "data:weight:systems:flight_controls:CG:x", val=np.nan, units="m"
        )

        self.add_input("data:weight:furniture:furnishing:CG:x", val=np.nan, units="m")
        self.add_input("data:weight:furniture:water:CG:x", val=np.nan, units="m")
        self.add_input(
            "data:weight:furniture:interior_integration:CG:x", val=np.nan, units="m"
        )
        self.add_input("data:weight:furniture:insulation:CG:x", val=np.nan, units="m")
        self.add_input(
            "data:weight:furniture:cabin_lighting:CG:x", val=np.nan, units="m"
        )
        self.add_input(
            "data:weight:furniture:seats_crew_accommodation:CG:x", val=np.nan, units="m"
        )
        self.add_input("data:weight:furniture:oxygen:CG:x", val=np.nan, units="m")

        self.add_input(
            "data:weight:operational:items:passenger_seats:CG:x", val=np.nan, units="m"
        )
        self.add_input(
            "data:weight:operational:items:unusable_fuel:CG:x", val=np.nan, units="m"
        )
        self.add_input(
            "data:weight:operational:items:documents_toolkit:CG:x",
            val=np.nan,
            units="m",
        )
        self.add_input(
            "data:weight:operational:items:galley_structure:CG:x", val=np.nan, units="m"
        )
        self.add_input(
            "data:weight:operational:equipment:others:CG:x", val=np.nan, units="m"
        )
