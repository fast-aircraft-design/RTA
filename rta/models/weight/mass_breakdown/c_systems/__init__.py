"""
Estimation of weight of all-mission systems
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

from .ATA21_environmental_control_systems_weight import ECSWeight
from .ATA22_autoflight_systems_weight import AutoFlightSystemWeight
from .ATA23_communication_systems_weight import CommunicationSystemWeightLegacy
from .ATA24_electrical_systems_weight import ElectricalPowerSystemWeight
from .ATA25_flight_furnishing_systems_weight import FlightFurnishingWeight
from .ATA26_fire_systems_weight import FireSystemWeight

from .ATA27_flight_control_systems_weight import FlightControlsSystemWeight
from .ATA29_hydraulic_systems_weight import HydraulicPowerSystemWeight
from .ATA30_deice_systems_weight import DeiceSystemWeight
from .ATA34_navigation_systems_weight import NavigationSystemWeight
from .ATA49_auxiliary_power_systems_weight import APUWeight
