"""
Estimation of furniture weight
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
from .ATA25_furnishing_weight import FurnishingWeight
from .ATA33_lighting_weight import LightsWeight
from .ATA35_oxygen_weight import OxygenWeight
from .ATA38_water_weight import WaterWeight
from .ATA2580_insulation_weight import InsulationWeight
from .ATA5345_5347_interior_weight import InteriorIntegrationWeight
from .ATA2510_crew_seats_weight import SeatsCrewWeight
