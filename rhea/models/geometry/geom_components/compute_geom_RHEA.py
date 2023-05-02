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
from openmdao.core.explicitcomponent import ExplicitComponent





class ComputeGeometry_RHEA(ExplicitComponent):
    """
    This module computes total aircraft wet area, cnbetafuselage and aero center.
    """

    def setup(self):
        # CGs

        self.add_input("data:geometry:wing:MAC:at25percent:x", val=np.nan, units="m")
        self.add_input("data:geometry:wing:MAC:length", val=np.nan, units="m")
        self.add_input("data:geometry:wing:MAC:leading_edge:x:local", val=np.nan, units="m")
        self.add_input("data:geometry:horizontal_tail:MAC:at25percent:x:local", val=np.nan, units="m")
        self.add_input("data:geometry:horizontal_tail:MAC:at25percent:x:from_wingMAC25", val=np.nan, units="m")        
            # Fuselage parameters
        self.add_input("data:geometry:fuselage:length", val=np.nan, units="m")
        self.add_input("data:geometry:fuselage:maximum_width", val=np.nan, units="m")
        self.add_input("data:geometry:fuselage:front_length", val=np.nan, units="m")
        self.add_input("data:geometry:fuselage:rear_length", val=np.nan, units="m")
    
        # Horizontal Tail parameters

        self.add_input("data:geometry:horizontal_tail:root:chord", val=np.nan, units="m")
        self.add_input("data:geometry:horizontal_tail:tip:chord", val=np.nan, units="m")
        self.add_input("data:geometry:horizontal_tail:span", val=np.nan, units="m")
        self.add_input("data:geometry:horizontal_tail:sweep_0", val=np.nan, units="deg")    
    
        # Wing parameters
        
        self.add_input("data:geometry:wing:kink:leading_edge:x:local", val=np.nan, units="m")
        self.add_input("data:geometry:wing:tip:leading_edge:x:local", val=np.nan, units="m")
        self.add_input("data:geometry:wing:root:y", val=np.nan, units="m")
        self.add_input("data:geometry:wing:kink:y", val=np.nan, units="m")
        self.add_input("data:geometry:wing:tip:y", val=np.nan, units="m")
        self.add_input("data:geometry:wing:root:chord", val=np.nan, units="m")
        self.add_input("data:geometry:wing:kink:chord", val=np.nan, units="m")
        self.add_input("data:geometry:wing:tip:chord", val=np.nan, units="m")
