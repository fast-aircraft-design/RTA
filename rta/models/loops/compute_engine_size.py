"""
Computation of turbopropeller output power
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
import pandas as pd
from scipy.constants import minute
from fastoad.module_management.constants import ModelDomain
from fastoad.module_management.service_registry import RegisterOpenMDAOSystem


@RegisterOpenMDAOSystem("rta.loop.engine_size", domain=ModelDomain.PROPULSION)
class ComputeEngineSize(om.ExplicitComponent):
    """
    May be replaced by an implicit component coupled with a mission module computing only takeoff
    """

    initial_RTO_power = 2051000

    def setup(self):

        self.add_input("data:mission:sizing:takeoff:distance", val=np.nan, units="m")
        self.add_input("data:TLAR:TOD", val=np.nan, units="m")
        self.add_input("data:TLAR:TTC", val=np.nan, units="min")
        self.add_input("data:TLAR:OEI_ceiling", val=np.nan, units="m")
        self.add_input("data:TLAR:cruise_mach", val=np.nan)

        self.add_output(
            "data:propulsion:RTO_power", val=self.initial_RTO_power, units="W"
        )
        self.add_output(
            "data:propulsion:Design_Thermo_Power",
            val=self.initial_RTO_power * 1.22,
            units="W",
        )
        self.add_output(
            "data:propulsion:propeller:max_power",
            val=self.initial_RTO_power * 1.094,
            units="W",
        )
        self.declare_partials("data:propulsion:RTO_power", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):

        TOD = inputs["data:mission:sizing:takeoff:distance"]
        TOD_target = inputs["data:TLAR:TOD"]

        if os.path.exists("previous_RTO.csv"):
            previous_RTO_power = pd.read_csv("previous_RTO.csv", index_col=0).values[0]
        else:
            previous_RTO_power = self.initial_RTO_power
            previous_RTO_power = pd.read_csv("previous_RTO.csv", index_col=0)
            previous_RTO_power = previous_RTO_power.values[0]
        except:
            previous_RTO_power = self.initial_RTO_power

        delta_TOD = 1500 * (TOD - TOD_target)  # 1500
        if abs(max(TOD - TOD_target)) < 10:  # 10m, 10s, 0.0
            RTO_power = previous_RTO_power
        else:
            RTO_power = previous_RTO_power + delta_TOD

        print("engine_loop")
        print("RTO_power " + str(previous_RTO_power))
        print("delta_TOD" + str(TOD - TOD_target))
        print("OEI_ceiling, Time To Climb and cruise Mach TLARs are deactivated.")
        previous_RTO_power = pd.Series(data={"RTO": float(RTO_power)})
        previous_RTO_power.to_csv("previous_RTO.csv")

        outputs["data:propulsion:RTO_power"] = RTO_power
        outputs["data:propulsion:Design_Thermo_Power"] = RTO_power * 1.22
        outputs["data:propulsion:propeller:max_power"] = RTO_power * 1.094
