"""
Computation of wing area
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
from scipy.constants import g,minute
from rhea.models.aerodynamics.constants import POLAR_POINT_COUNT
from fastoad.models.performances.mission.polar import Polar
import pandas as pd
class ComputeEngineSize(om.ExplicitComponent):
    """
    Computes needed wing area for:
      - having enough lift at required approach speed
      - being able to load enough fuel to achieve the sizing mission
    """
    initial_RTO_power=2051000

    def setup(self):

        self.add_input("data:mission:sizing:takeoff:distance", val=np.nan, units="m")
        self.add_input("data:mission:sizing:OEI:net_ceiling", val=np.nan, units="m")
        self.add_input("data:mission:sizing:main_route:climb:TTC", val=np.nan, units="s")
        self.add_input("data:mission:sizing:cruise:max_mach", val=np.nan)
        self.add_input("data:TLAR:TOD", val=np.nan, units="m")
        self.add_input("data:TLAR:TTC", val=np.nan, units="min")
        self.add_input("data:TLAR:OEI_ceiling", val=np.nan, units="m")
        self.add_input("data:TLAR:cruise_mach", val=np.nan)
        #inputs:TTC
        #TLAR:tod; TLAR:OEI; TLAR:TTC; cruise mach
        # self.add_input("data:aerodynamics:aircraft:cruise:CL", np.nan, shape=POLAR_POINT_COUNT)
        # self.add_input("data:aerodynamics:aircraft:cruise:CD", np.nan, shape=POLAR_POINT_COUNT)
        # self.add_input("data:weight:aircraft:MTOW", np.nan, units="kg")
        
        self.add_output("data:propulsion:RTO_power",val=self.initial_RTO_power,  units="W")
        self.add_output("data:propulsion:Design_Thermo_Power",val=self.initial_RTO_power*1.22,  units="W")
        self.add_output("data:propulsion:propeller:max_power",val=self.initial_RTO_power*1.094,  units="W")
        self.declare_partials("data:propulsion:RTO_power", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):



        TOD =inputs["data:mission:sizing:takeoff:distance"]
        OEI_ceiling = inputs["data:mission:sizing:OEI:net_ceiling"]
        TTC = inputs["data:mission:sizing:main_route:climb:TTC"]
        Mach = inputs["data:mission:sizing:cruise:max_mach"]
        
        TOD_target = inputs["data:TLAR:TOD"]
        OEI_ceiling_target = inputs["data:TLAR:OEI_ceiling"]
        TTC_target = inputs["data:TLAR:TTC"]*minute  
        Mach_target = inputs["data:TLAR:cruise_mach"]

        try:
            previous_RTO_power = pd.read_csv('previous_RTO.csv',index_col=0)
            previous_RTO_power=previous_RTO_power.values[0]
        except:
            previous_RTO_power = self.initial_RTO_power
        

        delta_TOD= 1500*(TOD-TOD_target) #1500
        delta_OEI_ceiling=200*(OEI_ceiling_target - OEI_ceiling) #200
        delta_TTC = 400*(TTC-TTC_target) #400
        delta_Mach = 1e6*(Mach_target-Mach )
        if abs(max(TTC-TTC_target,TOD-TOD_target,OEI_ceiling_target - OEI_ceiling,(Mach_target-Mach)*10000))<10: #10m, 10s, 0.0
            RTO_power = previous_RTO_power
        else:
            RTO_power = previous_RTO_power +max(delta_TTC,delta_TOD,delta_OEI_ceiling,delta_Mach)
        
        print('engine_loop')
        print('RTO_power '+str(previous_RTO_power))
        print('delta_TOD'+str(TOD-TOD_target))
        print('delta_OEI_ceiling'+str(OEI_ceiling_target-OEI_ceiling))
        print('delta_TTC'+str(TTC-TTC_target))
        print('delta_Mach'+str(Mach_target-Mach))
        previous_RTO_power = pd.Series(data={'RTO':float(RTO_power)})       
        previous_RTO_power.to_csv('previous_RTO.csv')
        
        
        
        outputs["data:propulsion:RTO_power"] = RTO_power
        outputs["data:propulsion:Design_Thermo_Power"]=RTO_power*1.22
        outputs["data:propulsion:propeller:max_power"]=RTO_power*1.094


