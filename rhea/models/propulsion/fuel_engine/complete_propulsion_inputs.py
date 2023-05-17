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
import os
from .constants import POWER_RATE_COUNT

class CompletePropulsionInputs(om.ExplicitComponent):
    """
    Completes propulsion inputs starting from P_non and RTO_power
    """

    def setup(self):


        
        self.add_input("data:propulsion:electric_systems:P_nom_norm", np.nan, units="W")
        self.add_input("data:propulsion:RTO_power_norm", np.nan, units="W")
        
        self.add_output("data:propulsion:electric_systems:P_nom", units="W")
        self.add_output("data:propulsion:RTO_power", units="W")   
        
        self.add_input("sego:climb:EM_rate0", np.nan)
        self.add_input("sego:climb:EM_rate1", np.nan)
        self.add_input("sego:climb:EM_rate2", np.nan)
        self.add_input("sego:climb:EM_rate3", np.nan)
        self.add_input("sego:climb:EM_rate4", np.nan)
        self.add_input("sego:climb:EM_rate5", np.nan)
        self.add_input("sego:climb:EM_rate6", np.nan)
        
        self.add_input("sego:climb:TP_rate0", np.nan)
        self.add_input("sego:climb:TP_rate1", np.nan)
        self.add_input("sego:climb:TP_rate2", np.nan)
        self.add_input("sego:climb:TP_rate3", np.nan)
        self.add_input("sego:climb:TP_rate4", np.nan)
        self.add_input("sego:climb:TP_rate5", np.nan)
        self.add_input("sego:climb:TP_rate6", np.nan)    
        
        # self.add_input("data:propulsion:RTO_power", np.nan, units="W")        
        self.add_output("data:mission:sizing:climb:EM_power_rate", shape=POWER_RATE_COUNT)
        self.add_output("data:mission:sizing:climb:TP_power_rate", shape=POWER_RATE_COUNT)
        
        self.add_output("data:propulsion:Design_Thermo_Power",  units="W")
        self.add_output("data:propulsion:propeller:max_power",  units="W")
        self.add_output("data:propulsion:electric_systems:fuel_cell:layout:stacks")
        self.declare_partials("data:propulsion:electric_systems:fuel_cell:layout:stacks", "data:propulsion:electric_systems:P_nom", method="fd")
        self.declare_partials("data:propulsion:propeller:max_power", "data:propulsion:RTO_power", method="fd")
        self.declare_partials("data:propulsion:Design_Thermo_Power", "data:propulsion:RTO_power", method="fd")
        self.declare_partials("data:propulsion:RTO_power", "data:propulsion:RTO_power_norm", method="fd")
        self.declare_partials("data:propulsion:electric_systems:P_nom", "data:propulsion:electric_systems:P_nom_norm", method="fd")
    
    
    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):     
        # RTO_power =inputs["data:propulsion:RTO_power"]
        # P_nom =inputs["data:propulsion:electric_systems:P_nom"]
        RTO_power_norm =inputs["data:propulsion:RTO_power_norm"]
        P_nom_norm =inputs["data:propulsion:electric_systems:P_nom_norm"]
        EM0 =inputs["sego:climb:EM_rate0"]
        EM1 =inputs["sego:climb:EM_rate1"]
        EM2 =inputs["sego:climb:EM_rate2"]
        EM3 =inputs["sego:climb:EM_rate3"]
        EM4 =inputs["sego:climb:EM_rate4"]
        EM5 =inputs["sego:climb:EM_rate5"]
        EM6 =inputs["sego:climb:EM_rate6"]
        
        TP0 =inputs["sego:climb:TP_rate0"]
        TP1 =inputs["sego:climb:TP_rate1"]
        TP2 =inputs["sego:climb:TP_rate2"]
        TP3 =inputs["sego:climb:TP_rate3"]
        TP4 =inputs["sego:climb:TP_rate4"]
        TP5 =inputs["sego:climb:TP_rate5"]
        TP6 =inputs["sego:climb:TP_rate6"]        
        
        outputs["data:mission:sizing:climb:EM_power_rate"]=[EM0,EM1,EM2,EM3,EM4,EM5,EM6]
        outputs["data:mission:sizing:climb:TP_power_rate"]=[TP0,TP1,TP2,TP3,TP4,TP5,TP6]
        
        try:
            os.remove("fail.txt")
        except:
            print("no previous fail to remove")
        #print('EM_power_rate', outputs["data:mission:sizing:climb:EM_power_rate"])
        # P_nom = P_nom_norm*(800000-400000)+400000
        # RTO_power=  RTO_power_norm *(2000000-1800000)+1800000
        P_nom = P_nom_norm*(1000000-500000)+500000
        RTO_power=  RTO_power_norm *(2000000-1200000)+1200000
        outputs["data:propulsion:electric_systems:P_nom"]=P_nom
        outputs["data:propulsion:RTO_power"]=RTO_power
       
        outputs["data:propulsion:electric_systems:fuel_cell:layout:stacks"] = P_nom/100000                      
        outputs["data:propulsion:Design_Thermo_Power"]=RTO_power*1.22
        #outputs["data:propulsion:propeller:max_power"]=max((TP0*RTO_power+P_nom),RTO_power)*1.094
        outputs["data:propulsion:propeller:max_power"]=max((RTO_power+P_nom),RTO_power)*1.094


