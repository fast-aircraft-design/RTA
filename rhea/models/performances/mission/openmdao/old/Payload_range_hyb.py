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
from scipy.optimize import root_scalar
from scipy import constants

from fastoad import BundleLoader
from fastoad.models.aerodynamics.constants import POLAR_POINT_COUNT

from models.performances.mission.openmdao.regional_flight import SizingFlight_RHEA

class Payload_range(SizingFlight_RHEA):
    """
    Computes payload range diagram points.
    """

    def setup(self):
        self._engine_wrapper = BundleLoader().instantiate_component(self.options["propulsion_id"])
        self._engine_wrapper.setup(self)

        # Inputs -----------------------------------------------------------------------------------
        self.add_input("data:mission:DOC:cruise:mach", np.nan)
        self.add_input("data:TLAR:cruise_mach", np.nan)
        self.add_input("data:mission:sizing:diversion:mach", np.nan)
        self.add_input("data:TLAR:range", np.nan, units="m")
        self.add_input("data:mission:DOC:range", np.nan, units="m")
        self.add_input("data:mission:DOC:payload", np.nan, units="kg")
        self.add_input("data:weight:aircraft:OWE", np.nan, units="kg")
        self.add_input("data:weight:aircraft:MZFW", np.nan, units="kg")
        self.add_input("data:weight:aircraft:MFW", np.nan, units="kg")
        self.add_input("data:weight:aircraft:payload", np.nan, units="kg")
        
        if 'PHFC' in self.options["propulsion_id"]:
            self.add_input("data:geometry:propulsion:motor:count",  np.nan)
            
        self.add_input("data:geometry:propulsion:engine:count", 2)
        self.add_input("data:geometry:wing:area", np.nan, units="m**2")

        self.add_input("data:aerodynamics:aircraft:cruise:CL", np.nan, shape=POLAR_POINT_COUNT)
        self.add_input("data:aerodynamics:aircraft:cruise:CD", np.nan, shape=POLAR_POINT_COUNT)

        self.add_input("data:aerodynamics:aircraft:takeoff:CL", np.nan, shape=POLAR_POINT_COUNT)
        self.add_input("data:aerodynamics:aircraft:takeoff:CD", np.nan, shape=POLAR_POINT_COUNT)

        self.add_input("data:weight:aircraft:MTOW", np.nan, units="kg")

        self.add_input("data:mission:DOC:taxi_out:fuel", np.nan, units="kg")

        self.add_input("data:mission:DOC:takeoff:V2", np.nan, units="m/s")

        self.add_input("data:mission:DOC:takeoff:altitude", np.nan, units="m")
        self.add_input("data:mission:DOC:takeoff:fuel", np.nan, units="kg")

        self.add_input("data:mission:DOC:climb:thrust_rate", np.nan)
        self.add_input("data:mission:DOC:climb:speed", np.nan,units="m/s")
        self.add_input("data:mission:DOC:main_route:cruise:altitude", np.nan, units="ft")
        self.add_input("data:mission:DOC:descent:thrust_rate", np.nan)
        self.add_input("data:mission:DOC:descent:speed", np.nan,units="m/s")
        
        
        self.add_input("data:mission:DOC:diversion:distance", np.nan, units="m")
        self.add_input("data:mission:DOC:diversion:altitude", np.nan, units="ft")
        self.add_input("data:mission:DOC:holding:duration", np.nan, units="s")
        
        self.add_input("data:mission:DOC:landing:fuel", np.nan, units="kg")
        self.add_input("data:mission:DOC:taxi_in:fuel", np.nan, units="kg")

        self.add_input("data:mission:DOC:taxi_in:duration", np.nan, units="s")
        self.add_input("data:mission:DOC:taxi_in:speed", np.nan, units="m/s")
        self.add_input("data:mission:DOC:taxi_in:thrust_rate", np.nan)
        
        

        # Outputs ----------------------------------------------------------------------------------
        self.add_output("data:mission:DOC:TOW", units="kg")
        
        self.add_output("data:mission:DOC:initial_climb:fuel", units="kg")
        self.add_output("data:mission:DOC:main_route:climb:fuel", units="kg")
        self.add_output("data:mission:DOC:main_route:cruise:fuel", units="kg")
        self.add_output("data:mission:DOC:main_route:descent:fuel", units="kg")
        self.add_output("data:mission:DOC:initial_climb:H2", units="kg",val=0)
        self.add_output("data:mission:DOC:main_route:climb:H2", units="kg" ,val=0.)
        self.add_output("data:mission:DOC:main_route:cruise:H2", units="kg" ,val=0.)
        self.add_output("data:mission:DOC:main_route:descent:H2", units="kg", val=0.)
        
        
        self.add_output("data:mission:DOC:initial_climb:distance", units="m")
        self.add_output("data:mission:DOC:main_route:climb:distance", units="m")
        self.add_output("data:mission:DOC:main_route:cruise:distance", units="m")
        self.add_output("data:mission:DOC:main_route:descent:distance", units="m")

        self.add_output("data:mission:DOC:initial_climb:duration", units="s")
        self.add_output("data:mission:DOC:main_route:climb:duration", units="s")
        self.add_output("data:mission:DOC:main_route:cruise:duration", units="s")
        self.add_output("data:mission:DOC:main_route:descent:duration", units="s")

        self.add_output("data:mission:DOC:diversion:climb:fuel", units="kg")
        self.add_output("data:mission:DOC:diversion:cruise:fuel", units="kg")
        self.add_output("data:mission:DOC:diversion:descent:fuel", units="kg")
        self.add_output("data:mission:DOC:diversion:climb:H2", units="kg" ,val=0.)
        self.add_output("data:mission:DOC:diversion:cruise:H2", units="kg" ,val=0.)
        self.add_output("data:mission:DOC:diversion:descent:H2", units="kg", val=0.)
        
        self.add_output("data:mission:DOC:diversion:climb:distance", units="m")
        self.add_output("data:mission:DOC:diversion:cruise:distance", units="m")
        self.add_output("data:mission:DOC:diversion:descent:distance", units="m")

        self.add_output("data:mission:DOC:diversion:climb:duration", units="s")
        self.add_output("data:mission:DOC:diversion:cruise:duration", units="s")
        self.add_output("data:mission:DOC:diversion:descent:duration", units="s")

        self.add_output("data:mission:DOC:holding:fuel", units="kg")
        self.add_output("data:mission:DOC:holding:H2", units="kg", val=0.)
        
        #self.add_output("data:mission:DOC:taxi_in:fuel", units="kg")

        self.add_output("data:mission:DOC:ZFW", units="kg")
        self.add_output("data:mission:DOC:fuel", units="kg")
        self.add_output("data:mission:DOC:H2", units="kg",val=0.)
        self.add_output("data:mission:DOC:trip_fuel", units="kg")  
        self.add_output("data:mission:DOC:trip_H2", units="kg", val=0.)                
        self.add_output("data:mission:DOC:block_fuel", units="kg")
        
        self.add_output("data:mission:payload_range:cruise_mach")
        self.add_output("data:mission:payload_range:range", units="NM",shape=6)
        self.add_output("data:mission:payload_range:TOW", units="kg",shape=6)
        self.add_output("data:mission:payload_range:ZFW", units="kg",shape=6)
        self.add_output("data:mission:payload_range:payload", units="kg",shape=6)
        self.add_output("data:mission:payload_range:fuel", units="kg",shape=6)
        self.add_output("data:mission:payload_range:H2", units="kg",shape=6)

        self.declare_partials(["*"], ["*"])

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        
        outputs["data:mission:payload_range:cruise_mach"]= inputs['data:TLAR:cruise_mach']
        
        PL_A,Range_A, fuel_A,TOW_A,ZFW_A, H2_A=self.compute_point_A( inputs, outputs)
        PL_B,Range_B, fuel_B,TOW_B,ZFW_B,H2_B=self.compute_point_B( inputs, outputs)
        
        PL_C,Range_C, fuel_C,TOW_C,ZFW_C,H2_C=self.compute_point_C( inputs, outputs)
        PL_D,Range_D, fuel_D,TOW_D,ZFW_D,H2_D=self.compute_point_D( inputs, outputs)

        PL_list=np.array([PL_A,PL_B,PL_A,PL_B,PL_C,PL_D])
        Range_list=np.array([0.,0.,Range_A,Range_B,Range_C,Range_D])/ constants.nautical_mile
        fuel_list=np.array([0.,0.,fuel_A,fuel_B,fuel_C,fuel_D])
        H2_list=np.array([0.,0.,H2_A,H2_B,H2_C,H2_D])
        ZFW_list=np.array([ZFW_A,ZFW_B,ZFW_A,ZFW_B,ZFW_C,ZFW_D])
        TOW_list=np.array([TOW_A,TOW_B,TOW_A,TOW_B,TOW_C,TOW_D])
        outputs["data:mission:payload_range:fuel"] = fuel_list.reshape(6)
        outputs["data:mission:payload_range:ZFW"]=ZFW_list.reshape(6)
        outputs["data:mission:payload_range:TOW"]=   TOW_list.reshape(6)
        outputs["data:mission:payload_range:payload"]=    PL_list.reshape(6)
        outputs["data:mission:payload_range:range"]=Range_list.reshape(6)
        outputs["data:mission:payload_range:H2"] = H2_list.reshape(6)

        

    def compute_point_A(self, inputs, outputs):        
        payload=inputs["data:weight:aircraft:payload"]
        OWE=inputs["data:weight:aircraft:OWE"]        
        TOW_input= inputs["data:weight:aircraft:MTOW"]       
        try:
            tol = 10e-3
            ZFW_target =  inputs["data:weight:aircraft:MZFW"]
            mission_type= 'payload_range'
            def compute_point(range_input):            
                loop_inputs=[range_input,TOW_input]
                self.compute_mission(inputs, outputs,loop_inputs,mission_type)
                return ZFW_target-outputs["data:mission:DOC:ZFW"]
            
          
            sol= root_scalar(compute_point, x0=inputs["data:TLAR:range"], x1=inputs["data:TLAR:range"] / 2.0, rtol=tol,method='secant')
            max_range = sol.root
            fuel = outputs["data:mission:DOC:fuel"]
            H2 = outputs["data:mission:DOC:H2"]
            ZFW = outputs["data:mission:DOC:ZFW"]
            fuel1=fuel+0
            ZFW1=ZFW+0
            H21=H2+0
            TOW=   ZFW + fuel + H2
            payload = ZFW-OWE 



        except IndexError:
            self.compute_breguet(inputs, outputs,TOW_input)
        return payload,max_range,fuel1,TOW,ZFW1,H21

    def compute_point_B(self, inputs, outputs):        
        payload=inputs["data:weight:aircraft:payload"]
        OWE=inputs["data:weight:aircraft:OWE"]  
        TOW_input= inputs["data:weight:aircraft:MTOW"]       
        try:
            tol = 10e-3
            ZFW_target =  OWE+payload
            mission_type= 'payload_range'
            def compute_point(range_input):            
                loop_inputs=[range_input,TOW_input]
                self.compute_mission(inputs, outputs,loop_inputs,mission_type)
                return ZFW_target-outputs["data:mission:DOC:ZFW"]
            
           
            sol= root_scalar(compute_point, x0=inputs["data:TLAR:range"], x1=inputs["data:TLAR:range"] / 2.0, rtol=tol,method='secant')
            max_range = sol.root
            fuel = outputs["data:mission:DOC:fuel"]
            ZFW=outputs["data:mission:DOC:ZFW"]
            H2 = outputs["data:mission:DOC:H2"]

            fuel2=fuel+0
            ZFW2=ZFW+0
            H22=H2+0
            TOW=   ZFW + fuel + H2 
            payload=ZFW-OWE 


        except IndexError:
            self.compute_breguet(inputs, outputs,TOW_input)        
        return payload,max_range,fuel2,TOW,ZFW2,H22

    
    def compute_point_C(self, inputs, outputs):        
        payload=inputs["data:weight:aircraft:payload"]
        OWE=inputs["data:weight:aircraft:OWE"]        
        TOW_input= inputs["data:weight:aircraft:MTOW"] 
        MFW= inputs["data:weight:aircraft:MFW"] 
        try:
            tol = 10e-3
            fuel_target =MFW  
            mission_type= 'payload_range'
            def compute_point(range_input):            
                loop_inputs=[range_input,TOW_input]
                self.compute_mission(inputs, outputs,loop_inputs,mission_type)
                return fuel_target-outputs["data:mission:DOC:fuel"]
            
           
            sol= root_scalar(compute_point, x0=inputs["data:TLAR:range"], x1=inputs["data:TLAR:range"] / 2.0, rtol=tol,method='secant')
            max_range = sol.root
            fuel = outputs["data:mission:DOC:fuel"]
            ZFW=outputs["data:mission:DOC:ZFW"]
            H2 = outputs["data:mission:DOC:H2"]
            
            fuel3=fuel+0
            ZFW3=ZFW+0
            H23=H2+0           
            TOW=   ZFW + fuel + H2
            payload=ZFW-OWE 


        except IndexError:
            self.compute_breguet(inputs, outputs,TOW_input)        
        return payload,max_range,fuel3,TOW,ZFW3, H23
    
    def compute_point_D(self, inputs, outputs):        
        payload=inputs["data:weight:aircraft:payload"]
        OWE=inputs["data:weight:aircraft:OWE"]               
        MFW= inputs["data:weight:aircraft:MFW"] 
        TOW_input= OWE+MFW
        try:
            tol = 10e-3
            fuel_target =MFW  
            mission_type= 'payload_range'
            def compute_point(range_input):            
                loop_inputs=[range_input,TOW_input]
                self.compute_mission(inputs, outputs,loop_inputs,mission_type)
                return fuel_target-outputs["data:mission:DOC:fuel"]
            
           
            sol= root_scalar(compute_point, x0=inputs["data:TLAR:range"], x1=inputs["data:TLAR:range"] / 2.0, rtol=tol,method='secant')
            max_range = sol.root
            fuel = outputs["data:mission:DOC:fuel"]
            ZFW=outputs["data:mission:DOC:ZFW"]
            H2 = outputs["data:mission:DOC:H2"]            
            
            fuel4=fuel+0
            ZFW4=ZFW+0
            H24=H2+0           
            TOW=   ZFW + fuel + H2 
            payload=ZFW-OWE 
            outputs["data:mission:DOC:TOW"]=TOW_input


        except IndexError:
            self.compute_breguet(inputs, outputs,TOW_input)        
        return payload,max_range,fuel4,TOW,ZFW4,H24