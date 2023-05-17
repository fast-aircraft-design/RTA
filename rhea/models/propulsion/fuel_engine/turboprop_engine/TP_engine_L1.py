"""Parametric turboprop engine."""
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
# import pickle

from scipy import constants
import logging
import math
from typing import Union, Sequence, Tuple, Optional
import os
import numpy as np
from fastoad.constants import FlightPhase
#from fastoad.models.propulsion import IPropulsion
#from models.propulsion import IPropulsion
from fastoad_cs25.models.propulsion.fuel_propulsion.rubber_engine.exceptions import (
    FastRubberEngineInconsistentInputParametersError,
)
from fastoad.model_base.atmosphere import Atmosphere
from ..turboprop_engine.engine_components.Ram import Ram
from ..turboprop_engine.engine_components.Compression_Nozzle import Compression_Nozzle
from ..turboprop_engine.engine_components.Low_Pressure_Compressor import LPC
from ..turboprop_engine.engine_components.High_Pressure_Compressor import HPC
from ..turboprop_engine.engine_components.Combustor import Combustor
from ..turboprop_engine.engine_components.Low_Pressure_Turbine import LPT
from ..turboprop_engine.engine_components.High_Pressure_Turbine import HPT
from ..turboprop_engine.engine_components.Power_Turbine import Power_Turbine
from ..turboprop_engine.engine_components.Expansion_Nozzle import Expansion_Nozzle
from ..turboprop_engine.engine_components.Thrust import Thrust
from ..turboprop_engine.engine_components.Fuel_Data import Fuel_data
import time
from scipy.optimize import root_scalar, fsolve
import pandas as pd
# from fastoad.models.propulsion.fuel_propulsion.base import AbstractFuelPropulsion
from ..turboprop_engine.base import AbstractFuelPropulsion
from fastoad.model_base.flight_point import FlightPoint
#from fastoad.model_base.dict import AddKeyAttributes
from ..turboprop_engine.engine_components.Propeller import Propeller

# Logger for this module
_LOGGER = logging.getLogger(__name__)
#AddKeyAttributes(["psfc","shaft_power", "power_rate","thermo_power","TP_thermal_efficiency","TP_residual_thrust","TP_air_flow","TP_total_pressure","TP_total_temperature","fuel_mass"
                  #,"H2_mass","TPshaft_power","EMshaft_power","FC_power","TP_power_rate","EM_power_rate","H2_fc"])(FlightPoint)


class TPEngine_L1(AbstractFuelPropulsion):
    def __init__(
        self,
        RTO_power: float,
        Design_Thermo_Power: float,
        Power_Offtake: float,
        gearbox_eta: float,
        d_prop:float,
        fuel:float,
        
        turbine_inlet_temperature: float,
        HP_bleed: float,
        LP_bleed: float,
        
        inlet_eta_pol: float,
        inlet_pressure_ratio: float,
        lpc_eta_pol: float,
        lpc_pressure_ratio: float,
       
        hpc_eta_pol: float,
        hpc_pressure_ratio: float,
       
        combustor_eta: float,
        combustor_pressure_ratio: float,
        
        hpt_eta_pol: float,
        hpt_eta_mech: float,

        lpt_eta_pol: float,
        lpt_eta_mech: float,

        pt_eta_pol: float,
        pt_eta_mech: float,
        nozzle_eta_pol: float,
        nozzle_pressure_ratio: float,
        nozzle_area_ratio: float,       
        
        k0: float,
        k1: float,
        k2: float,
        tau_t_sizing: float,
        pi_t_sizing: float,
        M_out_sizing: float,
        
        k_th_RTO:float,
        k_gb_RTO: float,
        k_th_NTO: float,
        k_gb_NTO: float,
        k_th_MCL: float,
        k_gb_MCL: float,
        k_th_MCT: float,
        k_gb_MCT: float,
        k_th_MCR: float,
        k_gb_MCR: float,
        k_th_FID: float,
        k_gb_FID: float, 
        
        k_psfc:float,
        k_prop:float,
    ):
        """
        Thermodynamic turboprop engine model.


        :param 
        """
        # pylint: disable=too-many-arguments  # they define the engine

        self.RTO_power = RTO_power
        self.Design_Thermo_Power = Design_Thermo_Power
        self.Power_Offtake = Power_Offtake
        self.gearbox_eta = gearbox_eta
        self.d_prop=d_prop
        self.fuel=fuel
        
        self.T4 =turbine_inlet_temperature
        self.HP_bleed=HP_bleed
        self.LP_bleed=LP_bleed
        
        self.inlet_eta_pol=inlet_eta_pol
        self.inlet_pressure_ratio=inlet_pressure_ratio
        self.lpc_eta_pol=lpc_eta_pol
        self.lpc_pressure_ratio=lpc_pressure_ratio
       
        self.hpc_eta_pol=hpc_eta_pol
        self.hpc_pressure_ratio=hpc_pressure_ratio
       
        self.combustor_eta=combustor_eta
        self.combustor_pressure_ratio=combustor_pressure_ratio
        
        self.hpt_eta_pol=hpt_eta_pol
        self.hpt_eta_mech=hpt_eta_mech

        self.lpt_eta_pol=lpt_eta_pol
        self.lpt_eta_mech=lpt_eta_mech

        self.pt_eta_pol=pt_eta_pol
        self.pt_eta_mech=pt_eta_mech
        self.nozzle_eta_pol=nozzle_eta_pol
        self.nozzle_pressure_ratio=nozzle_pressure_ratio
        self.nozzle_area_ratio=nozzle_area_ratio
        
        self.k0 = k0
        self.k1 =k1
        self.k2 =k2
        self.tau_t_sizing = tau_t_sizing
        self.pi_t_sizing = pi_t_sizing
        self.M_out_sizing = M_out_sizing

        self.k_th_RTO=k_th_RTO
        self.k_gb_RTO=k_gb_RTO
        self.k_th_NTO=k_th_NTO
        self.k_gb_NTO=k_gb_NTO
        self.k_th_MCL=k_th_MCL
        self.k_gb_MCL=k_gb_MCL
        self.k_th_MCT=k_th_MCT
        self.k_gb_MCT=k_gb_MCT
        self.k_th_MCR=k_th_MCR
        self.k_gb_MCR=k_gb_MCR
        self.k_th_FID=k_th_FID
        self.k_gb_FID= k_gb_FID
        
        self.k_psfc = k_psfc
        self.k_prop= k_prop

        
        self.ram = Ram() 
        self.inlet = Compression_Nozzle()
        self.lpc = LPC()
        self.hpc = HPC()
        self.combustor = Combustor()
        self.lpt = LPT()
        self.hpt = HPT()
        self.pt = Power_Turbine()
        self.nozzle = Expansion_Nozzle()
        self.thrust = Thrust()
        self.fuel_data = Fuel_data()       


        
    def compute_flight_points(self, flight_points: Union[FlightPoint, pd.DataFrame]):
        # pylint: disable=too-many-arguments  # they define the trajectory

        """
        

        :param mach: Mach number
        :param altitude: (unit=m) altitude w.r.t. to sea level
        :param delta_t4: (unit=K) difference between operational and design values of
                         turbine inlet temperature in K
        :param use_thrust_rate: tells if thrust_rate or thrust should be used (works element-wise)
        :param thrust_rate: thrust rate (unit=none)
        :param thrust: required thrust (unit=N)
        :return: SFC (in kg/s/N), thrust rate, thrust (in N)
        """

        Prop_fid='ADT'
        
            
        
        mach =np.asarray(flight_points.mach)
        altitude=  np.asarray(flight_points.altitude)
        thrust_rate=np.asarray(flight_points.thrust_rate)
        thrust=np.asarray(flight_points.thrust)
        phase = flight_points.engine_setting
        
        atmosphere = Atmosphere(altitude, altitude_in_feet=False)
        
        a = atmosphere.speed_of_sound

        if mach ==0:
            mach = 0.025

        V_TAS = mach*a
        #V_EAS = atmosphere.equivalent_airspeed(V_TAS)/constants.knot
        
        thrust_is_regulated = flight_points.thrust_is_regulated


        #mach= mach*constants.knot/atmosphere.speed_of_sound 

        if thrust_is_regulated is not None:
            thrust_is_regulated = np.asarray(np.round(thrust_is_regulated, 0), dtype=bool)

        
        thrust_is_regulated, thrust_rate, thrust = self._check_thrust_inputs(
            thrust_is_regulated, thrust_rate, thrust
        )
        thrust_is_regulated = np.asarray(np.round(thrust_is_regulated, 0), dtype=bool)
        max_shaft_power,max_thermo_power,Shp_gb_limit,engine_charac, thermo_data = self.compute_power(atmosphere, mach, phase, self.T4) 
        # print(max_shaft_power/constants.hp,max_thermo_power/constants.hp,Shp_gb_limit/constants.hp,engine_charac[0],engine_charac[1]/10)     
        # print(max_shaft_power,max_thermo_power,Shp_gb_limit)
        # max_thrust =self.power_to_thrust(atmosphere, mach,phase,max_shaft_power)+engine_charac[1]
        if phase==1:
            a=1
        
        T_prop, eta =Propeller().select('power_to_thrust',Prop_fid,self,atmosphere, mach,phase, max_shaft_power)
        FR =engine_charac[1]
        max_thrust=T_prop+FR
        # print(phase,altitude,mach,max_thrust,engine_charac[1])

        if not thrust_is_regulated:
            if thrust_rate==1:
                power_rate =1
                shaft_power=max_shaft_power
                thrust=thrust_rate*max_thrust
            else:
                thrust=thrust_rate*max_thrust
                # shaft_power = self.thrust_to_power(atmosphere, mach,phase,thrust)
                shaft_power,eta =Propeller().select('thrust_to_power',Prop_fid,self,atmosphere, mach,phase,thrust)
                # T_prop=shaft_power*self.gearbox_eta*eta/V_TAS
                # FR=thrust-T_prop                         
                power_rate=shaft_power/max_shaft_power
        else:
            power_rate=thrust_rate
            # shaft_power=self.thrust_to_power(atmosphere, mach,phase,thrust)
            shaft_power,eta= Propeller().select('thrust_to_power',Prop_fid,self,atmosphere, mach,phase,thrust)
            # T_prop=shaft_power*self.gearbox_eta*eta/V_TAS
            
        thrust_rate = np.asarray(thrust_rate)
        thrust = np.asarray(thrust)
        
        shaft_power = np.asarray(shaft_power) 
        power_rate = np.asarray(power_rate)
        

        # We compute thrust values from thrust rates when needed
        idx =  np.logical_not(thrust_is_regulated)
        if np.size(max_thrust) == 1:  #se scalare
            maximum_thrust = max_thrust
            maximum_shaft_power= max_shaft_power
            out_thrust_rate = thrust_rate
            out_thrust = thrust
            out_power_rate=power_rate
            out_power=shaft_power
        else:                           #se vettore
            out_thrust_rate = (
                np.full(np.shape(max_thrust), thrust_rate.item())
                if np.size(thrust_rate) == 1
                else thrust_rate
            )
            out_power_rate = (
                np.full(np.shape(max_shaft_power), power_rate.item())
                if np.size(power_rate) == 1
                else power_rate
            )            
            out_thrust = (
                np.full(np.shape(max_thrust), thrust.item()) if np.size(thrust) == 1 else thrust
            )
            out_power = (
                np.full(np.shape(max_shaft_power), shaft_power.item()) if np.size(shaft_power) == 1 else shaft_power
            )

            maximum_thrust = max_thrust[idx]
            maximum_shaft_power =max_shaft_power[idx]

        out_thrust[idx] = out_thrust_rate[idx] * maximum_thrust
        out_power[idx]=out_power_rate[idx]*maximum_shaft_power

        # thrust_rate is obtained from entire thrust vector (could be optimized if needed,
        # as some thrust rates that are computed may have been provided as input)
        out_thrust_rate = out_thrust / max_thrust
        out_power_rate = out_power/max_shaft_power
        

        design_power,thermo_power,gb_limit_power, engine_charac,thermo_data =self.compute_engine_point(atmosphere, mach, phase,out_power)
        if abs(design_power-out_power)>10000:
        #     # if phase.value==2:
        #     #     a=0
        #     # print(design_power/constants.hp)
            print('Warning: compute engine point convergence',phase, altitude,design_power-out_power,'Watt')
        tsfc = engine_charac[0]/out_thrust
        # if phase==3:
        # print(phase,altitude,mach,V_EAS,max_thrust,design_power)

        if phase==1:
            RPS=1200/60
        else:
            RPS=984/60


   
        flight_points.CT = out_thrust/(atmosphere.density*RPS**2*self.d_prop**4)  
        flight_points.psfc = engine_charac[0]/design_power *self.k_psfc
        flight_points.thrust_rate = out_thrust_rate
        flight_points.thrust = out_thrust
        flight_points.TPshaft_power = design_power
        flight_points.TP_power_rate = design_power/max_shaft_power
        flight_points.thermo_power = max_thermo_power    
        
        flight_points.TP_thermal_efficiency = engine_charac[2]
        flight_points.TP_residual_thrust  =engine_charac[1]
        flight_points.TP_air_flow=engine_charac[4]
        flight_points.TP_total_pressure= thermo_data.Pt.values.tolist()
        flight_points.TP_total_temperature  =  thermo_data.Tt.values.tolist()
        flight_points.sfc = tsfc
        
        # return for debug
        # return max_shaft_power,max_thermo_power,Shp_gb_limit,engine_charac, thermo_data
        #return tsfc, out_thrust_rate, out_thrust,out_power,out_power_rate,max_thermo_power

    @staticmethod
    def _check_thrust_inputs(
        thrust_is_regulated: Optional[Union[float, Sequence]],
        thrust_rate: Optional[Union[float, Sequence]],
        thrust: Optional[Union[float, Sequence]],
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Checks that inputs are consistent and return them in proper shape.

        Some of the inputs can be None, but outputs will be proper numpy arrays.

        :param thrust_is_regulated:
        :param thrust_rate:
        :param thrust:
        :return: the inputs, but transformed in numpy arrays.
        """
        # Ensure they are numpy array
        if thrust_is_regulated is not None:
            # As OpenMDAO may provide floats that could be slightly different
            # from 0. or 1., a rounding operation is needed before converting
            # to booleans
            thrust_is_regulated = np.asarray(np.round(thrust_is_regulated, 0), dtype=bool)
        if thrust_rate is not None:
            thrust_rate = np.asarray(thrust_rate)
        if thrust is not None:
            thrust = np.asarray(thrust)

        # Check inputs: if use_thrust_rate is None, we will use the provided input between
        # thrust_rate and thrust
        if thrust_is_regulated is None:
            if thrust_rate is not None:
                thrust_is_regulated = False
                thrust = np.empty_like(thrust_rate)
            elif thrust is not None:
                thrust_is_regulated = True
                thrust_rate = np.empty_like(thrust)
            else:
                raise FastRubberEngineInconsistentInputParametersError(
                    "When use_thrust_rate is None, either thrust_rate or thrust should be provided."
                )

        elif np.size(thrust_is_regulated) == 1:
            # Check inputs: if use_thrust_rate is a scalar, the matching input(thrust_rate or
            # thrust) must be provided.
            if thrust_is_regulated:
                if thrust is None:
                    raise FastRubberEngineInconsistentInputParametersError(
                        "When thrust_is_regulated is True, thrust should be provided."
                    )
                thrust_rate = np.empty_like(thrust)
            else:
                if thrust_rate is None:
                    raise FastRubberEngineInconsistentInputParametersError(
                        "When thrust_is_regulated is False, thrust_rate should be provided."
                    )
                thrust = np.empty_like(thrust_rate)

        else:
            # Check inputs: if use_thrust_rate is not a scalar, both thrust_rate and thrust must be
            # provided and have the same shape as use_thrust_rate
            if thrust_rate is None or thrust is None:
                raise FastRubberEngineInconsistentInputParametersError(
                    "When thrust_is_regulated is a sequence, both thrust_rate and thrust should be "
                    "provided."
                )
            if np.shape(thrust_rate) != np.shape(thrust_is_regulated) or np.shape(
                thrust
            ) != np.shape(thrust_is_regulated):
                raise FastRubberEngineInconsistentInputParametersError(
                    "When use_thrust_rate is a sequence, both thrust_rate and thrust should have "
                    "same shape as use_thrust_rate"
                )

        return thrust_is_regulated, thrust_rate, thrust


    def compute_engine_point(
        self,
        atmosphere: Atmosphere,
        mach: Union[float, Sequence[float]],
        phase: Union[FlightPhase, Sequence],
        shaft_power: Union[float, Sequence[float]],
    ) -> np.ndarray:   
        
        
        tol = 1e-02
        def compute_point(throttle):            
            design_power,thermo_power,gb_limit_power,engine_charac, thermo_data = self.compute_power(atmosphere, mach, phase, throttle*self.T4)
            # if phase==3 or phase==4:
                
            # print('loop',phase,thermo_power,shaft_power,design_power, throttle)
            return thermo_power-shaft_power
        # if phase==1 or phase==2: 
        #     throttle=1.
        # elif phase==3 or phase==4:
        #     throttle=0.85
        # elif
        # sol= root_scalar(compute_point, x0=throttle, x1=k*throttle, xtol=tol,maxiter=10,method='secant')
        # throttle = sol.root
        throttle, infodict,ier,msg= fsolve(compute_point, x0=0.9,full_output=True, xtol=tol)
        
 
        
        design_power,thermo_power,gb_limit_power, engine_charac, thermo_data = self.compute_power(atmosphere, mach, phase, throttle*self.T4)
        # print('last_loop',phase,thermo_power,shaft_power,design_power, throttle)
        #fuel_flow=engine_charac[0]  
        #if phase==3 or phase==5:
          #   print(phase,end-start)
        #return fuel_flow
        return design_power,thermo_power,gb_limit_power, engine_charac,thermo_data

         
           


    def compute_power(
        self,
        atmosphere: Atmosphere,
        mach: Union[float, Sequence[float]],
        phase: Union[FlightPhase, Sequence],
        T4: Union[float, Sequence],
    ) -> np.ndarray:
        """
        Computation of maximum available power.

        Uses model described in 

        :param atmosphere: Atmosphere instance at intended altitude (should be <=20km)
        :param mach: Mach number(s) (should be between 0.05 and 1.0)
        :param phase: flight phase which influences engine rating (max mechanical power)
        :param delta_t4: (unit=K) difference between T4 at flight point and design T4
        
        :return: maximum power (in W)
        """
        '''    TAXI_IN = 0
                TAKEOFF = 1
                CLIMB = 2
                CRUISE = 3
                DESCENT = 4
                LANDING = 6
                TAXI_OUT = 7'''
        altitude = atmosphere.get_altitude(altitude_in_feet=False)
        mach = np.asarray(mach)
        
        if phase == 2: #'MCL' 
            rating = self.k_th_MCL
            Shp_gb_limit = self.k_gb_MCL*self.RTO_power #2001582.729865489#self.RTO_power #
                  
        elif phase == 4:#'FID' 
            rating = self.k_th_FID       
            Shp_gb_limit =  self.k_gb_FID *self.RTO_power # 2001582.729865489#self.RTO_power                  
        elif phase == 1:#'TO' 
            rating =self.k_th_NTO       
            Shp_gb_limit =self.k_gb_NTO *self.RTO_power #2001582.729865489#self.RTO_power
            
        elif phase == 9 :
            rating = self.k_th_MCT  
            Shp_gb_limit =self.k_gb_MCT *self.RTO_power #2001582.729865489# self.RTO_power
            
        elif phase == 3: #'CRZ' 
            rating = self.k_th_MCR
            Shp_gb_limit =self.k_gb_MCR*self.RTO_power #2001582.729865489#self.RTO_power
            self.LP_bleed =0.96
        elif phase == 8:#'RTO' 
            rating = self.k_th_RTO 
            Shp_gb_limit = self.k_gb_RTO*self.RTO_power #2001582.729865489#self.RTO_power #Watt =2400. hp   
                    
        else:
            rating = 1   
            Shp_gb_limit =self.RTO_power #2001582.729865489#self.RTO_power       

        #initial values
        gb_rating =1.
        d_shp=100. 
        #start thermo cycle
        
        

        T4 =  rating*T4*gb_rating      
        tau_b= T4/atmosphere.temperature

            

        ram=self.ram
        ram.compute(atmosphere,mach) 
#        print 'ram', ram.stagnation_temperature,ram.stagnation_pressure
        
    
        #link inlet nozzle to ram 
        inlet=self.inlet    
        inlet.stagnation_temperature_in             = ram.stagnation_temperature 
        inlet.stagnation_pressure_in                = ram.stagnation_pressure
	
        #Flow through the inlet nozzle
        inlet.compute(atmosphere,self.inlet_eta_pol,self.inlet_pressure_ratio)
#        print 'inlet',inlet.stagnation_temperature_out,inlet.stagnation_pressure_out,inlet.stagnation_enthalpy_out
    
        #link low pressure compressor to the inlet nozzle
        lpc = self.lpc
        lpc.stagnation_temperature_in  = inlet.stagnation_temperature_out
        lpc.stagnation_pressure_in     = inlet.stagnation_pressure_out
        lpc.temperature_ratio           = 1 + self.k2*(tau_b/ram.temperature_ratio)
        #Flow through the low pressure compressor
        lpc.compute_offdesign(self.lpc_eta_pol)
#        print 'lpc', lpc.stagnation_temperature_out,lpc.stagnation_pressure_out,lpc.stagnation_enthalpy_out
    
        #link the high pressure compressor to the low pressure compressor
        hpc=self.hpc
        hpc.stagnation_temperature_in = lpc.stagnation_temperature_out
        hpc.stagnation_pressure_in    = lpc.stagnation_pressure_out
        hpc.temperature_ratio          = 1 + self.k1*(tau_b/(ram.temperature_ratio*lpc.temperature_ratio))
        #Flow through the high pressure compressor
        hpc.compute_offdesign(self.hpc_eta_pol)
#        print 'hpc', hpc.stagnation_temperature_out,hpc.stagnation_pressure_out,hpc.stagnation_enthalpy_out
 
        #configure chosen fuel
        fuel = self.fuel_data
        fuel.configure(self.fuel)
        
        #link the combustor to the high pressure compressor
        combustor = self.combustor
        combustor.stagnation_temperature_in                = hpc.stagnation_temperature_out
        combustor.stagnation_pressure_in                   = hpc.stagnation_pressure_out
        combustor.htf                                      = fuel.specific_energy    
        combustor.TIT                                      = T4
        #flow through the high pressor comprresor
        combustor.compute_offdesign(self.combustor_eta,self.combustor_pressure_ratio)
#        print 'combustor', self.TP['T4'],combustor.stagnation_pressure_out,combustor.stagnation_enthalpy_out,combustor.fuel_to_air_ratio


    	    # compute the corresponding fuel flow, air flow and total flow        
        air_flow = self.k0 * atmosphere.pressure*ram.pressure_ratio*self.inlet_pressure_ratio *lpc.pressure_ratio* \
        hpc.pressure_ratio*self.combustor_pressure_ratio /(T4**0.5)
        
        f                                   = combustor.fuel_to_air_ratio
        fuel_flow                           = f * air_flow
        total_flow                          = air_flow * (1 + f)
        
        
        #link the high pressure turbine to the combustor
        hpt = self.hpt
        hpt.stagnation_temperature_in    = T4
        hpt.stagnation_pressure_in       = combustor.stagnation_pressure_out
        hpt.fuel_to_air_ratio            = combustor.fuel_to_air_ratio
        #link the high pressure turbine to the high pressure compressor
        hpt.compressor_work             = hpc.work_done 
        hpt.bleed_offtake                = self.HP_bleed  #=1 solo per verifica ratings
        
        #flow through the high pressure turbine
        hpt.compute(self.Power_Offtake ,self.hpt_eta_mech, self.hpt_eta_pol)
#        print 'hpt',hpt.stagnation_pressure_in/hpt.stagnation_pressure_out #hpt.stagnation_temperature_out,hpt.stagnation_pressure_out,hpt.stagnation_enthalpy_out

        #link the low pressure turbine to the high pressure turbine
        lpt = self.lpt
        lpt.stagnation_temperature_in     = hpt.stagnation_temperature_out
        lpt.stagnation_pressure_in        = hpt.stagnation_pressure_out
        lpt.fuel_to_air_ratio             = combustor.fuel_to_air_ratio
        lpt.bleed_offtake                = self.HP_bleed *self.LP_bleed #=1 solo per verifica ratings
	
	
        #link the low pressure turbine to the low_pressure_compresor
        lpt.compressor_work            = lpc.work_done 

        #flow through the low pressure turbine
        lpt.compute(self.lpt_eta_pol, self.lpt_eta_mech)
#        print 'lpt', lpt.stagnation_pressure_in/lpt.stagnation_pressure_out,lpt.stagnation_temperature_out,lpt.stagnation_pressure_out,lpt.stagnation_enthalpy_out    
        #print('ITT', lpt.stagnation_temperature_out)
    

    
        #link the power turbine to the low pressure turbine
            	#link the power turbine to the low pressure turbine
        pt =self.pt
        pt.fuel_to_air_ratio = f
        pt.stagnation_pressure_in              = lpt.stagnation_pressure_out 
        pt.stagnation_temperature_in           = lpt.stagnation_temperature_out 
        pt.pi_c                                = hpc.pressure_ratio * lpc.pressure_ratio
        pt.pi_d                                = self.inlet_pressure_ratio
        pt.pi_r                                = ram.pressure_ratio
        pt.pi_n                                = self.nozzle_pressure_ratio
        pt.pi_b                                = self.combustor_pressure_ratio
        pt.pi_t                                = lpt.stagnation_pressure_out/lpt.stagnation_pressure_in*hpt.stagnation_pressure_out/hpt.stagnation_pressure_in     
        pt.tau_t                               = lpt.stagnation_temperature_out/lpt.stagnation_temperature_in*hpt.stagnation_temperature_out/hpt.stagnation_temperature_in     
        pt.M9_sizing                           = self.M_out_sizing
        pt.pi_t_sizing                         = self.pi_t_sizing
        pt.tau_t_sizing                        = self.tau_t_sizing     
        pt.bleed_offtake                       = self.HP_bleed *self.LP_bleed #=1 solo per verifica ratings
        
#        power_turb.inputs.throttle                            = throttle
        #flow through the power turbine
        pt.compute_offdesign(self.pt_eta_pol, self.pt_eta_mech)
#        print 'pt',pt.stagnation_pressure_in/pt.stagnation_pressure_out ,pt.stagnation_temperature_out,pt.stagnation_pressure_out,pt.stagnation_enthalpy_out
        
        #link the expansion nozzle to the power turbine 
        nozzle = self.nozzle
        nozzle.stagnation_temperature_in          = pt.stagnation_temperature_out        
        nozzle.stagnation_pressure_in             = pt.stagnation_pressure_out
        nozzle.M_out                              = pt.M_out_nozzle                   
        #flow through theexpansion nozzle
        nozzle.compute_offdesign(self.nozzle_eta_pol,self.nozzle_pressure_ratio) 
#        print 'nozzle', nozzle.stagnation_temperature_out,nozzle.stagnation_pressure_out, nozzle.M_out    

        #link low-pressure turbine output to thrust component
        thrust = self.thrust
        thrust.fuel_to_air_ratio                                = f
        thrust.Tt_PTin                                          = lpt.stagnation_temperature_out
        thrust.Tt_PTout                                         = pt.stagnation_temperature_out
        thrust.u_out                                            = nozzle.velocity_out
        
        #compute the thrust
        thrust.compute_offdesign(atmosphere, mach, self.HP_bleed, self.LP_bleed, self.pt_eta_mech)            
  
        #pack the output values
        shaft_power_adim                                     = pt.shaft_takeoff 
        jet_power_adim                                       = thrust.jet_power
        residual_thrust_adim                                 = thrust.residual_thrust_nd
                
#        design_SHP = shaft_power_adim * air_flow  # in watt
        thermo_power =  shaft_power_adim * air_flow  # in watt
        residual_thrust = residual_thrust_adim * air_flow

#            print 'phase',phase            
#            print 'gb_rating',gb_rating
#            print 'Shp_thermo',Shp_thermo
#            print 'Shp_gb_limit',Shp_gb_limit
#        print Mach,Alt,lpc.stagnation_pressure_out/lpc.stagnation_pressure_in, hpc.stagnation_pressure_out/hpc.stagnation_pressure_in,hpc.stagnation_pressure_out,air_flow,f
        design_power = thermo_power
        # print('ITT', lpt.stagnation_temperature_out)
        # calculation of thermal efficiency
        thermal_power = fuel_flow * fuel.specific_energy
        ESHP = shaft_power_adim*air_flow + jet_power_adim*air_flow
        thermal_efficiency = (pt.shaft_takeoff*air_flow+jet_power_adim*air_flow)/ thermal_power
        air_bleed = air_flow - self.HP_bleed *self.LP_bleed*air_flow
        # print(phase,air_bleed)
        #print('rating',thermo_power/constants.hp,fuel_flow*60*60/constants.lb/(ESHP/constants.hp),ESHP/constants.hp,fuel_flow, lpt.stagnation_temperature_out,residual_thrust/10)
        if design_power>Shp_gb_limit:
            design_power=Shp_gb_limit
        #print(design_power/constants.hp)
    
        thermo_dict = {'Tt': [ram.stagnation_pressure.item() ,
                              inlet.stagnation_pressure_out.item(),
                              lpc.stagnation_temperature_out.item(),
                              hpc.stagnation_temperature_out.item(),
                              combustor.stagnation_temperature_out.item(),
                              hpt.stagnation_temperature_out.item(),
                              lpt.stagnation_temperature_out.item(),
                              pt.stagnation_temperature_out.item(),
                              nozzle.stagnation_temperature_out.item()
                              ],
                        'Pt':[ram.stagnation_pressure.item() ,
                              inlet.stagnation_pressure_out.item(),
                              lpc.stagnation_pressure_out.item(),
                              hpc.stagnation_pressure_out.item() ,
                              combustor.stagnation_pressure_out.item(),
                              hpt.stagnation_pressure_out.item(),
                              lpt.stagnation_pressure_out.item() ,
                              pt.stagnation_pressure_out.item() ,
                              nozzle.stagnation_pressure_out.item()
                              ] }          
        
        thermo_data= pd.DataFrame(data=thermo_dict)  
            
            
       
        
        
        
        
        return (design_power,thermo_power,Shp_gb_limit,[fuel_flow,residual_thrust,thermal_efficiency,ESHP, air_flow],thermo_data)


    




       