"""Parametric parallel hybrid fuel cell engine."""
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
# from sklearn.preprocessing import PolynomialFeatures
from scipy import constants
import logging
import math
from typing import Union, Sequence, Tuple, Optional
import os

import numpy as np
from fastoad.constants import FlightPhase
#from fastoad.models.propulsion import IPropulsion
#from models.propulsion import IPropulsion
#from fastoad.models.propulsion.fuel_propulsion.rubber_engine.exceptions import (
#   FastRubberEngineInconsistentInputParametersError,
#)
from fastoad.model_base.atmosphere import Atmosphere
import pandas as pd
from fastoad.model_base.flight_point import FlightPoint
#from models.propulsion.fuel_engine.hybrid_engine import AbstractHybridPropulsion
from .base import AbstractHybridPropulsion
# Logger for this module
_LOGGER = logging.getLogger(__name__)

#from models.propulsion.fuel_engine.hybrid_engine.engine_components.Battery_L0 import Battery
from .engine_components.ElectricMotor import ElectricMotor
from .engine_components.Fuel_cell_L0 import Fuel_cell
from .engine_components.IDC import IDC
from ..turboprop_engine.TP_engine_L1 import TPEngine_L1
from ..turboprop_engine.engine_components.Propeller import Propeller
#from fastoad.base.dict import AddKeyAttributes

#AddKeyAttributes(["psfc","shaft_power", "power_rate","thermo_power","TP_thermal_efficiency","TP_residual_thrust","TP_air_flow","TP_total_pressure","TP_total_temperature","fuel_mass"
                 # ,"H2_mass","TPshaft_power","EMshaft_power","FC_power","TP_power_rate","EM_power_rate","H2_fc","FC_efficiency","FC_airflow","V_cell","V_stack","V_pack","V_core","I_cell","I_stack","I_pack","I_core","BAT_ec","BAT_power","CT"])(FlightPoint)


Prop_fid='ADT'


class PHFCEngine_L1(AbstractHybridPropulsion):
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
        
        Elec_nom_power:float,
        power_electronics_eta: float,
        motor_eta:float,
        # battery_eta: float,
        # fuel_cell_eta: float
        V_vec: list,
        i_vec: list,
        Nstacks: float,
        Npacks:float,
        Acell:float, 
        Ncells:float,    
        Nmotors:float,
        FC_Power_Offtake: float,
        Gross_net_power_ratio:float,
    ):
        """
        Parametric parallel hybrid fuel cell engine.

        It computes engine characteristics using analytical models.

        :param bypass_ratio:
        :param overall_pressure_ratio:
        :param turbine_inlet_temperature: (unit=K) also noted T4
        :param mto_thrust: (unit=N) Maximum TakeOff thrust, i.e. maximum thrust
                           on ground at speed 0, also noted F0
        :param maximum_mach:
        :param design_altitude: (unit=m)
        :param delta_t4_climb: (unit=K) difference between T4 during climb and design T4
        :param delta_t4_cruise: (unit=K) difference between T4 during cruise and design T4
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
        
        self.k0 =k0
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
        
        self.Elec_nom_power = Elec_nom_power
        self.power_electronics_eta = power_electronics_eta
        self.motor_eta = motor_eta
        #self.battery_eta = battery_eta
        # self.fuel_cell_eta = fuel_cell_eta
        
        #self.battery= Battery(battery_eta)
        self.fuel_cell = Fuel_cell( V_vec,
                                    i_vec,
                                    Nstacks,
                                    Npacks,
                                    Acell, 
                                    Ncells,
                                    Nmotors)
        self.motor = ElectricMotor(motor_eta)
        self.power_electronics = IDC(power_electronics_eta)
        self.turbine = TPEngine_L1( RTO_power,
                                    Design_Thermo_Power,
                                    Power_Offtake,
                                    gearbox_eta,
                                    d_prop,
                                    fuel,                                    
                                    turbine_inlet_temperature,
                                    HP_bleed,
                                    LP_bleed,                                   
                                    inlet_eta_pol,
                                    inlet_pressure_ratio,
                                    lpc_eta_pol,
                                    lpc_pressure_ratio,                                   
                                    hpc_eta_pol,
                                    hpc_pressure_ratio,                                  
                                    combustor_eta,
                                    combustor_pressure_ratio,                                   
                                    hpt_eta_pol,
                                    hpt_eta_mech,                           
                                    lpt_eta_pol,
                                    lpt_eta_mech,                          
                                    pt_eta_pol,
                                    pt_eta_mech,
                                    nozzle_eta_pol,
                                    nozzle_pressure_ratio,
                                    nozzle_area_ratio,                               
                                    k0,
                                    k1,
                                    k2,
                                    tau_t_sizing,
                                    pi_t_sizing,
                                    M_out_sizing,
                                    k_th_RTO,
                                    k_gb_RTO,
                                    k_th_NTO,
                                    k_gb_NTO,
                                    k_th_MCL,
                                    k_gb_MCL,
                                    k_th_MCT,
                                    k_gb_MCT,
                                    k_th_MCR,
                                    k_gb_MCR,
                                    k_th_FID,
                                    k_gb_FID,
                                    k_psfc,
                                    k_prop)
        self.motor_count=Nmotors
        self.Gross_net_power_ratio =Gross_net_power_ratio
        self.FC_Power_Offtake =FC_Power_Offtake



    def compute_flight_points(self, flight_points: Union[FlightPoint, pd.DataFrame]):
        # pylint: disable=too-many-arguments  # they define the trajectory

        """
        Same as :meth:`compute_flight_points` except that delta_t4 is used directly
        instead of specifying flight phase.

        :param mach: Mach number
        :param altitude: (unit=m) altitude w.r.t. to sea level
        :param delta_t4: (unit=K) difference between operational and design values of
                         turbine inlet temperature in K
        :param use_thrust_rate: tells if thrust_rate or thrust should be used (works element-wise)
        :param thrust_rate: thrust rate (unit=none)
        :param thrust: required thrust (unit=N)
        :return: SFC (in kg/s/N), thrust rate, thrust (in N)
        """
        mach =np.asarray(flight_points.mach)
        altitude=  np.asarray(flight_points.altitude)
        thrust_rate=np.asarray(flight_points.thrust_rate)
        thrust=np.asarray(flight_points.thrust)
        phase = flight_points.engine_setting
        thrust_is_regulated = flight_points.thrust_is_regulated

        atmosphere = Atmosphere(altitude, altitude_in_feet=False)
        # if phase==4:
        #     a = atmosphere.speed_of_sound
        #     V_TAS = mach*a
        #     V_EAS = atmosphere.get_equivalent_airspeed(V_TAS)
        #     print(phase,altitude,V_EAS)        


        if thrust_is_regulated is not None:
            thrust_is_regulated = np.asarray(np.round(thrust_is_regulated, 0), dtype=bool)

        
        thrust_is_regulated, thrust_rate, thrust = self.turbine._check_thrust_inputs(
            thrust_is_regulated, thrust_rate, thrust
        )
        thrust_is_regulated = np.asarray(np.round(thrust_is_regulated, 0), dtype=bool)
        
        #Compute max available power and thrust
        max_shaft_power,max_thermo_power,max_motor_power,max_available_power,gearbox_limit_power,max_TPshaft_power,engine_charac= self.max_power(atmosphere, mach, phase)
        # max_thrust =self.turbine.power_to_thrust(atmosphere, mach,phase,max_shaft_power)
        # max_TPthrust=self.turbine.power_to_thrust(atmosphere, mach,phase,max_TPshaft_power)+engine_charac[1]


            
        max_thrust, eta = Propeller().select('power_to_thrust',Prop_fid,self,atmosphere, mach,phase, max_shaft_power)
        max_TPthrust, eta =Propeller().select('power_to_thrust',Prop_fid,self,atmosphere, mach,phase, max_TPshaft_power)
        max_TPthrust += engine_charac[1] 
        max_thrust += engine_charac[1] 
        
        #Hybridization strategy: compute power contribution from turbine and electric motor
        thrust, shaft_power, motor_power, turbine_power, power_rate, TP_power_rate, EM_power_rate,thrust_rate = self.Constant_elec_power_Variable_TTC(atmosphere, mach, flight_points,max_shaft_power,
                                                                                                                                         max_thermo_power,max_motor_power,max_available_power,
                                                                                                                                         gearbox_limit_power,max_TPshaft_power,thrust_rate,thrust,max_thrust,max_TPthrust,thrust_is_regulated)
                                                                    
        '''# We compute thrust values from thrust rates when needed
        idx =  np.logical_not(thrust_is_regulated)
        if np.size(max_thrust) == 1:  #se scalare
            maximum_thrust = max_thrust
            maximum_shaft_power= max_shaft_power
            maximum_TPshaft_power= max_TPshaft_power
            out_thrust_rate = thrust_rate
            out_thrust = thrust
            out_power_rate=power_rate
            out_TP_power_rate=TP_power_rate
            out_power=shaft_power
            out_turbine_power= turbine_power


        out_thrust[idx] = out_thrust_rate[idx] * maximum_thrust
        # print(flight_points.name)
        out_power[idx]=out_power_rate[idx]*maximum_shaft_power

        out_turbine_power[idx]=out_TP_power_rate[idx]*max_TPshaft_power

        # thrust_rate is obtained from entire thrust vector (could be optimized if needed,
        # as some thrust rates that are computed may have been provided as input)
        out_thrust_rate = out_thrust / max_thrust
        out_power_rate = out_power/max_shaft_power   
        out_TP_power_rate=out_turbine_power/max_TPshaft_power

        if flight_points.name=='diversion cruise':
            a=1
            print(out_turbine_power)
            if out_turbine_power<70000:
                a=1'''
            
        # Compute turbine SFC
        # design_power,thermo_power,gb_limit_power, engine_charac,thermo_data =self.turbine.compute_engine_point(atmosphere, mach, phase,out_turbine_power)
        # if abs(design_power-out_turbine_power)>10000:
        #     print('Warning: compute engine point convergence',flight_points.name ,phase.value, altitude,design_power-out_turbine_power,'Watt')
        design_power,thermo_power,gb_limit_power, engine_charac,thermo_data =self.turbine.compute_engine_point(atmosphere, mach, phase,turbine_power)
        if abs(design_power-turbine_power)>10000:
            print('Warning: compute engine point convergence',flight_points.name ,phase.value, altitude,design_power-turbine_power,'Watt')
  
        
        # Compute Hydrogen consumption
        motor_power_in = self.motor.get_motor_power(motor_power,'downstream')
        power_electronics_power =motor_power_in
        power_electronics_power_in = self.power_electronics.get_idc_power(power_electronics_power, power_flow='downstream')
        fuel_cell_power = power_electronics_power_in
        fc_stack_power= fuel_cell_power/self.Gross_net_power_ratio + self.FC_Power_Offtake
        fc_perfo = self.fuel_cell.get_fc_perfo(fc_stack_power) #kg/s        

        
        a = atmosphere.speed_of_sound
        V_TAS = mach*a
        V_EAS = atmosphere.get_equivalent_airspeed(V_TAS)/constants.knot
        # print(phase,altitude,mach,V_EAS)        
        # print(thrust/max_thrust,TP_power_rate,turbine_power/constants.hp,motor_power/constants.hp)
        if phase==1:
            RPS=1200/60
        else:
            RPS=984/60
            
           
        flight_points.CT = thrust/(atmosphere.density*RPS**2*self.d_prop**4)  
        # flight_points.psfc = engine_charac[0]/out_turbine_power*self.k_psfc
        # flight_points.thrust_rate = out_thrust_rate
        # flight_points.thrust = out_thrust
        # flight_points.TPshaft_power = out_turbine_power
        # flight_points.TP_power_rate = out_TP_power_rate
        flight_points.psfc = engine_charac[0]/turbine_power*self.k_psfc
        flight_points.thrust_rate = thrust_rate
        flight_points.thrust = thrust
        flight_points.TPshaft_power = turbine_power
        flight_points.TP_power_rate = TP_power_rate        
        flight_points.thermo_power = max_thermo_power
        
        flight_points.TP_thermal_efficiency = engine_charac[2]
        flight_points.TP_residual_thrust  =engine_charac[1]
        flight_points.TP_air_flow=engine_charac[4]
        flight_points.TP_total_pressure= thermo_data.Pt.values.tolist()
        flight_points.TP_total_temperature  =  thermo_data.Tt.values.tolist()  
        
        # flight_points.shaft_power = out_power
        # flight_points.power_rate =out_power_rate 
        flight_points.shaft_power = shaft_power
        flight_points.power_rate =power_rate 
        flight_points.EMshaft_power = motor_power
        flight_points.FC_power = fuel_cell_power
        flight_points.FC_stack_power = fc_stack_power
        #flight_points.BAT_power = 
        
        flight_points.EM_power_rate = EM_power_rate 
        flight_points.H2_fc = fc_perfo[0]
        flight_points.FC_efficiency=fc_perfo[1]
        flight_points.FC_airflow=fc_perfo[2]
        flight_points.V_cell=fc_perfo[3]
        flight_points.V_stack=fc_perfo[4]
        flight_points.V_pack=fc_perfo[5]
        flight_points.V_core=fc_perfo[6]
        flight_points.I_cell=fc_perfo[7]
        flight_points.I_stack=fc_perfo[8]
        flight_points.I_pack=fc_perfo[9]
        flight_points.I_core=fc_perfo[10]
        
        
        #flight_points.BAT_SOC = 
        
        #return tsfc, out_thrust_rate, out_thrust,out_power,out_power_rate,max_thermo_power


    def Constant_elec_power_Constant_TTC(
        self,
        atmosphere: Atmosphere,
        mach: Union[float, Sequence[float]],
        flight_phase: Union[FlightPhase, Sequence],  
        max_shaft_power: Union[float, Sequence[float]],
        max_thermo_power: Union[float, Sequence[float]],
        max_motor_power: Union[float, Sequence[float]],
        max_available_power: Union[float, Sequence[float]],
        gearbox_limit_power : Union[float, Sequence[float]], 
        max_TPshaft_power: Union[float, Sequence[float]], 
        thrust_rate: Union[float, Sequence[float]],
        thrust: Union[float, Sequence[float]],
        max_thrust: Union[float, Sequence[float]],
        max_TPthrust: Union[float, Sequence[float]],
        thrust_is_regulated:  Union[float, Sequence[float]]
            )-> np.ndarray:
        """
        Hybridization strategy with constant electrical power during the hybrid segments (climb and cruise). 
        The gasturbine throttle is derived so to have the total shaft power (elec+turbine) exactly equal to the available turbine power 
        of the turbine at each flight point: this results in the same power profile along the mission, 
        therefore the same TTC as the conventional aircraft.
        """
        phase = flight_phase.engine_setting
        if phase==1 or phase==2 or phase==9:  
            if 'diversion' in str(flight_phase.name):
                power_rate =[1]
                shaft_power=max_TPshaft_power                              
                thrust=thrust_rate*max_TPthrust
                turbine_power= shaft_power
                motor_power = 0.  
            elif 'holding' in str(flight_phase.name):
                power_rate=thrust_rate
                # shaft_power=self.turbine.thrust_to_power(atmosphere, mach,phase,thrust)
                shaft_power, eta = Propeller().select('thrust_to_power',Prop_fid,self,atmosphere, mach,phase,thrust)
                motor_power= 0.
                turbine_power= shaft_power-motor_power
                
            else:                
                shaft_power=max_TPshaft_power
                thrust=thrust_rate*max_TPthrust
                motor_power= max_motor_power
                turbine_power= shaft_power-motor_power 
                power_rate =  shaft_power/max_available_power 
                if max_available_power>gearbox_limit_power:
                    power_rate = shaft_power/gearbox_limit_power #so power_rate will be 1 when we are at the gb_limit_power
            
        
        elif phase==4 : #descent
            thrust=thrust_rate*max_TPthrust
            # shaft_power = self.turbine.thrust_to_power(atmosphere, mach,phase,thrust)
            shaft_power, eta = Propeller().select('thrust_to_power',Prop_fid,self,atmosphere, mach,phase,thrust)
            power_rate=shaft_power/max_available_power
            turbine_power= shaft_power
            motor_power = 0.
            
        elif phase==3: #cruise
            if 'diversion' in str(flight_phase.name):
                power_rate=thrust_rate
                # shaft_power=self.turbine.thrust_to_power(atmosphere, mach,phase,thrust)
                shaft_power, eta = Propeller().select('thrust_to_power',Prop_fid,self,atmosphere, mach,phase,thrust)
                motor_power= 0.
                turbine_power= shaft_power-motor_power
            else:
                power_rate=thrust_rate
                # shaft_power=self.turbine.thrust_to_power(atmosphere, mach,phase,thrust)
                shaft_power, eta = Propeller().select('thrust_to_power',Prop_fid,self,atmosphere, mach,phase,thrust)
                motor_power= max_motor_power
                turbine_power= shaft_power-motor_power  
   
                
                 
        thrust_rate = np.asarray(thrust_rate)
        thrust = np.asarray(thrust)
        
        shaft_power = np.asarray(shaft_power) 
        power_rate = np.asarray(power_rate)
        
        
        
        TP_power_rate = np.asarray(turbine_power/max_thermo_power)
        EM_power_rate = np.asarray(motor_power/max_motor_power)
        
             
        return thrust, shaft_power, motor_power, turbine_power, power_rate, TP_power_rate, EM_power_rate

    def Constant_elec_power_Variable_TTC(
        self,
        atmosphere: Atmosphere,
        mach: Union[float, Sequence[float]],
        flight_phase: Union[FlightPhase, Sequence],  
        max_shaft_power: Union[float, Sequence[float]],
        max_thermo_power: Union[float, Sequence[float]],
        max_motor_power: Union[float, Sequence[float]],
        max_available_power: Union[float, Sequence[float]],
        gearbox_limit_power : Union[float, Sequence[float]], 
        max_TPshaft_power: Union[float, Sequence[float]],                 
        thrust_rate: Union[float, Sequence[float]],
        thrust: Union[float, Sequence[float]],
        max_thrust: Union[float, Sequence[float]], 
        max_TPthrust: Union[float, Sequence[float]],  
        thrust_is_regulated:  Union[float, Sequence[float]]
            )-> np.ndarray:
        """
        Hybridization strategy with constant electrical power (throttle=1) during the hybrid segments (climb and cruise). 
        The gasturbine throttle is also 1. TTC is then function of the max available power from gasturbine and electric motor.
        """
        phase = flight_phase.engine_setting
        hybrid_reserves=False
            
        if phase==1 or phase==2 or phase==9:     
            if not hybrid_reserves:
                if 'diversion' in str(flight_phase.name):
                    power_rate =[1]
                    shaft_power=max_TPshaft_power                              
                    thrust=thrust_rate*max_TPthrust
                    turbine_power= shaft_power
                    motor_power = 0.  
                # elif 'holding' in str(flight_phase.name):
                #     power_rate=thrust_rate
                #     # shaft_power=self.turbine.thrust_to_power(atmosphere, mach,phase,thrust)
                #     shaft_power, eta = Propeller().select('thrust_to_power',Prop_fid,self,atmosphere, mach,phase,thrust)
                #     motor_power= 0.
                #     turbine_power= shaft_power-motor_power            
                else:
                    shaft_power=max_shaft_power
                    # print('thrust_rate ',thrust_rate)
                    thrust=thrust_rate*max_thrust
                    motor_power= max_motor_power
                    turbine_power= max_shaft_power-motor_power 
                    power_rate =  shaft_power/max_available_power 
                    if max_available_power>gearbox_limit_power:
                        power_rate = shaft_power/gearbox_limit_power #so power_rate will be 1 when we are at the gb_limit_power
            else:
                    shaft_power=max_shaft_power
                    # print('thrust_rate ',thrust_rate)
                    thrust=thrust_rate*max_thrust
                    motor_power= max_motor_power
                    turbine_power= max_shaft_power-motor_power 
                    power_rate =  shaft_power/max_available_power 
                    if max_available_power>gearbox_limit_power:
                        power_rate = shaft_power/gearbox_limit_power #so power_rate will be 1 when we are at the gb_limit_power
                
                
                
                
                
        # elif phase==4 : #descent
        #     thrust=thrust_rate*max_TPthrust
        #     # shaft_power = self.turbine.thrust_to_power(atmosphere, mach,phase,thrust)
        #     shaft_power, eta = Propeller().select('thrust_to_power',Prop_fid,self,atmosphere, mach,phase,thrust)
        #     power_rate=shaft_power/max_available_power
        #     turbine_power= shaft_power
        #     motor_power = 0.
            
        elif phase==3: 
            if not hybrid_reserves:
                if 'diversion' in str(flight_phase.name):
                    power_rate=thrust_rate
                    # shaft_power=self.turbine.thrust_to_power(atmosphere, mach,phase,thrust)
                    shaft_power, eta = Propeller().select('thrust_to_power',Prop_fid,self,atmosphere, mach,phase,thrust)
                    motor_power= 0.
                    turbine_power= shaft_power-motor_power
                    
                # elif str(flight_phase.name)== 'descent':
                #     thrust=thrust_rate*max_TPthrust
                #     # shaft_power = self.turbine.thrust_to_power(atmosphere, mach,phase,thrust)
                #     shaft_power, eta = Propeller().select('thrust_to_power',Prop_fid,self,atmosphere, mach,phase,thrust)
                #     power_rate=shaft_power/max_available_power
                #     motor_power = 0.
                #     turbine_power= shaft_power-motor_power 
                elif 'descent' in str(flight_phase.name):
                    ##### comment section if descent  with fixed thrust_rate
                    if thrust_is_regulated:
                        thrust_rate=thrust/max_TPthrust 
                    else:
                        thrust=thrust_rate*max_TPthrust
                    #####    
                    thrust=thrust_rate*max_TPthrust    
                    shaft_power, eta = Propeller().select('thrust_to_power',Prop_fid,self,atmosphere, mach,phase,thrust)
                    power_rate=shaft_power/max_available_power
                    motor_power = 0.
                    turbine_power= shaft_power-motor_power                     
                                    
                elif 'holding' in str(flight_phase.name):
                    power_rate=thrust_rate
                    # shaft_power=self.turbine.thrust_to_power(atmosphere, mach,phase,thrust)
                    shaft_power, eta = Propeller().select('thrust_to_power',Prop_fid,self,atmosphere, mach,phase,thrust)
                    motor_power= 0.
                    turbine_power= shaft_power-motor_power                       
                else:
                    power_rate=thrust_rate
                    # shaft_power=self.turbine.thrust_to_power(atmosphere, mach,phase,thrust)
                    shaft_power, eta = Propeller().select('thrust_to_power',Prop_fid,self,atmosphere, mach,phase,thrust)
                    motor_power= max_motor_power
                    turbine_power= shaft_power-motor_power 
            else:
                power_rate=thrust_rate
                # shaft_power=self.turbine.thrust_to_power(atmosphere, mach,phase,thrust)
                shaft_power, eta = Propeller().select('thrust_to_power',Prop_fid,self,atmosphere, mach,phase,thrust)
                motor_power= max_motor_power
                turbine_power= shaft_power-motor_power        
                if motor_power>shaft_power:
                    print('Error: nominal electric power too high for cruise',flight_phase.name)
                    motor_power=shaft_power
                    turbine_power=100000.
                
            thrust_rate = np.asarray(thrust_rate)
            thrust = np.asarray(thrust)   
            idx =  np.logical_not(thrust_is_regulated)
            if np.size(max_TPthrust) == 1:  #se scalare
                maximum_thrust = max_TPthrust
                out_thrust_rate = thrust_rate
                out_thrust = thrust
                # maximum_thrust = max_TPthrust[idx]
            
            out_thrust[idx] = out_thrust_rate[idx] * maximum_thrust

            out_thrust_rate = out_thrust / max_TPthrust
            thrust=out_thrust
            thrust_rate=out_thrust_rate   
        
        
        
        
        thrust_rate = np.asarray(thrust_rate)
        thrust = np.asarray(thrust)
        
        shaft_power = np.asarray(shaft_power) 
        power_rate = np.asarray(power_rate)
        
        TP_power_rate = np.asarray(turbine_power/max_thermo_power)
        EM_power_rate = np.asarray(motor_power/max_motor_power)
        
             
        return thrust, shaft_power, motor_power, turbine_power, power_rate, TP_power_rate, EM_power_rate, thrust_rate


        
    def max_power(
        self,
        atmosphere: Atmosphere,
        mach: Union[float, Sequence[float]],
        phase: Union[FlightPhase, Sequence],
    ) -> np.ndarray:
        """
        Computation of maximum available power from all power sources.

        Uses model described in :cite:`roux:2005`, p.57-58

        :param atmosphere: Atmosphere instance at intended altitude (should be <=20km)
        :param mach: Mach number(s) (should be between 0.05 and 1.0)
        :param phase: flight phase which influences engine rating (max mechanical power)
        :return: maximum power (in W)
        """
        min_RTO=1201582.729865489
        if phase == 2: #'MCL' 
            Shp_gb_limit = self.k_gb_MCL*(max(self.RTO_power,min_RTO) + self.Elec_nom_power)#2001582.729865489#self.RTO_power #
                  
        elif phase == 4:#'FID' 
            Shp_gb_limit =  self.k_gb_FID *(max(self.RTO_power,min_RTO) + self.Elec_nom_power)# 2001582.729865489#self.RTO_power                  
        elif phase == 1:#'TO' 
            Shp_gb_limit =self.k_gb_NTO *(max(self.RTO_power,min_RTO)+ self.Elec_nom_power) #2001582.729865489#self.RTO_power
            
        elif phase == 9 :
            Shp_gb_limit =self.k_gb_MCT *(max(self.RTO_power,min_RTO) + self.Elec_nom_power) #2001582.729865489# self.RTO_power
            
        elif phase == 3: #'CRZ' 
            Shp_gb_limit =self.k_gb_MCR*(max(self.RTO_power,min_RTO) + self.Elec_nom_power) #2001582.729865489#self.RTO_power
            self.LP_bleed =0.96
        elif phase == 8:#'RTO' 
            Shp_gb_limit = self.k_gb_RTO*(max(self.RTO_power,min_RTO) + self.Elec_nom_power) #2001582.729865489#self.RTO_power #Watt =2400. hp   
                    
        else:
            rating = 1   
            Shp_gb_limit =(max(self.RTO_power,min_RTO) + self.Elec_nom_power) #2001582.729865489#self.RTO_power       

        altitude = atmosphere.get_altitude(altitude_in_feet=False)
        mach = np.asarray(mach)

  

        max_TPshaft_power,max_thermo_power, gearbox_limit_power,engine_charac, thermo_data=self.turbine.compute_power(atmosphere, mach, phase, self.T4)     
        
        max_motor_power = self.Elec_nom_power
        
        max_available_power = max_motor_power + max_thermo_power
            

        if max_available_power>Shp_gb_limit:
            # print(phase,Shp_gb_limit,self.RTO_power,self.Elec_nom_power)
            max_shaft_power=Shp_gb_limit  #gearbox mechanical limit 
        else:
            max_shaft_power=max_available_power
            
        return max_shaft_power,max_thermo_power,max_motor_power,max_available_power,Shp_gb_limit,max_TPshaft_power,engine_charac
    



