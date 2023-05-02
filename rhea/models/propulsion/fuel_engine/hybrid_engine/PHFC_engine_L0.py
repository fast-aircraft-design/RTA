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
import pickle

from sklearn.preprocessing import PolynomialFeatures
from scipy import constants
import logging
import math
from typing import Union, Sequence, Tuple, Optional
import os

import numpy as np
from fastoad.constants import FlightPhase
from fastoad.models.propulsion import IPropulsion
#from models.propulsion import IPropulsion
from fastoad.models.propulsion.fuel_propulsion.rubber_engine.exceptions import (
    FastRubberEngineInconsistentInputParametersError,
)
from fastoad.utils.physics import Atmosphere
import pandas as pd
from fastoad.base.flight_point import FlightPoint
#from models.propulsion.fuel_engine.hybrid_engine import AbstractHybridPropulsion
from models.propulsion.fuel_engine.hybrid_engine.base import AbstractHybridPropulsion
# Logger for this module
_LOGGER = logging.getLogger(__name__)

#from models.propulsion.fuel_engine.hybrid_engine.engine_components.Battery_L0 import Battery
from models.propulsion.fuel_engine.hybrid_engine.engine_components.ElectricMotor import ElectricMotor
from models.propulsion.fuel_engine.hybrid_engine.engine_components.Fuel_cell_L0 import Fuel_cell
from models.propulsion.fuel_engine.hybrid_engine.engine_components.IDC import IDC
from models.propulsion.fuel_engine.turboprop_engine.TP_engine_L0 import TPEngine_L0
from models.propulsion.fuel_engine.turboprop_engine.engine_components.Propeller import Propeller
from fastoad.base.dict import AddKeyAttributes

AddKeyAttributes(["psfc","shaft_power", "power_rate","thermo_power","TP_thermal_efficiency","TP_residual_thrust","TP_air_flow","TP_total_pressure","TP_total_temperature","fuel_mass" 
                  ,"H2_mass","TPshaft_power","EMshaft_power","FC_power","TP_power_rate","EM_power_rate","H2_fc","FC_efficiency","FC_airflow","V_cell","V_stack","V_pack","V_core","I_cell","I_stack","I_pack","I_core","BAT_ec","BAT_power","CT"])(FlightPoint)

Prop_fid='ADT'

class PHFCEngine_L0(AbstractHybridPropulsion):
    def __init__(
        self,
        RTO_power: float,
        Design_Thermo_Power: float,
        Power_Offtake: float,
        gearbox_eta: float,
        d_prop:float,

                
        k_gb_RTO: float,
        k_gb_NTO: float,
        k_gb_MCL: float,
        k_gb_MCT: float,
        k_gb_MCR: float,
        k_gb_FID: float, 
        
        k_psfc:float,
        k_prop:float,        
        Elec_nom_power:float,
        power_electronics_eta: float,
        motor_eta:float,
        # battery_eta: float,
        fuel_cell_eta: float,
        Nmotors:float,
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
        
        
        self.k_gb_RTO=k_gb_RTO
        self.k_gb_NTO=k_gb_NTO
        self.k_gb_MCL=k_gb_MCL
        self.k_gb_MCT=k_gb_MCT
        self.k_gb_MCR=k_gb_MCR
        self.k_gb_FID= k_gb_FID
        
        self.k_psfc = k_psfc
        self.k_prop= k_prop
        
        self.Elec_nom_power = Elec_nom_power
        self.power_electronics_eta = power_electronics_eta
        self.motor_eta = motor_eta
        #self.battery_eta = battery_eta
        self.fuel_cell_eta = fuel_cell_eta
        
        #self.battery= Battery(battery_eta)
        self.fuel_cell = Fuel_cell(fuel_cell_eta)
        self.motor = ElectricMotor(motor_eta)
        self.power_electronics = IDC(power_electronics_eta)
        self.turbine = TPEngine_L0(RTO_power, Design_Thermo_Power, Power_Offtake, gearbox_eta,d_prop , k_gb_RTO,
        k_gb_NTO,
        k_gb_MCL,
        k_gb_MCT,
        k_gb_MCR,
        k_gb_FID, 
        k_psfc,
        k_prop)
        
        self.motor_count=Nmotors



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
        

        if thrust_is_regulated is not None:
            thrust_is_regulated = np.asarray(np.round(thrust_is_regulated, 0), dtype=bool)

        
        thrust_is_regulated, thrust_rate, thrust = self.turbine._check_thrust_inputs(
            thrust_is_regulated, thrust_rate, thrust
        )
        thrust_is_regulated = np.asarray(np.round(thrust_is_regulated, 0), dtype=bool)
        
        #Compute max available power and thrust
        max_shaft_power,max_thermo_power,max_motor_power,max_available_power,gearbox_limit_power,max_TPshaft_power = self.max_power(atmosphere, mach, phase)
        FR=0
        
        # max_thrust =self.turbine.power_to_thrust(atmosphere, mach,phase,max_shaft_power)
        # max_TPthrust=self.turbine.power_to_thrust(atmosphere, mach,phase,max_TPshaft_power)
        
        max_thrust, eta = Propeller().select('power_to_thrust',Prop_fid,self,atmosphere, mach,phase, max_shaft_power)
        max_TPthrust, eta =Propeller().select('power_to_thrust',Prop_fid,self,atmosphere, mach,phase, max_TPshaft_power)        
        
        #Hybridization strategy: compute power contribution from turbine and electric motor
        thrust, shaft_power, motor_power, turbine_power, power_rate, TP_power_rate, EM_power_rate = self.Constant_elec_power_Constant_TTC(atmosphere, mach, phase,max_shaft_power,
                                                                                                                                         max_thermo_power,max_motor_power,max_available_power,
                                                                                                                                         gearbox_limit_power,max_TPshaft_power,thrust_rate,thrust,max_thrust,max_TPthrust)

        # Compute turbine SFC
        psfc_0 =self.turbine.sfc_at_max_power() #lb/hp/hr 
        psfc = psfc_0 * self.turbine.sfc_ratio(atmosphere, mach, TP_power_rate, turbine_power) #lb/hp/hr 
        ff=psfc*constants.lb/constants.hour* turbine_power/constants.hp #Kg/s
        
        
        # Compute Hydrogen consumption
        motor_power_in = self.motor.get_motor_power(motor_power,'downstream')
        power_electronics_power =motor_power_in
        power_electronics_power_in = self.power_electronics.get_idc_power(power_electronics_power, power_flow='downstream')
        fuel_cell_power=power_electronics_power_in
        FC_H2 = self.fuel_cell.get_fc_perfo(fuel_cell_power, power_flow='downstream') #kg/s        
        
        
        if phase==1:
            RPS=1200/60
        else:
            RPS=984/60
        flight_points.CT = thrust/(atmosphere.density*RPS**2*self.d_prop**4)  
        
        flight_points.psfc = psfc*constants.lb/constants.hour/constants.hp*self.k_psfc
        flight_points.thrust_rate = thrust_rate
        flight_points.thrust = thrust
        flight_points.TPshaft_power = turbine_power
        flight_points.TP_power_rate = TP_power_rate
        flight_points.thermo_power = max_thermo_power 
        flight_points.TP_residual_thrust = FR
        
        flight_points.shaft_power = shaft_power
        flight_points.power_rate =power_rate 
        flight_points.EMshaft_power = motor_power
        flight_points.FC_power =fuel_cell_power
        #flight_points.BAT_power = 
        
        flight_points.EM_power_rate = EM_power_rate 
        flight_points.H2_fc = FC_H2
        #flight_points.BAT_SOC = 
        
        #return tsfc, out_thrust_rate, out_thrust,out_power,out_power_rate,max_thermo_power


    def Constant_elec_power_Constant_TTC(
        self,
        atmosphere: Atmosphere,
        mach: Union[float, Sequence[float]],
        phase: Union[FlightPhase, Sequence],  
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
            )-> np.ndarray:
        """
        Hybridization strategy with constant electrical power during the hybrid segments (climb and cruise). 
        The gasturbine throttle is derived so to have the total shaft power (elec+turbine) exactly equal to the available turbine power 
        of the turbine at each flight point: this results in the same power profile along the mission, 
        therefore the same TTC as the conventional aircraft.
        """
        
        
        if phase==1 or phase==2 or phase==9: #takeoff or climb or MCT       
            shaft_power=max_TPshaft_power
            thrust=thrust_rate*max_TPthrust
            motor_power= max_motor_power
            turbine_power= shaft_power-motor_power 
            power_rate =  shaft_power/max_available_power 
            if max_available_power>gearbox_limit_power:
                power_rate = shaft_power/gearbox_limit_power #so power_rate will be 1 when we are at the gb_limit_power
        
        
        elif phase==4 : #descent
            thrust=thrust_rate*max_TPthrust
            #Θshaft_power = self.turbine.thrust_to_power(atmosphere, mach,phase,thrust)
            shaft_power, eta = Propeller().select('thrust_to_power',Prop_fid,self,atmosphere, mach,phase,thrust) 
            power_rate=shaft_power/max_available_power
            turbine_power= shaft_power
            motor_power = 0.
            
        elif phase==3: #cruise
            power_rate=thrust_rate
            # shaft_power = self.turbine.thrust_to_power(atmosphere, mach,phase,thrust)
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
        phase: Union[FlightPhase, Sequence],  
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
            )-> np.ndarray:
        """
        Hybridization strategy with constant electrical power (throttle=1) during the hybrid segments (climb and cruise). 
        The gasturbine throttle is also 1. TTC is then function of the max available power from gasturbine and electric motor.
        """
        if phase==1 or phase==2 or phase==9:
            shaft_power=max_shaft_power
            thrust=thrust_rate*max_thrust
            motor_power= max_motor_power
            turbine_power= max_shaft_power-motor_power 
            power_rate =  shaft_power/max_available_power 
            if max_available_power>gearbox_limit_power:
                power_rate = shaft_power/gearbox_limit_power #so power_rate will be 1 when we are at the gb_limit_power
                
        elif phase==4 : #descent
            thrust=thrust_rate*max_thrust
            shaft_power = self.turbine.thrust_to_power(atmosphere, mach,phase,thrust)
            power_rate=shaft_power/max_available_power
            turbine_power= shaft_power
            motor_power = 0.
            
        elif phase==3: #cruise
            power_rate=thrust_rate
            shaft_power=self.turbine.thrust_to_power(atmosphere, mach,phase,thrust)
            motor_power= max_motor_power
            turbine_power= shaft_power-motor_power  
        
        thrust_rate = np.asarray(thrust_rate)
        thrust = np.asarray(thrust)
        
        shaft_power = np.asarray(shaft_power) 
        power_rate = np.asarray(power_rate)
        
        TP_power_rate = np.asarray(turbine_power/max_thermo_power)
        EM_power_rate = np.asarray(motor_power/max_motor_power)
        
             
        return thrust, shaft_power, motor_power, turbine_power, power_rate, TP_power_rate, EM_power_rate


        
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

        altitude = atmosphere.get_altitude(altitude_in_feet=False)
        mach = np.asarray(mach)

  

        max_TPshaft_power,max_thermo_power, gearbox_limit_power=self.turbine.max_power(Atmosphere(altitude, altitude_in_feet=False), mach, phase)
        
        max_motor_power = self.Elec_nom_power
        
        max_available_power = max_motor_power + max_thermo_power
            

        if max_available_power>gearbox_limit_power:
            max_shaft_power=gearbox_limit_power  #gearbox mechanical limit 
        else:
            max_shaft_power=max_available_power
            
        return max_shaft_power,max_thermo_power,max_motor_power,max_available_power,gearbox_limit_power,max_TPshaft_power
    


