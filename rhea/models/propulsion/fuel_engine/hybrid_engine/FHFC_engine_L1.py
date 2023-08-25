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
from fastoad.model_base.propulsion import IPropulsion
#from models.propulsion import IPropulsion
from fastoad_cs25.models.propulsion.fuel_propulsion.rubber_engine.exceptions import (
    FastRubberEngineInconsistentInputParametersError,
)
from fastoad.model_base.atmosphere import Atmosphere
import pandas as pd
from fastoad.model_base.flight_point import FlightPoint
#from models.propulsion.fuel_engine.hybrid_engine import AbstractHybridPropulsion
from .base import AbstractHybridPropulsion
# Logger for this module
_LOGGER = logging.getLogger(__name__)

#from models.propulsion.fuel_engine.hybrid_engine.engine_components.Battery_L0 import Battery
from .engine_components.ElectricMotor import ElectricMotor
from .engine_components.Fuel_cell_L1 import Fuel_cell
from .engine_components.IDC import IDC
from ..turboprop_engine.engine_components.Propeller import Propeller
#from fastoad.model_base.dict import AddKeyAttributes

# # Simply add the parameters:
#AddKeyAttributes(["psfc","shaft_power", "power_rate","thermo_power","TP_thermal_efficiency","TP_residual_thrust","TP_air_flow","TP_total_pressure","TP_total_temperature","fuel_mass"
            # ,"H2_mass","TPshaft_power","EMshaft_power","FC_power","TP_power_rate","EM_power_rate","H2_fc","FC_efficiency","FC_airflow","V_cell","V_stack","V_pack","V_core","I_cell","I_stack","I_pack","I_core","CT","FC_stack_power"])(FlightPoint)


Prop_fid='ADT'


class FHFCEngine_L1(AbstractHybridPropulsion):
    def __init__(
        self,
        FC_Power_Offtake: float,
        Gross_net_power_ratio:float,
        gearbox_eta: float,
        d_prop:float,

        k_prop: float,
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


        self.gearbox_eta = gearbox_eta
        self.d_prop=d_prop

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

        
        thrust_is_regulated, thrust_rate, thrust = self._check_thrust_inputs(
            thrust_is_regulated, thrust_rate, thrust
        )
        thrust_is_regulated = np.asarray(np.round(thrust_is_regulated, 0), dtype=bool)
        
        #Compute max available power and thrust

        max_shaft_power = self.Elec_nom_power

            
        max_thrust, eta = Propeller().select('power_to_thrust',Prop_fid,self,atmosphere, mach,phase, max_shaft_power)
        
        #Hybridization strategy: compute power contribution from turbine and electric motor
        thrust, shaft_power, motor_power, power_rate, EM_power_rate, thrust_rate = self.Segment_power_rating(atmosphere, mach, flight_points,max_shaft_power
                                                                                                                                          ,thrust_rate,thrust,max_thrust,thrust_is_regulated)
                     
        # Compute Hydrogen consumption
        motor_power_in = self.motor.get_motor_power(motor_power,'downstream')
        
        power_electronics_power = motor_power_in 
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

        flight_points.thrust_rate = thrust_rate
        flight_points.thrust = thrust
        # flight_points.TPshaft_power = 0.
        # flight_points.psfc = 0.





        flight_points.shaft_power = shaft_power
        flight_points.power_rate = power_rate
        flight_points.EMshaft_power = motor_power
        flight_points.FC_power = fuel_cell_power
        flight_points.FC_stack_power = fc_stack_power
        
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
        
        



 
    def Segment_power_rating(
        self,
        atmosphere: Atmosphere,
        mach: Union[float, Sequence[float]],
        flight_phase: Union[FlightPhase, Sequence],  
        max_shaft_power: Union[float, Sequence[float]],
        thrust_rate: Union[float, Sequence[float]],
        thrust: Union[float, Sequence[float]],
        max_thrust: Union[float, Sequence[float]], 
        thrust_is_regulated:  Union[float, Sequence[float]]
            )-> np.ndarray:
        """
        Power magement strategy: throttle=1 during climb. Determination of required power in cruise. 
        """
        phase = flight_phase.engine_setting
            
        if phase==1 or phase==2 or phase==9:
            shaft_power=max_shaft_power
            thrust=thrust_rate*max_thrust
            motor_power= shaft_power
            power_rate =  shaft_power/ max_shaft_power

                                                   
        elif phase==4 : #descent
            thrust=thrust_rate*max_thrust
            shaft_power, eta = Propeller().select('thrust_to_power',Prop_fid,self,atmosphere, mach,phase,thrust)
            power_rate=shaft_power/max_shaft_power
            motor_power = shaft_power
            
        elif phase==3: 


            power_rate=thrust_rate
            shaft_power, eta = Propeller().select('thrust_to_power',Prop_fid,self,atmosphere, mach,phase,thrust)
            motor_power= shaft_power

                
        thrust_rate = np.asarray(thrust_rate)
        thrust = np.asarray(thrust)   
        idx =  np.logical_not(thrust_is_regulated)
        if np.size(max_thrust) == 1:  #se scalare
            maximum_thrust = max_thrust
            out_thrust_rate = thrust_rate
            out_thrust = thrust
        
        out_thrust[idx] = out_thrust_rate[idx] * maximum_thrust

        out_thrust_rate = out_thrust / max_thrust
        thrust=out_thrust
        thrust_rate=out_thrust_rate   
        
        
        
        
        thrust_rate = np.asarray(thrust_rate)
        thrust = np.asarray(thrust)
        
        shaft_power = np.asarray(shaft_power) 
        power_rate = np.asarray(power_rate)
        
        EM_power_rate = np.asarray(motor_power/max_shaft_power)
        
             
        return thrust, shaft_power, motor_power, power_rate, EM_power_rate, thrust_rate


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

        


