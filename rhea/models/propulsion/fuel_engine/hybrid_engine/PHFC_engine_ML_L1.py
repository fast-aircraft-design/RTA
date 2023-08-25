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

# from fastoad.models.propulsion import IPropulsion
# from models.propulsion import IPropulsion
# from fastoad.models.propulsion.fuel_propulsion.rubber_engine.exceptions import (
#     FastRubberEngineInconsistentInputParametersError,
# )
from fastoad.model_base.atmosphere import Atmosphere
import pandas as pd
from fastoad.model_base.flight_point import FlightPoint

# from models.propulsion.fuel_engine.hybrid_engine import AbstractHybridPropulsion
from .base import AbstractHybridPropulsion

# Logger for this module
_LOGGER = logging.getLogger(__name__)

# from models.propulsion.fuel_engine.hybrid_engine.engine_components.Battery_L0 import Battery
from .engine_components.ElectricMotor import ElectricMotor
from .engine_components.Fuel_cell_L1 import Fuel_cell
from .engine_components.IDC import IDC
from ..turboprop_engine.ML_TP_L1 import ML_TP_L1
from ..turboprop_engine.engine_components.Propeller import Propeller

# from fastoad.base.dict import AddKeyAttributes

# AddKeyAttributes(["psfc","shaft_power", "power_rate","thermo_power","TP_thermal_efficiency","TP_residual_thrust","TP_air_flow","TP_total_pressure","TP_total_temperature","fuel_mass"
#                ,"H2_mass","TPshaft_power","EMshaft_power","FC_power","TP_power_rate","EM_power_rate","H2_fc","FC_efficiency","FC_airflow","V_cell","V_stack","V_pack","V_core","I_cell","I_stack","I_pack","I_core"])(FlightPoint)

Prop_fid = "ADT"


class PHFCEngine_ML_L1(AbstractHybridPropulsion):
    def __init__(
        self,
        RTO_power: float,
        Design_Thermo_Power: float,
        Power_Offtake: float,
        gearbox_eta: float,
        d_prop: float,
        k_gb_RTO: float,
        k_gb_NTO: float,
        k_gb_MCL: float,
        k_gb_MCT: float,
        k_gb_MCR: float,
        k_gb_FID: float,
        k_psfc: float,
        k_prop: float,
        # k_fc : float,
        Elec_nom_power: float,
        power_electronics_eta: float,
        motor_eta: float,
        # battery_eta: float,
        # fuel_cell_eta: float
        V_vec: list,
        i_vec: list,
        Nstacks: float,
        Npacks: float,
        Acell: float,
        Ncells: float,
        Nmotors: float,
        FC_Power_Offtake: float,
        Gross_net_power_ratio: float,
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
        self.d_prop = d_prop

        self.k_gb_RTO = k_gb_RTO
        self.k_gb_NTO = k_gb_NTO
        self.k_gb_MCL = k_gb_MCL
        self.k_gb_MCT = k_gb_MCT
        self.k_gb_MCR = k_gb_MCR
        self.k_gb_FID = k_gb_FID

        self.k_psfc = k_psfc
        self.k_prop = k_prop

        self.Elec_nom_power = Elec_nom_power
        self.power_electronics_eta = power_electronics_eta
        self.motor_eta = motor_eta
        # self.battery_eta = battery_eta
        # self.fuel_cell_eta = fuel_cell_eta

        # self.battery= Battery(battery_eta)
        self.fuel_cell = Fuel_cell(
            V_vec, i_vec, Nstacks, Npacks, Acell, Ncells, Nmotors
        )
        self.motor = ElectricMotor(motor_eta)
        self.power_electronics = IDC(power_electronics_eta)
        self.turbine = ML_TP_L1(
            RTO_power,
            Design_Thermo_Power,
            Power_Offtake,
            gearbox_eta,
            d_prop,
            k_gb_RTO,
            k_gb_NTO,
            k_gb_MCL,
            k_gb_MCT,
            k_gb_MCR,
            k_gb_FID,
            k_psfc,
            k_prop,
        )

        self.motor_count = Nmotors
        self.Gross_net_power_ratio = Gross_net_power_ratio
        self.FC_Power_Offtake = FC_Power_Offtake

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

        # print(self.RTO_power,self.Elec_nom_power)
        # if flight_points.name=='climb':
        #     print(flight_points.name, flight_points.EM_power_rate,flight_points.TP_power_rate)

        EM_power_rate = np.asarray(flight_points.EM_power_rate)
        TP_power_rate = np.asarray(flight_points.TP_power_rate)
        mach = np.asarray(flight_points.mach)
        altitude = np.asarray(flight_points.altitude)
        thrust_rate = np.asarray(flight_points.thrust_rate)
        thrust = np.asarray(flight_points.thrust)
        phase = flight_points.engine_setting
        thrust_is_regulated = flight_points.thrust_is_regulated

        atmosphere = Atmosphere(altitude, altitude_in_feet=False)
        # if flight_points.name=='cruise':
        #     print(altitude,mach)

        if thrust_is_regulated is not None:
            thrust_is_regulated = np.asarray(
                np.round(thrust_is_regulated, 0), dtype=bool
            )

        thrust_is_regulated, thrust_rate, thrust = self.turbine._check_thrust_inputs(
            thrust_is_regulated, thrust_rate, thrust
        )
        thrust_is_regulated = np.asarray(np.round(thrust_is_regulated, 0), dtype=bool)

        # Hybridization strategy: compute power contribution from turbine and electric motor
        (
            thrust,
            shaft_power,
            motor_power,
            turbine_power,
            power_rate,
            TP_power_rate,
            EM_power_rate,
            thrust_rate,
            residual_thrust,
            max_thermo_power,
        ) = self.Variable_Power_Rates(
            atmosphere,
            mach,
            flight_points,
            thrust_rate,
            thrust,
            thrust_is_regulated,
            EM_power_rate,
            TP_power_rate,
        )

        # Compute turbine SFC
        # here you are obliged to give cruise rating even if the actual rating is NTO or RTO,
        # because the psfc model only accepts power_rate with cruise rating
        psfc = self.turbine.psfc(atmosphere, mach, TP_power_rate, phase=3)  # kg/hp/hr

        ff = psfc / constants.hour * turbine_power / constants.hp  # Kg/s

        # Compute Hydrogen consumption
        motor_power_in = self.motor.get_motor_power(motor_power, "downstream")
        power_electronics_power = motor_power_in
        power_electronics_power_in = self.power_electronics.get_idc_power(
            power_electronics_power, power_flow="downstream"
        )
        fuel_cell_power = (
            power_electronics_power_in / self.Gross_net_power_ratio
            + self.FC_Power_Offtake
        )
        fc_perfo = self.fuel_cell.get_fc_perfo(fuel_cell_power)  # kg/s

        # print(altitude,mach,shaft_power,motor_power,turbine_power)

        flight_points.TP_residual_thrust = residual_thrust

        flight_points.psfc = psfc / constants.hour / constants.hp * self.k_psfc

        flight_points.thrust_rate = thrust_rate
        flight_points.thrust = thrust + residual_thrust
        flight_points.TPshaft_power = turbine_power
        flight_points.TP_power_rate = TP_power_rate
        flight_points.thermo_power = max_thermo_power

        flight_points.shaft_power = shaft_power
        flight_points.power_rate = power_rate
        flight_points.EMshaft_power = motor_power
        flight_points.FC_power = fuel_cell_power
        # flight_points.BAT_power =

        flight_points.EM_power_rate = EM_power_rate
        flight_points.H2_fc = fc_perfo[0]
        flight_points.FC_efficiency = fc_perfo[1]
        flight_points.FC_airflow = fc_perfo[2]
        flight_points.V_cell = fc_perfo[3]
        flight_points.V_stack = fc_perfo[4]
        flight_points.V_pack = fc_perfo[5]
        flight_points.V_core = fc_perfo[6]
        flight_points.I_cell = fc_perfo[7]
        flight_points.I_stack = fc_perfo[8]
        flight_points.I_pack = fc_perfo[9]
        flight_points.I_core = fc_perfo[10]

    def Variable_Power_Rates(
        self,
        atmosphere: Atmosphere,
        mach: Union[float, Sequence[float]],
        flight_phase: Union[FlightPhase, Sequence],
        thrust_rate: Union[float, Sequence[float]],
        thrust: Union[float, Sequence[float]],
        thrust_is_regulated: Union[float, Sequence[float]],
        EM_power_rate: Union[float, Sequence[float]],
        TP_power_rate: Union[float, Sequence[float]],
    ) -> np.ndarray:

        """
        Hybridization strategy with given power rate for turbine and electric motor during the manual_thrust segments
        (TO, climb and descent ). During the regulated_thrust segments (cruise) the electric power rate is given
        and the turbine power rate is calculated to provide required total thrust. The turbine has only 2 ratings
        which is the NTO (1) for normal operation and the RTO/MCT (8,9) for OEI condition.
        Residual thrust is only calculated for manual_thrust segments (TO and climb). In cruise all required thrust is provided by the propeller
        and in descent it is neglected.

        descent is hybrid
        """
        phase = flight_phase.engine_setting
        if phase != 9 and phase != 8:
            (
                max_TPshaft_power,
                max_thermo_power,
                gearbox_limit_power,
            ) = self.turbine.max_power(atmosphere, mach, phase=1)
        else:
            (
                max_TPshaft_power,
                max_thermo_power,
                gearbox_limit_power,
            ) = self.turbine.max_power(atmosphere, mach, phase=phase)
        max_TPthrust, eta = Propeller().select(
            "power_to_thrust", Prop_fid, self, atmosphere, mach, 3, max_TPshaft_power
        )  # phase here only matters for ML model to choose between NP100 or NP82
        max_motor_power = self.Elec_nom_power
        max_available_power = max_motor_power + max_TPshaft_power
        limit_mechanical_power = 1.094 * (
            max_motor_power
            + self.turbine.max_power(
                Atmosphere(0, altitude_in_feet=False), 0.2, phase=8
            )[0]
        )  # 2200000#max_power_propeller

        # Manual thrust segments
        if (
            phase == 1
            or phase == 2
            or phase == 9
            or phase == 8
            or (
                phase == 3
                and not thrust_is_regulated
                and "descent" not in str(flight_phase.name)
            )
        ):  # last condition is for top of climb max speed evaluation

            turbine_power = TP_power_rate * max_TPshaft_power
            motor_power = max_motor_power * EM_power_rate
            shaft_power = motor_power + turbine_power
            thrust, eta = Propeller().select(
                "power_to_thrust", Prop_fid, self, atmosphere, mach, phase, shaft_power
            )
            residual_thrust = self.turbine.compute_engine_point(
                atmosphere, mach, phase, T_prop=thrust, shaft_power=turbine_power
            )

            power_rate = shaft_power / max_available_power
            if max_available_power > limit_mechanical_power:
                print("max_available_power > limit_mechanical_power")
                power_rate = (
                    shaft_power / gearbox_limit_power
                )  # so power_rate will be 1 when we are at the gb_limit_power

        # Regulated thrust segments
        elif phase == 3:
            if "descent" in str(flight_phase.name):

                ##### comment section if descent  with fixed thrust_rate
                if thrust_is_regulated:  # fixed slope descent
                    thrust_rate = thrust / max_TPthrust
                else:  # acceleration descent
                    thrust = thrust_rate * max_TPthrust
                    # thrust=100
                    # thrust_rate=thrust/max_TPthrust
                #####
                thrust = thrust_rate * max_TPthrust
                shaft_power, eta = Propeller().select(
                    "thrust_to_power", Prop_fid, self, atmosphere, mach, phase, thrust
                )
                power_rate = shaft_power / max_available_power

                #################################MODIFIED HERE FOR HYBRID DESCENT###################################################
                if (
                    "diversion" in str(flight_phase.name) or not thrust_is_regulated
                ):  # discersion or acceleration descent
                    motor_power = max_motor_power * EM_power_rate  # 0.
                    turbine_power = shaft_power - motor_power
                else:
                    turbine_power = TP_power_rate * max_TPshaft_power
                    motor_power = (
                        shaft_power - turbine_power
                    )  # assume that in idle no thrust is produced
                residual_thrust = 0.0
                ####################################################################################

            else:

                shaft_power, eta = Propeller().select(
                    "thrust_to_power", Prop_fid, self, atmosphere, mach, phase, thrust
                )
                motor_power = EM_power_rate * max_motor_power
                turbine_power = shaft_power - motor_power
                power_rate = shaft_power / max_available_power
                residual_thrust = 0.0

            thrust_rate = np.asarray(thrust_rate)
            thrust = np.asarray(thrust)
            idx = np.logical_not(thrust_is_regulated)
            if np.size(max_TPthrust) == 1:  # se scalare
                maximum_thrust = max_TPthrust
                out_thrust_rate = thrust_rate
                out_thrust = thrust

            out_thrust[idx] = out_thrust_rate[idx] * maximum_thrust

            out_thrust_rate = out_thrust / max_TPthrust
            thrust = out_thrust
            thrust_rate = out_thrust_rate

        thrust_rate = np.asarray(thrust_rate)
        thrust = np.asarray(thrust)

        shaft_power = np.asarray(shaft_power)
        power_rate = np.asarray(power_rate)

        # TP_power_rate = np.asarray(turbine_power/max_thermo_power)
        TP_power_rate = np.asarray(turbine_power / max_TPshaft_power)
        EM_power_rate = np.asarray(motor_power / max_motor_power)

        return (
            thrust,
            shaft_power,
            motor_power,
            turbine_power,
            power_rate,
            TP_power_rate,
            EM_power_rate,
            thrust_rate,
            residual_thrust,
            max_thermo_power,
        )

    '''def Variable_Power_Rates(
            
        self,
        atmosphere: Atmosphere,
        mach: Union[float, Sequence[float]],
        flight_phase: Union[FlightPhase, Sequence],  
        thrust_rate: Union[float, Sequence[float]],
        thrust: Union[float, Sequence[float]],
        thrust_is_regulated:  Union[float, Sequence[float]],
        EM_power_rate: Union[float, Sequence[float]],
        TP_power_rate: Union[float, Sequence[float]],
            )-> np.ndarray:

        """
        Hybridization strategy with given power rate for turbine and electric motor during the manual_thrust segments
        (TO, climb and descent ). During the regulated_thrust segments (cruise) the electric power rate is given 
        and the turbine power rate is calculated to provide required total thrust. The turbine has only 2 ratings 
        which is the NTO (1) for normal operation and the RTO/MCT (8,9) for OEI condition.
        Residual thrust is only calculated for manual_thrust segments (TO and climb). In cruise all required thrust is provided by the propeller 
        and in descent it is neglected.
        """
        phase = flight_phase.engine_setting
        if phase!=9 and phase!=8:            
            max_TPshaft_power, max_thermo_power, gearbox_limit_power = self.turbine.max_power(atmosphere, mach, phase=1)    
        else:
            max_TPshaft_power, max_thermo_power, gearbox_limit_power = self.turbine.max_power(atmosphere, mach, phase=phase)    
        max_TPthrust, eta =Propeller().select('power_to_thrust',Prop_fid,self,atmosphere, mach,3, max_TPshaft_power)#phase here only matters for ML model to choose between NP100 or NP82
        max_motor_power = self.Elec_nom_power        
        max_available_power = max_motor_power + max_TPshaft_power
        limit_mechanical_power = 1.094*(max_motor_power+self.turbine.max_power(Atmosphere(0, altitude_in_feet=False), 0.2, phase=8)[0])   #2200000#max_power_propeller


        #Manual thrust segments    
        if phase==1 or phase==2 or phase==9 or phase==8 or (phase==3 and not thrust_is_regulated and 'descent' not in str(flight_phase.name)): #last condition is for top of climb max speed evaluation
            turbine_power = TP_power_rate*max_TPshaft_power                    
            motor_power= max_motor_power*EM_power_rate
            shaft_power=motor_power+turbine_power
            thrust, eta = Propeller().select('power_to_thrust',Prop_fid,self,atmosphere, mach,phase,shaft_power)
            residual_thrust = self.turbine.compute_engine_point(atmosphere, mach, phase,T_prop=thrust,shaft_power=turbine_power)

            power_rate =  shaft_power/max_available_power 
            if max_available_power>limit_mechanical_power:
                print('max_available_power > limit_mechanical_power')
                power_rate = shaft_power/gearbox_limit_power #so power_rate will be 1 when we are at the gb_limit_power
    
        #Regulated thrust segments                           
        elif phase==3: 
            if 'descent' in str(flight_phase.name):
                ##### comment section if descent  with fixed thrust_rate
                if thrust_is_regulated:
                    thrust_rate=thrust/max_TPthrust 
                else:
                    thrust=thrust_rate*max_TPthrust
                    # thrust=100
                    # thrust_rate=thrust/max_TPthrust 
                #####    
                thrust=thrust_rate*max_TPthrust    
                shaft_power, eta = Propeller().select('thrust_to_power',Prop_fid,self,atmosphere, mach,phase,thrust)
                power_rate=shaft_power/max_available_power
                motor_power =max_motor_power*EM_power_rate# 0.
                turbine_power= shaft_power-motor_power       
                residual_thrust=0.
            
            else:

                shaft_power, eta = Propeller().select('thrust_to_power',Prop_fid,self,atmosphere, mach,phase,thrust)
                motor_power=  EM_power_rate*max_motor_power
                turbine_power= shaft_power-motor_power 
                power_rate = shaft_power/max_available_power
                residual_thrust=0.

                
            thrust_rate = np.asarray(thrust_rate)
            thrust = np.asarray(thrust)   
            idx =  np.logical_not(thrust_is_regulated)
            if np.size(max_TPthrust) == 1:  #se scalare
                maximum_thrust = max_TPthrust
                out_thrust_rate = thrust_rate
                out_thrust = thrust
            
            out_thrust[idx] = out_thrust_rate[idx] * maximum_thrust

            out_thrust_rate = out_thrust / max_TPthrust
            thrust=out_thrust
            thrust_rate=out_thrust_rate   
        
        
        
        
        thrust_rate = np.asarray(thrust_rate)
        thrust = np.asarray(thrust)
        
        shaft_power = np.asarray(shaft_power) 
        power_rate = np.asarray(power_rate)
        
        # TP_power_rate = np.asarray(turbine_power/max_thermo_power)
        TP_power_rate = np.asarray(turbine_power/max_TPshaft_power)
        EM_power_rate = np.asarray(motor_power/max_motor_power)
             
        return thrust, shaft_power, motor_power, turbine_power, power_rate, TP_power_rate, EM_power_rate, thrust_rate,residual_thrust,max_thermo_power'''

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
        min_RTO = 0  # 1201582.729865489
        if phase == 2:  #'MCL'
            Shp_gb_limit = self.k_gb_MCL * (
                max(self.RTO_power, min_RTO) + self.Elec_nom_power
            )  # 2001582.729865489#self.RTO_power #

        elif phase == 4:  #'FID'
            Shp_gb_limit = self.k_gb_FID * (
                max(self.RTO_power, min_RTO) + self.Elec_nom_power
            )  # 2001582.729865489#self.RTO_power
        elif phase == 1:  #'TO'
            Shp_gb_limit = self.k_gb_NTO * (
                max(self.RTO_power, min_RTO) + self.Elec_nom_power
            )  # 2001582.729865489#self.RTO_power

        elif phase == 9:
            Shp_gb_limit = self.k_gb_MCT * (
                max(self.RTO_power, min_RTO) + self.Elec_nom_power
            )  # 2001582.729865489# self.RTO_power

        elif phase == 3:  #'CRZ'
            Shp_gb_limit = self.k_gb_MCR * (
                max(self.RTO_power, min_RTO) + self.Elec_nom_power
            )  # 2001582.729865489#self.RTO_power
        elif phase == 8:  #'RTO'
            Shp_gb_limit = self.k_gb_RTO * (
                max(self.RTO_power, min_RTO) + self.Elec_nom_power
            )  # 2001582.729865489#self.RTO_power #Watt =2400. hp

        else:
            rating = 1
            Shp_gb_limit = (
                max(self.RTO_power, min_RTO) + self.Elec_nom_power
            )  # 2001582.729865489#self.RTO_power

        altitude = atmosphere.get_altitude(altitude_in_feet=False)
        mach = np.asarray(mach)

        (
            max_TPshaft_power,
            max_thermo_power,
            gearbox_limit_power,
        ) = self.turbine.max_power(
            Atmosphere(altitude, altitude_in_feet=False), mach, phase
        )

        max_motor_power = self.Elec_nom_power

        max_available_power = max_motor_power + max_thermo_power

        if max_available_power > Shp_gb_limit:
            # print(phase,Shp_gb_limit,self.RTO_power,self.Elec_nom_power)
            max_shaft_power = Shp_gb_limit  # gearbox mechanical limit
        else:
            max_shaft_power = max_available_power

        return (
            max_shaft_power,
            max_thermo_power,
            max_motor_power,
            max_available_power,
            gearbox_limit_power,
            max_TPshaft_power,
        )

    def Constant_elec_power_Variable_TTC_2(
        self,
        atmosphere: Atmosphere,
        mach: Union[float, Sequence[float]],
        flight_phase: Union[FlightPhase, Sequence],
        max_shaft_power: Union[float, Sequence[float]],
        max_thermo_power: Union[float, Sequence[float]],
        max_motor_power: Union[float, Sequence[float]],
        max_available_power: Union[float, Sequence[float]],
        gearbox_limit_power: Union[float, Sequence[float]],
        max_TPshaft_power: Union[float, Sequence[float]],
        thrust_rate: Union[float, Sequence[float]],
        thrust: Union[float, Sequence[float]],
        max_thrust: Union[float, Sequence[float]],
        max_TPthrust: Union[float, Sequence[float]],
        thrust_is_regulated: Union[float, Sequence[float]],
        EM_power_rate: Union[float, Sequence[float]],
        TP_power_rate: Union[float, Sequence[float]],
    ) -> np.ndarray:

        """
        Hybridization strategy with constant electrical power (throttle=1) during the hybrid segments (climb and cruise).
        The gasturbine throttle is also 1. TTC is then function of the max available power from gasturbine and electric motor.
        """
        phase = flight_phase.engine_setting
        hybrid_reserves = False

        if phase == 1 or phase == 2 or phase == 9:
            if not hybrid_reserves:
                if "diversion" in str(flight_phase.name):
                    power_rate = [1]
                    shaft_power = max_TPshaft_power
                    thrust = thrust_rate * max_TPthrust
                    turbine_power = shaft_power
                    motor_power = 0.0

                # elif 'holding' in str(flight_phase.name):
                #     power_rate=thrust_rate
                #     # shaft_power=self.turbine.thrust_to_power(atmosphere, mach,phase,thrust)
                #     shaft_power, eta = Propeller().select('thrust_to_power',Prop_fid,self,atmosphere, mach,phase,thrust)
                #     motor_power= 0.
                #     turbine_power= shaft_power-motor_power
                else:
                    turbine_power = TP_power_rate * max_thermo_power
                    motor_power = max_motor_power * EM_power_rate
                    shaft_power = motor_power + turbine_power
                    thrust, eta = Propeller().select(
                        "power_to_thrust",
                        Prop_fid,
                        self,
                        atmosphere,
                        mach,
                        phase,
                        shaft_power,
                    )

                    power_rate = shaft_power / max_available_power
                    if max_available_power > gearbox_limit_power:
                        power_rate = (
                            shaft_power / gearbox_limit_power
                        )  # so power_rate will be 1 when we are at the gb_limit_power

            else:
                shaft_power = max_shaft_power
                # print('thrust_rate ',thrust_rate)
                thrust = thrust_rate * max_thrust
                motor_power = max_motor_power * EM_power_rate
                if max_TPshaft_power >= (max_shaft_power - motor_power):
                    turbine_power = max_shaft_power - motor_power
                else:
                    turbine_power = max_TPshaft_power
                    shaft_power = motor_power + turbine_power
                power_rate = shaft_power / max_available_power
                if max_available_power > gearbox_limit_power:
                    power_rate = (
                        shaft_power / gearbox_limit_power
                    )  # so power_rate will be 1 when we are at the gb_limit_power

        ##########   comment section if descent with fixed slope (thrust_is_regulated)
        # elif phase==4 : #descent
        #     thrust=thrust_rate*max_TPthrust   ##comment if descent with fixed slope (thrust_is_regulated)
        #     shaft_power, eta = Propeller().select('thrust_to_power',Prop_fid,self,atmosphere, mach,phase,thrust)
        #     power_rate=shaft_power/max_available_power
        #     turbine_power= shaft_power
        #     motor_power = 0.
        ############

        elif phase == 3:
            if not hybrid_reserves:
                if "diversion" in str(flight_phase.name):
                    power_rate = thrust_rate
                    # shaft_power=self.turbine.thrust_to_power(atmosphere, mach,phase,thrust)
                    shaft_power, eta = Propeller().select(
                        "thrust_to_power",
                        Prop_fid,
                        self,
                        atmosphere,
                        mach,
                        phase,
                        thrust,
                    )
                    motor_power = 0.0
                    turbine_power = shaft_power - motor_power

                elif "descent" in str(flight_phase.name):
                    ##### comment section if descent  with fixed thrust_rate
                    if thrust_is_regulated:
                        thrust_rate = thrust / max_TPthrust
                    else:
                        thrust = thrust_rate * max_TPthrust
                    #####
                    thrust = thrust_rate * max_TPthrust
                    shaft_power, eta = Propeller().select(
                        "thrust_to_power",
                        Prop_fid,
                        self,
                        atmosphere,
                        mach,
                        phase,
                        thrust,
                    )
                    power_rate = shaft_power / max_available_power
                    motor_power = 0.0
                    turbine_power = shaft_power - motor_power

                elif "holding" in str(flight_phase.name):
                    power_rate = thrust_rate
                    # shaft_power=self.turbine.thrust_to_power(atmosphere, mach,phase,thrust)
                    shaft_power, eta = Propeller().select(
                        "thrust_to_power",
                        Prop_fid,
                        self,
                        atmosphere,
                        mach,
                        phase,
                        thrust,
                    )
                    motor_power = 0.0
                    turbine_power = shaft_power - motor_power
                else:

                    # shaft_power=self.turbine.thrust_to_power(atmosphere, mach,phase,thrust)
                    # print(mach,flight_phase.name,thrust)
                    shaft_power, eta = Propeller().select(
                        "thrust_to_power",
                        Prop_fid,
                        self,
                        atmosphere,
                        mach,
                        phase,
                        thrust,
                    )
                    motor_power = EM_power_rate * max_motor_power
                    turbine_power = shaft_power - motor_power
                    power_rate = shaft_power / max_available_power
                    # print(motor_power,turbine_power)
            else:
                power_rate = thrust_rate
                # shaft_power=self.turbine.thrust_to_power(atmosphere, mach,phase,thrust)
                shaft_power, eta = Propeller().select(
                    "thrust_to_power", Prop_fid, self, atmosphere, mach, phase, thrust
                )
                motor_power = EM_power_rate * max_motor_power
                turbine_power = shaft_power - motor_power
                if motor_power > shaft_power:
                    print(
                        "Error: nominal electric power too high for cruise",
                        flight_phase.name,
                    )
                    motor_power = shaft_power
                    turbine_power = 100000.0

            thrust_rate = np.asarray(thrust_rate)
            thrust = np.asarray(thrust)
            idx = np.logical_not(thrust_is_regulated)
            if np.size(max_TPthrust) == 1:  # se scalare
                maximum_thrust = max_TPthrust
                out_thrust_rate = thrust_rate
                out_thrust = thrust
                # maximum_thrust = max_TPthrust[idx]

            out_thrust[idx] = out_thrust_rate[idx] * maximum_thrust

            out_thrust_rate = out_thrust / max_TPthrust
            thrust = out_thrust
            thrust_rate = out_thrust_rate

        thrust_rate = np.asarray(thrust_rate)
        thrust = np.asarray(thrust)

        shaft_power = np.asarray(shaft_power)
        power_rate = np.asarray(power_rate)

        TP_power_rate = np.asarray(turbine_power / max_thermo_power)
        EM_power_rate = np.asarray(motor_power / max_motor_power)

        return (
            thrust,
            shaft_power,
            motor_power,
            turbine_power,
            power_rate,
            TP_power_rate,
            EM_power_rate,
            thrust_rate,
        )
