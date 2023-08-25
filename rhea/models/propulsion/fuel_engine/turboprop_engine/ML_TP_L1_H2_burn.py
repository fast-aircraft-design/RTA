# -*- coding: utf-8 -*-
"""
Created on Tue May 10 09:55:39 2022

@author: LA202059
"""

import logging

#from models.propulsion import IPropulsion

import pandas as pd
from fastoad.model_base.flight_point import FlightPoint
#from models.propulsion.fuel_engine.turboprop_engine.base import AbstractFuelPropulsion
from typing import Union, Sequence, Tuple, Optional
from .ML_TP_L1 import ML_TP_L1

# Logger for this module
_LOGGER = logging.getLogger(__name__)
#from fastoad.base.dict import AddKeyAttributes

#AddKeyAttributes(["psfc","shaft_power", "power_rate","thermo_power","TP_thermal_efficiency","TP_residual_thrust","TP_air_flow","TP_total_pressure","TP_total_temperature","fuel_mass"
                 # ,"H2_mass","TPshaft_power","EMshaft_power","FC_power","TP_power_rate","EM_power_rate","H2_fc","CT"])(FlightPoint)


class ML_TP_L1_H2_burn(ML_TP_L1):
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
        H2_blend:float,

    ):
        """
        Parametric turboprop engine.

        It computes engine characteristics using analytical model from following
        sources:

         [1] F. Stagliano and H. Lobentanzer, "Impact of Novel Propulsion System Architectures Incorporating Diesel Engines on Mission Fuel Burn for a Tilt-Wing Transport Aircraft," in 12th AIAA Aviation Technology, Integration, and Operations (ATIO) Conference, Indianapolis, Indiana, 2012. AIAA 2012-5587.
         [2] Stückl, S. (2016). Methods for the Design and Evaluation of Future Aircraft Concepts Utilizing Electric Propulsion Systems (Doctoral dissertation, Technische Universität München).


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

        super().__init__(RTO_power,Design_Thermo_Power, Power_Offtake, gearbox_eta,d_prop,k_gb_RTO,
                            k_gb_NTO,
                            k_gb_MCL,
                            k_gb_MCT,
                            k_gb_MCR,
                            k_gb_FID,         
                            k_psfc,
                            k_prop,)
        self.H2_blend = H2_blend


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
        
        
        super().compute_flight_points(flight_points) 

        #H2_blend =0.1#inteso come percentage of power to be given by h2 combustion
        if 'diversion' in str(flight_points.name) or 'holding' in str(flight_points.name) :#or 'descent' in str(flight_points.name)  :
            H2_blend=0
        else: 
            H2_blend = self.H2_blend
        flight_points.H2_fc = (flight_points.psfc*11889/33330 ) * flight_points.TPshaft_power* H2_blend 
        flight_points.psfc=flight_points.psfc * (1-H2_blend)  #0.        
        #print(H2_blend, flight_points.H2_fc[-1])