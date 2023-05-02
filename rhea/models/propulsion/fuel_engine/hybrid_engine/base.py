"""Base classes for fuel-consuming propulsion models."""
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

from abc import ABC
from typing import Union

import pandas as pd

from fastoad.base.flight_point import FlightPoint
from fastoad.models.propulsion import IPropulsion
from rhea.models.propulsion.fuel_engine.turboprop_engine.base import AbstractFuelPropulsion

class AbstractHybridPropulsion(IPropulsion, ABC):
    """
    Propulsion model that consume any fuel should inherit from this one.

    In inheritors, :meth:`compute_flight_points` is expected to define
    "sfc" and "thrust" in computed FlightPoint instances.
    """

    def get_consumed_mass(self, flight_point: FlightPoint, time_step: float) -> float:
        #return time_step * flight_point.sfc * flight_point.thrust
        return time_step * flight_point.psfc * flight_point.TPshaft_power + time_step * flight_point.H2_fc





class HybridEngineSet(AbstractHybridPropulsion):
    
    #def __init__(self, engine: IPropulsion, engine_count, motor_count):
    def __init__(self, engine: IPropulsion, engine_count):
       """
       Class for modelling an assembly of identical fuel engines.

       Thrust is supposed equally distributed among them.

       :param engine: the engine model
       :param engine_count:
       """
       self.engine = engine
       self.engine_count = engine_count #number of gasturbines
       self.motor_count = engine.motor_count #number of electrical motors
       
    def compute_flight_points(self, flight_points: Union[FlightPoint, pd.DataFrame]):

        if isinstance(flight_points, FlightPoint):
            flight_points_per_engine = FlightPoint(flight_points)
        else:
            flight_points_per_engine = flight_points.copy()

        if flight_points.thrust is not None:
            flight_points_per_engine.thrust = flight_points.thrust / self.engine_count

        self.engine.compute_flight_points(flight_points_per_engine)
        
        # flight_points.psfc = flight_points_per_engine.psfc
        flight_points.psfc = float(flight_points_per_engine.psfc or 0)
        flight_points.thrust = flight_points_per_engine.thrust * self.engine_count
        flight_points.thrust_rate = flight_points_per_engine.thrust_rate
        
        # flight_points.TPshaft_power = flight_points_per_engine.TPshaft_power* self.engine_count
        flight_points.TPshaft_power = float(flight_points_per_engine.TPshaft_power or 0)* self.engine_count
        flight_points.TP_power_rate = flight_points_per_engine.TP_power_rate 
        
        flight_points.thermo_power = float(flight_points_per_engine.thermo_power or 0) * self.engine_count
        
        flight_points.CT = flight_points_per_engine.CT
        
        flight_points.TP_thermal_efficiency = flight_points_per_engine.TP_thermal_efficiency
        if flight_points_per_engine.TP_residual_thrust :
            flight_points.TP_residual_thrust  =flight_points_per_engine.TP_residual_thrust  * self.engine_count
        flight_points.TP_air_flow=flight_points_per_engine.TP_air_flow #* self.engine_count
        flight_points.TP_total_pressure= flight_points_per_engine.TP_total_pressure 
        flight_points.TP_total_temperature  =  flight_points_per_engine.TP_total_temperature

        flight_points.shaft_power = flight_points_per_engine.shaft_power * self.engine_count
        flight_points.power_rate = flight_points_per_engine.power_rate         
        flight_points.EMshaft_power = flight_points_per_engine.EMshaft_power * self.motor_count
        flight_points.FC_power = float(flight_points_per_engine.FC_power or 0) * self.motor_count
        
        flight_points.FC_stack_power = float(flight_points_per_engine.FC_stack_power or 0) * self.motor_count
        
        flight_points.EM_power_rate = flight_points_per_engine.EM_power_rate 
        flight_points.H2_fc = float(flight_points_per_engine.H2_fc or 0) * self.motor_count
        
        flight_points.BAT_ec = float(flight_points_per_engine.BAT_ec or 0)* self.motor_count
        flight_points.BAT_power = float(flight_points_per_engine.BAT_power or 0) * self.motor_count
       
        flight_points.FC_efficiency = flight_points_per_engine.FC_efficiency
        flight_points.FC_airflow = flight_points_per_engine.FC_airflow
        flight_points.V_cell = flight_points_per_engine.V_cell
        flight_points.V_stack = flight_points_per_engine.V_stack
        flight_points.V_pack = flight_points_per_engine.V_pack
        flight_points.V_core = flight_points_per_engine.V_core
        flight_points.I_cell = flight_points_per_engine.I_cell
        flight_points.I_stack = flight_points_per_engine.I_stack
        flight_points.I_pack = flight_points_per_engine.I_pack
        flight_points.I_core = flight_points_per_engine.I_core
        
    
