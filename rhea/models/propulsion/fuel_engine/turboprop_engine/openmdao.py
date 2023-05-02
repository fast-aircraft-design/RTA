"""OpenMDAO wrapping of RubberEngine."""
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
from fastoad.models.propulsion import IOMPropulsionWrapper, IPropulsion, BaseOMPropulsionComponent
from fastoad.module_management.service_registry import RegisterPropulsion
from fastoad.openmdao.validity_checker import ValidityDomainChecker
from openmdao.core.component import Component
from typing import Union, Sequence, Optional, Tuple
from fastoad.constants import EngineSetting
import pandas as pd
from fastoad.base.flight_point import FlightPoint


#from .rubber_TP_engine import RubberTPEngine
from .TP_engine_L0 import TPEngine_L0
from .TP_engine_L1 import TPEngine_L1

from .ML_TP_L1 import ML_TP_L1
from .ML_TP_L1_H2_burn import ML_TP_L1_H2_burn
# Note: For the decorator to work, this module must be started as an iPOPO bundle,
# which is automatically done because OMRubberEngineComponent is currently registered
# as an OpenMDAO component with OpenMDAOSystemRegistry.register_system() in fastoad.register

####################################################################################################
#TURBOPROP model  
####################################################################################################
@RegisterPropulsion("rhea.wrapper.propulsion.ML_TP_L1")
class OMMLTPL1Wrapper(IOMPropulsionWrapper):
    """
    Wrapper class of for metamodel of TP engine L1 model.

    It is made to allow a direct call to :class:`~.turboprop_engine.ML_TP_L1` in an OpenMDAO
    component.

    """

    def setup(self, component: Component):
        component.add_input("data:propulsion:RTO_power", np.nan, units="W")
        component.add_input("data:propulsion:Design_Thermo_Power", np.nan, units="W")
        component.add_input("data:propulsion:Power_Offtake", np.nan, units="W")
        component.add_input("data:propulsion:gearbox_eta", np.nan)
        component.add_input("data:geometry:propulsion:propeller:diameter", np.nan,units="m")
        #tuning factors
        component.add_input("tuning:propulsion:k_psfc", np.nan)
        component.add_input("tuning:propulsion:k_prop", np.nan)
        
        #rating settings
        component.add_input("settings:propulsion:ratings:RTO:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:NTO:k_gb", np.nan)        
        component.add_input("settings:propulsion:ratings:MCL:k_gb", np.nan)        
        component.add_input("settings:propulsion:ratings:MCT:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:MCR:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:FID:k_gb", np.nan)
        #component.add_output("data:propulsion:shaft_power", units="W")
        #component.add_output("data:propulsion:power_rate")
        


    @staticmethod
    def get_model(inputs) -> IPropulsion:
        """

        :param inputs: input parameters that define the engine
        :return: an :class:`ML_TP_L1` instance
        """
        engine_params = {
            "RTO_power": inputs["data:propulsion:RTO_power"],
            "Design_Thermo_Power": inputs["data:propulsion:Design_Thermo_Power"],
            "Power_Offtake": inputs["data:propulsion:Power_Offtake"],
            "gearbox_eta": inputs["data:propulsion:gearbox_eta"],
            "d_prop":  inputs["data:geometry:propulsion:propeller:diameter"],
            
            "k_gb_RTO": inputs["settings:propulsion:ratings:RTO:k_gb"],
            "k_gb_NTO": inputs["settings:propulsion:ratings:NTO:k_gb"],       
            "k_gb_MCL": inputs["settings:propulsion:ratings:MCL:k_gb"],       
            "k_gb_MCT": inputs["settings:propulsion:ratings:MCT:k_gb"],
            "k_gb_MCR": inputs["settings:propulsion:ratings:MCR:k_gb"],
            "k_gb_FID": inputs["settings:propulsion:ratings:FID:k_gb"],
            
            "k_psfc":  inputs["tuning:propulsion:k_psfc"],
            "k_prop":  inputs["tuning:propulsion:k_prop"],
            

        }

        return ML_TP_L1(**engine_params)


@ValidityDomainChecker(
    {
        "data:propulsion:altitude": (None, 20000.0),
        "data:propulsion:mach": (0.1, 0.85),  # limitation of SFC ratio model
        "data:propulsion:rubber_engine:overall_pressure_ratio": (20.0, 40.0),
        "data:propulsion:rubber_engine:bypass_ratio": (3.0, 6.0),
        "data:propulsion:thrust_rate": (0.01, 1.0),  # limitation of SFC ratio model
        "data:propulsion:rubber_engine:turbine_inlet_temperature": (
            800.0,
            1600.0,
        ),  # limitation of max thrust model
        "data:propulsion:rubber_engine:delta_t4_climb": (
            -100.0,
            0.0,
        ),  # limitation of max thrust model
        "data:propulsion:rubber_engine:delta_t4_cruise": (
            -100.0,
            0.0,
        ),  # limitation of max thrust model
    }
)
class OMMLTPL1Component(BaseOMPropulsionComponent):
    """
    Parametric engine model as OpenMDAO component

    See :class:`ML_TP_L1` for more information.
    """

    def setup(self):
        shape = self.options["flight_point_count"]
        self.add_input("data:propulsion:mach", np.nan, shape=shape)
        self.add_input("data:propulsion:altitude", np.nan, shape=shape, units="m")
        self.add_input("data:propulsion:engine_setting", np.nan, shape=shape)
        self.add_input("data:propulsion:use_thrust_rate", np.nan, shape=shape)
        self.add_input("data:propulsion:required_thrust_rate", np.nan, shape=shape)
        self.add_input("data:propulsion:required_thrust", np.nan, shape=shape, units="N")

        self.add_output("data:propulsion:PSFC", shape=shape, units="kg/s/W", ref=1e-4)
        self.add_output("data:propulsion:thrust_rate", shape=shape, lower=0.0, upper=1.0)
        self.add_output("data:propulsion:thrust", shape=shape, units="N", ref=1e5)

        
        self.add_output("data:propulsion:TPshaft_power", shape=shape, units="W")
        self.add_output("data:propulsion:TP_power_rate", shape=shape, lower=0.0, upper=1.0)
        self.add_output("data:propulsion:max_thermodynamic_power", shape=shape, units="W")
        self.add_output("data:propulsion:TP_residual_thrust", shape=shape,  units="N")
        
        self.declare_partials("*", "*", method="fd")
        self.get_wrapper().setup(self)  
        
    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        wrapper = self.get_wrapper().get_model(inputs)
        flight_point = FlightPoint(
            mach=inputs["data:propulsion:mach"],
            altitude=inputs["data:propulsion:altitude"],
            engine_setting=inputs["data:propulsion:engine_setting"],
            thrust_is_regulated=np.logical_not(
                inputs["data:propulsion:use_thrust_rate"].astype(int)
            ),
            thrust_rate=inputs["data:propulsion:required_thrust_rate"],
            thrust=inputs["data:propulsion:required_thrust"],
        )
        wrapper.compute_flight_points(flight_point)
        outputs["data:propulsion:PSFC"] = flight_point.psfc
        outputs["data:propulsion:thrust_rate"] = flight_point.thrust_rate
        outputs["data:propulsion:thrust"] = flight_point.thrust        
        outputs["data:propulsion:TPshaft_power"]=flight_point.TPshaft_power
        outputs["data:propulsion:TP_power_rate"]=flight_point.TP_power_rate
        outputs["data:propulsion:max_thermodynamic_power"]= flight_point.thermo_power   
        outputs["data:propulsion:TP_residual_thrust"]=flight_point.TP_residual_thrust  
         
      
        
    @staticmethod
    def get_wrapper() -> OMMLTPL1Wrapper:
        return OMMLTPL1Wrapper()


@RegisterPropulsion("rhea.wrapper.propulsion.ML_TP_L1_H2_burn")
class OMMLTPL1H2Wrapper(IOMPropulsionWrapper):
    """
    Wrapper class of for ML TP L1 H2 engine model.



    """

    def setup(self, component: Component):
        component.add_input("data:propulsion:RTO_power", np.nan, units="W")
        component.add_input("data:propulsion:Design_Thermo_Power", np.nan, units="W")
        component.add_input("data:propulsion:Power_Offtake", np.nan, units="W")
        component.add_input("data:propulsion:gearbox_eta", np.nan)
        component.add_input("data:geometry:propulsion:propeller:diameter", np.nan,units="m")
        #tuning factors
        component.add_input("tuning:propulsion:k_psfc", np.nan)
        component.add_input("tuning:propulsion:k_prop", np.nan)
        component.add_input("data:propulsion:H2_blend", 1.)
        #rating settings
        component.add_input("settings:propulsion:ratings:RTO:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:NTO:k_gb", np.nan)        
        component.add_input("settings:propulsion:ratings:MCL:k_gb", np.nan)        
        component.add_input("settings:propulsion:ratings:MCT:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:MCR:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:FID:k_gb", np.nan)
        #component.add_output("data:propulsion:shaft_power", units="W")
        #component.add_output("data:propulsion:power_rate")
        


    @staticmethod
    def get_model(inputs) -> IPropulsion:
        """

        :param inputs: input parameters that define the engine
        :return: an :class:`PW100` instance
        """
        engine_params = {
            "RTO_power": inputs["data:propulsion:RTO_power"],
            "Design_Thermo_Power": inputs["data:propulsion:Design_Thermo_Power"],
            "Power_Offtake": inputs["data:propulsion:Power_Offtake"],
            "gearbox_eta": inputs["data:propulsion:gearbox_eta"],
            "d_prop":  inputs["data:geometry:propulsion:propeller:diameter"],
            
            "k_gb_RTO": inputs["settings:propulsion:ratings:RTO:k_gb"],
            "k_gb_NTO": inputs["settings:propulsion:ratings:NTO:k_gb"],       
            "k_gb_MCL": inputs["settings:propulsion:ratings:MCL:k_gb"],       
            "k_gb_MCT": inputs["settings:propulsion:ratings:MCT:k_gb"],
            "k_gb_MCR": inputs["settings:propulsion:ratings:MCR:k_gb"],
            "k_gb_FID": inputs["settings:propulsion:ratings:FID:k_gb"],
            
            "k_psfc":  inputs["tuning:propulsion:k_psfc"],
            "k_prop":  inputs["tuning:propulsion:k_prop"],
            "H2_blend":  inputs["data:propulsion:H2_blend"],

        }

        return ML_TP_L1_H2_burn(**engine_params)


@ValidityDomainChecker(
    {
        "data:propulsion:altitude": (None, 20000.0),
        "data:propulsion:mach": (0.1, 0.85),  # limitation of SFC ratio model
        "data:propulsion:rubber_engine:overall_pressure_ratio": (20.0, 40.0),
        "data:propulsion:rubber_engine:bypass_ratio": (3.0, 6.0),
        "data:propulsion:thrust_rate": (0.01, 1.0),  # limitation of SFC ratio model
        "data:propulsion:rubber_engine:turbine_inlet_temperature": (
            800.0,
            1600.0,
        ),  # limitation of max thrust model
        "data:propulsion:rubber_engine:delta_t4_climb": (
            -100.0,
            0.0,
        ),  # limitation of max thrust model
        "data:propulsion:rubber_engine:delta_t4_cruise": (
            -100.0,
            0.0,
        ),  # limitation of max thrust model
    }
)
class OMMLTPL1H2Component(BaseOMPropulsionComponent):
    """
    Parametric engine model as OpenMDAO component

    See :class:`PW100` for more information.
    """

    def setup(self):
        shape = self.options["flight_point_count"]
        #self.add_input("data:propulsion:mach", np.nan, shape=shape)
        
        self.add_input("data:propulsion:mach", np.nan, shape=shape)
        self.add_input("data:propulsion:altitude", np.nan, shape=shape, units="m")
        self.add_input("data:propulsion:engine_setting", np.nan, shape=shape)
        self.add_input("data:propulsion:use_thrust_rate", np.nan, shape=shape)
        self.add_input("data:propulsion:required_thrust_rate", np.nan, shape=shape)
        self.add_input("data:propulsion:required_thrust", np.nan, shape=shape, units="N")

        self.add_output("data:propulsion:PSFC", shape=shape, units="kg/s/W", ref=1e-4)
        self.add_output("data:propulsion:thrust_rate", shape=shape, lower=0.0, upper=1.0)
        self.add_output("data:propulsion:thrust", shape=shape, units="N", ref=1e5)

        
        self.add_output("data:propulsion:TPshaft_power", shape=shape, units="W")
        self.add_output("data:propulsion:TP_power_rate", shape=shape, lower=0.0, upper=1.0)
        self.add_output("data:propulsion:max_thermodynamic_power", shape=shape, units="W")
        self.add_output("data:propulsion:TP_residual_thrust", shape=shape,  units="N")
        
        self.declare_partials("*", "*", method="fd")
        self.get_wrapper().setup(self)  
        
    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        wrapper = self.get_wrapper().get_model(inputs)
        flight_point = FlightPoint(
            mach=inputs["data:propulsion:mach"],
            altitude=inputs["data:propulsion:altitude"],
            engine_setting=inputs["data:propulsion:engine_setting"],
            thrust_is_regulated=np.logical_not(
                inputs["data:propulsion:use_thrust_rate"].astype(int)
            ),
            thrust_rate=inputs["data:propulsion:required_thrust_rate"],
            thrust=inputs["data:propulsion:required_thrust"],
        )
        wrapper.compute_flight_points(flight_point)
        outputs["data:propulsion:PSFC"] = flight_point.psfc
        outputs["data:propulsion:thrust_rate"] = flight_point.thrust_rate
        outputs["data:propulsion:thrust"] = flight_point.thrust        
        outputs["data:propulsion:TPshaft_power"]=flight_point.TPshaft_power
        outputs["data:propulsion:TP_power_rate"]=flight_point.TP_power_rate
        outputs["data:propulsion:max_thermodynamic_power"]= flight_point.thermo_power   
        outputs["data:propulsion:TP_residual_thrust"]=flight_point.TP_residual_thrust  
         
      
        
    @staticmethod
    def get_wrapper() -> OMMLTPL1H2Wrapper:
        return OMMLTPL1H2Wrapper()











@RegisterPropulsion("rhea.wrapper.propulsion.TP_engine_L0")
class OMTPEngineL0Wrapper(IOMPropulsionWrapper):
    """
    Wrapper class of for L0 turnoprop engine model.

    It is made to allow a direct call to :class:`~.turboprop_engine.TPEngine_L0` in an OpenMDAO
    component.

    """

    def setup(self, component: Component):
        component.add_input("data:propulsion:RTO_power", np.nan, units="W")
        component.add_input("data:propulsion:Design_Thermo_Power", np.nan, units="W")
        component.add_input("data:propulsion:Power_Offtake", np.nan, units="W")
        component.add_input("data:propulsion:gearbox_eta", np.nan)
        component.add_input("data:geometry:propulsion:propeller:diameter", np.nan,units="m")
        #tuning factors
        component.add_input("tuning:propulsion:k_psfc", np.nan)
        component.add_input("tuning:propulsion:k_prop", np.nan)   

        #rating settings
        component.add_input("settings:propulsion:ratings:RTO:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:NTO:k_gb", np.nan)        
        component.add_input("settings:propulsion:ratings:MCL:k_gb", np.nan)        
        component.add_input("settings:propulsion:ratings:MCT:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:MCR:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:FID:k_gb", np.nan)        
        #component.add_output("data:propulsion:shaft_power", units="W")
        #component.add_output("data:propulsion:power_rate")
        


    @staticmethod
    def get_model(inputs) -> IPropulsion:
        """

        :param inputs: input parameters that define the engine
        :return: an :class:`TPEngine_L0` instance
        """
        engine_params = {
            "RTO_power": inputs["data:propulsion:RTO_power"],
            "Design_Thermo_Power": inputs["data:propulsion:Design_Thermo_Power"],
            "Power_Offtake": inputs["data:propulsion:Power_Offtake"],
            "gearbox_eta": inputs["data:propulsion:gearbox_eta"],
            "d_prop":  inputs["data:geometry:propulsion:propeller:diameter"],
            
            "k_gb_RTO": inputs["settings:propulsion:ratings:RTO:k_gb"],
            "k_gb_NTO": inputs["settings:propulsion:ratings:NTO:k_gb"],       
            "k_gb_MCL": inputs["settings:propulsion:ratings:MCL:k_gb"],       
            "k_gb_MCT": inputs["settings:propulsion:ratings:MCT:k_gb"],
            "k_gb_MCR": inputs["settings:propulsion:ratings:MCR:k_gb"],
            "k_gb_FID": inputs["settings:propulsion:ratings:FID:k_gb"],
            
            "k_psfc":  inputs["tuning:propulsion:k_psfc"],
            "k_prop":  inputs["tuning:propulsion:k_prop"],
            
        }

        return TPEngine_L0(**engine_params)


@ValidityDomainChecker(
    {
        "data:propulsion:altitude": (None, 20000.0),
        "data:propulsion:mach": (0.1, 0.85),  # limitation of SFC ratio model
        "data:propulsion:rubber_engine:overall_pressure_ratio": (20.0, 40.0),
        "data:propulsion:rubber_engine:bypass_ratio": (3.0, 6.0),
        "data:propulsion:thrust_rate": (0.01, 1.0),  # limitation of SFC ratio model
        "data:propulsion:rubber_engine:turbine_inlet_temperature": (
            800.0,
            1600.0,
        ),  # limitation of max thrust model
        "data:propulsion:rubber_engine:delta_t4_climb": (
            -100.0,
            0.0,
        ),  # limitation of max thrust model
        "data:propulsion:rubber_engine:delta_t4_cruise": (
            -100.0,
            0.0,
        ),  # limitation of max thrust model
    }
)
class OMTPEngineL0Component(BaseOMPropulsionComponent):
    """
    Parametric engine model as OpenMDAO component

    See :class:`TPEngine_L0` for more information.
    """

    def setup(self):
        shape = self.options["flight_point_count"]
        self.add_input("data:propulsion:mach", np.nan, shape=shape)
        self.add_input("data:propulsion:altitude", np.nan, shape=shape, units="m")
        self.add_input("data:propulsion:engine_setting", np.nan, shape=shape)
        self.add_input("data:propulsion:use_thrust_rate", np.nan, shape=shape)
        self.add_input("data:propulsion:required_thrust_rate", np.nan, shape=shape)
        self.add_input("data:propulsion:required_thrust", np.nan, shape=shape, units="N")

        self.add_output("data:propulsion:PSFC", shape=shape, units="kg/s/W", ref=1e-4)
        self.add_output("data:propulsion:thrust_rate", shape=shape, lower=0.0, upper=1.0)
        self.add_output("data:propulsion:thrust", shape=shape, units="N", ref=1e5)

        
        self.add_output("data:propulsion:TPshaft_power", shape=shape, units="W")
        self.add_output("data:propulsion:TP_power_rate", shape=shape, lower=0.0, upper=1.0)
        self.add_output("data:propulsion:max_thermodynamic_power", shape=shape, units="W")
        
        self.declare_partials("*", "*", method="fd")
        self.get_wrapper().setup(self)  
        
    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        wrapper = self.get_wrapper().get_model(inputs)
        flight_point = FlightPoint(
            mach=inputs["data:propulsion:mach"],
            altitude=inputs["data:propulsion:altitude"],
            engine_setting=inputs["data:propulsion:engine_setting"],
            thrust_is_regulated=np.logical_not(
                inputs["data:propulsion:use_thrust_rate"].astype(int)
            ),
            thrust_rate=inputs["data:propulsion:required_thrust_rate"],
            thrust=inputs["data:propulsion:required_thrust"],
        )
        wrapper.compute_flight_points(flight_point)
        outputs["data:propulsion:PSFC"] = flight_point.psfc
        outputs["data:propulsion:thrust_rate"] = flight_point.thrust_rate
        outputs["data:propulsion:thrust"] = flight_point.thrust        
        outputs["data:propulsion:TPshaft_power"]=flight_point.TPshaft_power
        outputs["data:propulsion:TP_power_rate"]=flight_point.TP_power_rate
        outputs["data:propulsion:max_thermodynamic_power"]= flight_point.thermo_power            
      
        
    @staticmethod
    def get_wrapper() -> OMTPEngineL0Wrapper:
        return OMTPEngineL0Wrapper()

@RegisterPropulsion("rhea.wrapper.propulsion.TP_engine_L1")
class OMTPEngineL1Wrapper(IOMPropulsionWrapper):
    """
    Wrapper class of for L1 turnoprop engine model.

    It is made to allow a direct call to :class:`~.turboprop_engine.TPEngine_L1` in an OpenMDAO
    component.

    """

    def setup(self, component: Component):
        component.add_input("data:propulsion:RTO_power", np.nan, units="W")
        component.add_input("data:propulsion:Design_Thermo_Power", np.nan, units="W")
        component.add_input("data:propulsion:Power_Offtake", np.nan, units="W")
        component.add_input("data:propulsion:gearbox_eta", np.nan)   
        component.add_input("data:geometry:propulsion:propeller:diameter", np.nan,units="m")
        
        
        component.add_input("data:propulsion:L1_engine:fuel", np.nan) 
        #design values
        component.add_input("data:propulsion:L1_engine:turbine_inlet_temperature", np.nan, units="K")
        component.add_input("data:propulsion:L1_engine:HP_bleed", np.nan)
        component.add_input("data:propulsion:L1_engine:LP_bleed", np.nan)

        component.add_input("data:propulsion:L1_engine:inlet:inlet_eta_pol", np.nan)
        component.add_input("data:propulsion:L1_engine:inlet:inlet_pressure_ratio", np.nan)        
        component.add_input("data:propulsion:L1_engine:lpc:lpc_eta_pol", np.nan)
        component.add_input("data:propulsion:L1_engine:lpc:lpc_pressure_ratio", np.nan)          
        component.add_input("data:propulsion:L1_engine:hpc:hpc_eta_pol", np.nan)
        component.add_input("data:propulsion:L1_engine:hpc:hpc_pressure_ratio", np.nan) 
        component.add_input("data:propulsion:L1_engine:combustor:combustor_eta", np.nan)
        component.add_input("data:propulsion:L1_engine:combustor:combustor_pressure_ratio", np.nan)     
        component.add_input("data:propulsion:L1_engine:hpt:hpt_eta_pol", np.nan)
        component.add_input("data:propulsion:L1_engine:hpt:hpt_eta_mech", np.nan)   
        component.add_input("data:propulsion:L1_engine:lpt:lpt_eta_pol", np.nan)
        component.add_input("data:propulsion:L1_engine:lpt:lpt_eta_mech", np.nan)          
        component.add_input("data:propulsion:L1_engine:pt:pt_eta_pol", np.nan)
        component.add_input("data:propulsion:L1_engine:pt:pt_eta_mech", np.nan)           
        component.add_input("data:propulsion:L1_engine:nozzle:nozzle_eta_pol", np.nan)
        component.add_input("data:propulsion:L1_engine:nozzle:nozzle_pressure_ratio", np.nan)
        component.add_input("data:propulsion:L1_engine:nozzle:nozzle_area_ratio", np.nan)
           
        #sizing factors
        component.add_input("data:propulsion:L1_engine:sizing:k0", np.nan)
        component.add_input("data:propulsion:L1_engine:sizing:k1", np.nan)        
        component.add_input("data:propulsion:L1_engine:sizing:k2", np.nan)
        component.add_input("data:propulsion:L1_engine:sizing:tau_t_sizing", np.nan)
        component.add_input("data:propulsion:L1_engine:sizing:pi_t_sizing", np.nan)        
        component.add_input("data:propulsion:L1_engine:sizing:M_out_sizing", np.nan)

        #tuning factors
        component.add_input("tuning:propulsion:k_psfc", np.nan)
        component.add_input("tuning:propulsion:k_prop", np.nan)

        #rating settings
        component.add_input("settings:propulsion:ratings:RTO:k_th", np.nan)
        component.add_input("settings:propulsion:ratings:RTO:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:NTO:k_th", np.nan)
        component.add_input("settings:propulsion:ratings:NTO:k_gb", np.nan)        
        component.add_input("settings:propulsion:ratings:MCL:k_th", np.nan)
        component.add_input("settings:propulsion:ratings:MCL:k_gb", np.nan)        
        component.add_input("settings:propulsion:ratings:MCT:k_th", np.nan)
        component.add_input("settings:propulsion:ratings:MCT:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:MCR:k_th", np.nan)
        component.add_input("settings:propulsion:ratings:MCR:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:FID:k_th", np.nan)
        component.add_input("settings:propulsion:ratings:FID:k_gb", np.nan)
        
    @staticmethod
    def get_model(inputs) -> IPropulsion:
        """

        :param inputs: input parameters that define the engine
        :return: an :class:`TPEngine_L1` instance
        """
        engine_params = {
            "RTO_power": inputs["data:propulsion:RTO_power"],
            "Design_Thermo_Power": inputs["data:propulsion:Design_Thermo_Power"],
            "Power_Offtake": inputs["data:propulsion:Power_Offtake"],
            "gearbox_eta": inputs["data:propulsion:gearbox_eta"], 
            "d_prop":  inputs["data:geometry:propulsion:propeller:diameter"],
            
            "fuel": inputs["data:propulsion:L1_engine:fuel"],


            "turbine_inlet_temperature": inputs["data:propulsion:L1_engine:turbine_inlet_temperature"],            
            "HP_bleed": inputs["data:propulsion:L1_engine:HP_bleed"],
            "LP_bleed": inputs["data:propulsion:L1_engine:LP_bleed"],
            
            "inlet_eta_pol": inputs["data:propulsion:L1_engine:inlet:inlet_eta_pol"],
            "inlet_pressure_ratio": inputs["data:propulsion:L1_engine:inlet:inlet_pressure_ratio"],

             "lpc_eta_pol": inputs["data:propulsion:L1_engine:lpc:lpc_eta_pol"],
             "lpc_pressure_ratio": inputs["data:propulsion:L1_engine:lpc:lpc_pressure_ratio"],
           
            "hpc_eta_pol": inputs["data:propulsion:L1_engine:hpc:hpc_eta_pol"],
            "hpc_pressure_ratio": inputs["data:propulsion:L1_engine:hpc:hpc_pressure_ratio"],
           
            "combustor_eta": inputs["data:propulsion:L1_engine:combustor:combustor_eta"],
            "combustor_pressure_ratio": inputs["data:propulsion:L1_engine:combustor:combustor_pressure_ratio"],
            
            "hpt_eta_pol": inputs["data:propulsion:L1_engine:hpt:hpt_eta_pol"],
            "hpt_eta_mech": inputs["data:propulsion:L1_engine:hpt:hpt_eta_mech"],

            "lpt_eta_pol": inputs["data:propulsion:L1_engine:lpt:lpt_eta_pol"],
            "lpt_eta_mech": inputs["data:propulsion:L1_engine:lpt:lpt_eta_mech"],

            "pt_eta_pol": inputs["data:propulsion:L1_engine:pt:pt_eta_pol"],
            "pt_eta_mech": inputs["data:propulsion:L1_engine:pt:pt_eta_mech"],

            "nozzle_eta_pol": inputs["data:propulsion:L1_engine:nozzle:nozzle_eta_pol"],
            "nozzle_pressure_ratio": inputs["data:propulsion:L1_engine:nozzle:nozzle_pressure_ratio"],
            "nozzle_area_ratio": inputs["data:propulsion:L1_engine:nozzle:nozzle_area_ratio"],
            
            
            "k0": inputs["data:propulsion:L1_engine:sizing:k0"],
            "k1": inputs["data:propulsion:L1_engine:sizing:k1"],
            "k2": inputs["data:propulsion:L1_engine:sizing:k2"],
            "tau_t_sizing": inputs["data:propulsion:L1_engine:sizing:tau_t_sizing"],
            "pi_t_sizing": inputs["data:propulsion:L1_engine:sizing:pi_t_sizing"],
            "M_out_sizing": inputs["data:propulsion:L1_engine:sizing:M_out_sizing"],

            "k_th_RTO": inputs["settings:propulsion:ratings:RTO:k_th"],
            "k_gb_RTO": inputs["settings:propulsion:ratings:RTO:k_gb"],
            "k_th_NTO": inputs["settings:propulsion:ratings:NTO:k_th"],
            "k_gb_NTO": inputs["settings:propulsion:ratings:NTO:k_gb"],       
            "k_th_MCL": inputs["settings:propulsion:ratings:MCL:k_th"],
            "k_gb_MCL": inputs["settings:propulsion:ratings:MCL:k_gb"],       
            "k_th_MCT": inputs["settings:propulsion:ratings:MCT:k_th"],
            "k_gb_MCT": inputs["settings:propulsion:ratings:MCT:k_gb"],
            "k_th_MCR": inputs["settings:propulsion:ratings:MCR:k_th"],
            "k_gb_MCR": inputs["settings:propulsion:ratings:MCR:k_gb"],
            "k_th_FID": inputs["settings:propulsion:ratings:FID:k_th"],
            "k_gb_FID": inputs["settings:propulsion:ratings:FID:k_gb"],
            
            "k_psfc":  inputs["tuning:propulsion:k_psfc"],
            "k_prop":  inputs["tuning:propulsion:k_prop"],

            
        }

        return TPEngine_L1(**engine_params)


@ValidityDomainChecker(
    {
        "data:propulsion:altitude": (None, 20000.0),
        "data:propulsion:mach": (0.0001, 0.85),  # limitation of SFC ratio model 
        "data:propulsion:rubber_engine:overall_pressure_ratio": (20.0, 40.0),
        "data:propulsion:rubber_engine:bypass_ratio": (3.0, 6.0),
        "data:propulsion:thrust_rate": (0.01, 1.0),  # limitation of SFC ratio model
        "data:propulsion:rubber_engine:turbine_inlet_temperature": (
            800.0,
            1600.0,
        ),  # limitation of max thrust model
        "data:propulsion:rubber_engine:delta_t4_climb": (
            -100.0,
            0.0,
        ),  # limitation of max thrust model
        "data:propulsion:rubber_engine:delta_t4_cruise": (
            -100.0,
            0.0,
        ),  # limitation of max thrust model
    }
)
class OMTPEngineL1Component(BaseOMPropulsionComponent):
    """
    Parametric engine model as OpenMDAO component

    See :class:`TPEngine_L1` for more information.
    """

    def setup(self):
        shape = self.options["flight_point_count"]
        self.add_input("data:propulsion:mach", np.nan, shape=shape)
        self.add_input("data:propulsion:altitude", np.nan, shape=shape, units="m")
        self.add_input("data:propulsion:engine_setting", np.nan, shape=shape)
        self.add_input("data:propulsion:use_thrust_rate", np.nan, shape=shape)
        self.add_input("data:propulsion:required_thrust_rate", np.nan, shape=shape)
        self.add_input("data:propulsion:required_thrust", np.nan, shape=shape, units="N")

        self.add_output("data:propulsion:PSFC", shape=shape, units="kg/s/W", ref=1e-4)
        self.add_output("data:propulsion:thrust_rate", shape=shape, lower=0.0, upper=1.0)
        self.add_output("data:propulsion:thrust", shape=shape, units="N", ref=1e5)

        self.add_output("data:propulsion:TPshaft_power", shape=shape, units="W")
        self.add_output("data:propulsion:TP_power_rate", shape=shape, lower=0.0, upper=1.0)
        self.add_output("data:propulsion:max_thermodynamic_power", shape=shape, units="W")
        
        self.add_output("data:propulsion:TP_thermal_efficiency", shape=shape)
        self.add_output("data:propulsion:TP_residual_thrust", shape=shape,  units="N")
        self.add_output("data:propulsion:TP_air_flow", shape=shape, units="kg/s")
        self.add_output("data:propulsion:TP_total_pressure", shape=shape)
        self.add_output("data:propulsion:TP_total_temperature", shape=shape, units="K")
        
        self.declare_partials("*", "*", method="fd")
        self.get_wrapper().setup(self)  
        
    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        wrapper = self.get_wrapper().get_model(inputs)
        flight_point = FlightPoint(
            mach=inputs["data:propulsion:mach"],
            altitude=inputs["data:propulsion:altitude"],
            engine_setting=inputs["data:propulsion:engine_setting"],
            thrust_is_regulated=np.logical_not(
                inputs["data:propulsion:use_thrust_rate"].astype(int)
            ),
            thrust_rate=inputs["data:propulsion:required_thrust_rate"],
            thrust=inputs["data:propulsion:required_thrust"],
        )
        wrapper.compute_flight_points(flight_point)
        outputs["data:propulsion:PSFC"] = flight_point.psfc
        outputs["data:propulsion:thrust_rate"] = flight_point.thrust_rate
        outputs["data:propulsion:thrust"] = flight_point.thrust        
        outputs["data:propulsion:TPshaft_power"]=flight_point.TPshaft_power
        outputs["data:propulsion:TP_power_rate"]=flight_point.TP_power_rate
        outputs["data:propulsion:max_thermodynamic_power"]= flight_point.thermo_power   
        
        outputs["data:propulsion:TP_thermal_efficiency"]=flight_point.TP_thermal_efficiency
        outputs["data:propulsion:TP_residual_thrust"]=flight_point.TP_residual_thrust  
        outputs["data:propulsion:TP_air_flow"]=flight_point.TP_air_flow
        outputs["data:propulsion:TP_total_pressure"]=flight_point.TP_total_pressure
        outputs["data:propulsion:TP_total_temperature"]=flight_point.TP_total_temperature
        
    @staticmethod
    def get_wrapper() -> OMTPEngineL1Wrapper:
        return OMTPEngineL1Wrapper()
