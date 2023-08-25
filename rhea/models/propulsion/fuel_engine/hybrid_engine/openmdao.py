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
from fastoad.model_base.propulsion import (
    IOMPropulsionWrapper,
    IPropulsion,
    BaseOMPropulsionComponent,
)

from fastoad.module_management.service_registry import RegisterPropulsion
from fastoad.openmdao.validity_checker import ValidityDomainChecker
from openmdao.core.component import Component

from fastoad.model_base.flight_point import FlightPoint


# from .rubber_TP_engine import RubberTPEngine

from .PHFC_engine_L0 import PHFCEngine_L0
from .PHFC_engine_L1 import PHFCEngine_L1
from .FHFC_engine_L1 import FHFCEngine_L1
from .PHFC_engine_ML_L1 import PHFCEngine_ML_L1
from .PHEB_engine_ML_L1 import PHEBEngine_ML_L1

# Note: For the decorator to work, this module must be started as an iPOPO bundle,
# which is automatically done because OMRubberEngineComponent is currently registered
# as an OpenMDAO component with OpenMDAOSystemRegistry.register_system() in fastoad.register

####################################################################################################
# HYBRID  model
####################################################################################################
@RegisterPropulsion("rhea.wrapper.propulsion.PHEB_engine_ML_L1")
class OMPHEBEngineMLL1Wrapper(IOMPropulsionWrapper):
    """

    Wrapper class for ML_L1 parallel hybrid battery engine model.

    It is made to allow a direct call to :class:`~.hybrid_engine.HybridEngine_ML_L1` in an OpenMDAO
    component.

    """

    def setup(self, component: Component):
        component.add_input("data:propulsion:RTO_power", np.nan, units="W")
        component.add_input("data:propulsion:Design_Thermo_Power", np.nan, units="W")
        component.add_input("data:propulsion:Power_Offtake", np.nan, units="W")
        component.add_input("data:propulsion:gearbox_eta", np.nan)
        component.add_input(
            "data:geometry:propulsion:propeller:diameter", np.nan, units="m"
        )

        # tuning factors
        component.add_input("tuning:propulsion:k_psfc", np.nan)
        component.add_input("tuning:propulsion:k_prop", np.nan)

        # rating settings
        component.add_input("settings:propulsion:ratings:RTO:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:NTO:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:MCL:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:MCT:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:MCR:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:FID:k_gb", np.nan)

        # electric inputs
        component.add_input("data:propulsion:electric_systems:P_nom", np.nan, units="W")
        component.add_input(
            "data:propulsion:electric_systems:power_electronics:power_electronics_eta",
            np.nan,
        )
        component.add_input("data:propulsion:electric_systems:motor:motor_eta", np.nan)
        component.add_input(
            "data:propulsion:electric_systems:battery:battery_eta", np.nan
        )
        component.add_input(
            "data:propulsion:electric_systems:battery:specific_energy",
            np.nan,
            units="W*h/kg",
        )
        component.add_input(
            "data:propulsion:electric_systems:battery:specific_power",
            np.nan,
            units="W/kg",
        )
        component.add_input(
            "data:propulsion:electric_systems:battery:battery_SOCmin", np.nan
        )

        component.add_input("data:geometry:propulsion:motor:count", np.nan)
        # component.add_output("data:propulsion:shaft_power", units="W")
        # component.add_output("data:propulsion:power_rate")

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
            "d_prop": inputs["data:geometry:propulsion:propeller:diameter"],
            "k_gb_RTO": inputs["settings:propulsion:ratings:RTO:k_gb"],
            "k_gb_NTO": inputs["settings:propulsion:ratings:NTO:k_gb"],
            "k_gb_MCL": inputs["settings:propulsion:ratings:MCL:k_gb"],
            "k_gb_MCT": inputs["settings:propulsion:ratings:MCT:k_gb"],
            "k_gb_MCR": inputs["settings:propulsion:ratings:MCR:k_gb"],
            "k_gb_FID": inputs["settings:propulsion:ratings:FID:k_gb"],
            "k_psfc": inputs["tuning:propulsion:k_psfc"],
            "k_prop": inputs["tuning:propulsion:k_prop"],
            "Elec_nom_power": inputs["data:propulsion:electric_systems:P_nom"],
            "power_electronics_eta": inputs[
                "data:propulsion:electric_systems:power_electronics:power_electronics_eta"
            ],
            "motor_eta": inputs["data:propulsion:electric_systems:motor:motor_eta"],
            "battery_eta": inputs[
                "data:propulsion:electric_systems:battery:battery_eta"
            ],
            "battery_SOCmin": inputs[
                "data:propulsion:electric_systems:battery:battery_SOCmin"
            ],
            "battery_Esp": inputs[
                "data:propulsion:electric_systems:battery:specific_energy"
            ],
            "battery_Psp": inputs[
                "data:propulsion:electric_systems:battery:specific_power"
            ],
            "Nmotors": inputs["data:geometry:propulsion:motor:count"],
        }

        return PHEBEngine_ML_L1(**engine_params)


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
class OMPHEBEngineMLL1Component(BaseOMPropulsionComponent):
    """
    Parametric engine model as OpenMDAO component

    See :class:`PHEBEngine_ML_L1` for more information.
    """

    def setup(self):
        super().setup()
        shape = self.options["flight_point_count"]
        self.add_output("data:propulsion:shaft_power", shape=shape, units="W")
        self.add_output("data:propulsion:power_rate", shape=shape, lower=0.0, upper=1.0)
        self.add_output(
            "data:propulsion:max_thermodynamic_power", shape=shape, units="W"
        )

        self.add_output("data:propulsion:TPshaft_power", shape=shape, units="W")
        self.add_output("data:propulsion:EMshaft_power", shape=shape, units="W")
        self.add_output("data:propulsion:BAT_power", shape=shape, units="W")
        self.add_output("data:propulsion:BAT_ec", shape=shape, units="W")

        self.add_output(
            "data:propulsion:TP_power_rate", shape=shape, lower=0.0, upper=1.0
        )
        self.add_output(
            "data:propulsion:EM_power_rate", shape=shape, lower=0.0, upper=1.0
        )

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
        outputs["data:propulsion:shaft_power"] = flight_point.shaft_power
        outputs["data:propulsion:power_rate"] = flight_point.power_rate
        outputs["data:propulsion:max_thermodynamic_power"] = flight_point.thermo_power

        outputs["data:propulsion:TPshaft_power"] = flight_point.TPshaft_power
        outputs["data:propulsion:EMshaft_power"] = flight_point.EMshaft_power
        outputs["data:propulsion:BAT_power"] = flight_point.BAT_power
        outputs["data:propulsion:TP_power_rate"] = flight_point.TP_power_rate
        outputs["data:propulsion:EM_power_rate"] = flight_point.EM_power_rate
        # outputs["data:propulsion:H2_fc"]= flight_point.H2_fc
        outputs["data:propulsion:BAT_ec"] = flight_point.BAT_ec

    @staticmethod
    def get_wrapper() -> OMPHEBEngineMLL1Wrapper:
        return OMPHEBEngineMLL1Wrapper()


@RegisterPropulsion("rhea.wrapper.propulsion.PHFC_engine_ML_L1")
class OMPHFCEngineML_L1Wrapper(IOMPropulsionWrapper):
    """

    Wrapper class for ML_L1 parallel hybrid fuel cell engine model.

    It is made to allow a direct call to :class:`~.hybrid_engine.HybridEngine_ML_L1` in an OpenMDAO
    component.

    """

    def setup(self, component: Component):

        component.add_input("data:propulsion:RTO_power", np.nan, units="W")
        component.add_input("data:propulsion:Design_Thermo_Power", np.nan, units="W")
        component.add_input("data:propulsion:Power_Offtake", np.nan, units="W")
        component.add_input("data:propulsion:gearbox_eta", np.nan)
        component.add_input(
            "data:geometry:propulsion:propeller:diameter", np.nan, units="m"
        )

        # tuning factors
        component.add_input("tuning:propulsion:k_psfc", np.nan)
        component.add_input("tuning:propulsion:k_prop", np.nan)
        # component.add_input("tuning:propulsion:electric_systems:fuel_cell:k_eta", np.nan)

        # rating settings
        component.add_input("settings:propulsion:ratings:RTO:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:NTO:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:MCL:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:MCT:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:MCR:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:FID:k_gb", np.nan)

        # electric inputs
        component.add_input("data:propulsion:electric_systems:P_nom", np.nan, units="W")
        component.add_input(
            "data:propulsion:electric_systems:power_electronics:power_electronics_eta",
            np.nan,
        )
        component.add_input("data:propulsion:electric_systems:motor:motor_eta", np.nan)
        # component.add_input("data:propulsion:electric_systems:battery:battery_eta", np.nan)
        # component.add_input("data:propulsion:electric_systems:fuel_cell:fuel_cell_eta", np.nan)

        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:polarization_curve:V",
            np.nan,
            shape=771,
        )
        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:polarization_curve:i",
            np.nan,
            shape=771,
        )
        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:layout:stacks", np.nan
        )
        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:layout:packs", np.nan
        )
        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:sizing:A_cell",
            np.nan,
            units="m**2",
        )

        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:layout:cells", np.nan
        )
        component.add_input("data:geometry:propulsion:motor:count", np.nan)
        # component.add_output("data:propulsion:shaft_power", units="W")
        # component.add_output("data:propulsion:power_rate")
        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:Power_Offtake",
            np.nan,
            units="W",
        )
        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:Gross_net_power_ratio", np.nan
        )
        component._OPTIONS["prop_fid"]

    @staticmethod
    def get_model(inputs) -> IPropulsion:
        """

        :param inputs: input parameters that define the engine
        :return: an :class:`TPEngine_ML_L1` instance
        """
        engine_params = {
            "RTO_power": inputs["data:propulsion:RTO_power"],
            "Design_Thermo_Power": inputs["data:propulsion:Design_Thermo_Power"],
            "Power_Offtake": inputs["data:propulsion:Power_Offtake"],
            "gearbox_eta": inputs["data:propulsion:gearbox_eta"],
            "d_prop": inputs["data:geometry:propulsion:propeller:diameter"],
            "k_gb_RTO": inputs["settings:propulsion:ratings:RTO:k_gb"],
            "k_gb_NTO": inputs["settings:propulsion:ratings:NTO:k_gb"],
            "k_gb_MCL": inputs["settings:propulsion:ratings:MCL:k_gb"],
            "k_gb_MCT": inputs["settings:propulsion:ratings:MCT:k_gb"],
            "k_gb_MCR": inputs["settings:propulsion:ratings:MCR:k_gb"],
            "k_gb_FID": inputs["settings:propulsion:ratings:FID:k_gb"],
            "k_psfc": inputs["tuning:propulsion:k_psfc"],
            "k_prop": inputs["tuning:propulsion:k_prop"],
            # "k_fc":   inputs["tuning:propulsion:electric_systems:fuel_cell:k_eta"],
            "Elec_nom_power": inputs["data:propulsion:electric_systems:P_nom"],
            "power_electronics_eta": inputs[
                "data:propulsion:electric_systems:power_electronics:power_electronics_eta"
            ],
            "motor_eta": inputs["data:propulsion:electric_systems:motor:motor_eta"],
            # "battery_eta": inputs["data:propulsion:electric_systems:battery:battery_eta"],
            # "fuel_cell_eta": inputs["data:propulsion:electric_systems:fuel_cell:fuel_cell_eta"],
            "V_vec": inputs[
                "data:propulsion:electric_systems:fuel_cell:polarization_curve:V"
            ],
            "i_vec": inputs[
                "data:propulsion:electric_systems:fuel_cell:polarization_curve:i"
            ],
            "Nstacks": inputs[
                "data:propulsion:electric_systems:fuel_cell:layout:stacks"
            ],
            "Npacks": inputs["data:propulsion:electric_systems:fuel_cell:layout:packs"],
            "Acell": inputs["data:propulsion:electric_systems:fuel_cell:sizing:A_cell"],
            "Ncells": inputs["data:propulsion:electric_systems:fuel_cell:layout:cells"],
            "Nmotors": inputs["data:geometry:propulsion:motor:count"],
            "FC_Power_Offtake": inputs[
                "data:propulsion:electric_systems:fuel_cell:Power_Offtake"
            ],
            "Gross_net_power_ratio": inputs[
                "data:propulsion:electric_systems:fuel_cell:Gross_net_power_ratio"
            ],
        }

        return PHFCEngine_ML_L1(**engine_params)


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
class OMPHFCEngineML_L1Component(BaseOMPropulsionComponent):
    """
    Parametric engine model as OpenMDAO component

    See :class:`PHFCEngine_ML_L1` for more information.
    """

    # def initialize(self):
    #     self.options.declare("flight_point_count", 1, types=(int, tuple))

    def setup(self):
        super().setup()
        shape = self.options["flight_point_count"]
        self.add_output("data:propulsion:shaft_power", shape=shape, units="W")
        self.add_output("data:propulsion:power_rate", shape=shape, lower=0.0, upper=1.0)
        self.add_output(
            "data:propulsion:max_thermodynamic_power", shape=shape, units="W"
        )

        self.add_output("data:propulsion:TPshaft_power", shape=shape, units="W")
        self.add_output("data:propulsion:EMshaft_power", shape=shape, units="W")
        self.add_output("data:propulsion:FC_power", shape=shape, units="W")
        self.add_output("data:propulsion:H2_fc", shape=shape, units="kg/s")
        # self.add_output("data:propulsion:BAT_power", shape=shape, units="W")
        # self.add_output("data:propulsion:BAT_SOC", shape=shape)

        self.add_output(
            "data:propulsion:TP_power_rate", shape=shape, lower=0.0, upper=1.0
        )
        self.add_output(
            "data:propulsion:EM_power_rate", shape=shape, lower=0.0, upper=1.0
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:FC_efficiency", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:FC_airflow",
            shape=shape,
            units="kg/s",
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:V_cell", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:V_stack", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:V_pack", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:V_core", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:I_cell", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:I_stack", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:I_pack", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:I_core", shape=shape
        )

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
        outputs["data:propulsion:shaft_power"] = flight_point.shaft_power
        outputs["data:propulsion:power_rate"] = flight_point.power_rate
        outputs["data:propulsion:max_thermodynamic_power"] = flight_point.thermo_power

        outputs["data:propulsion:TPshaft_power"] = flight_point.TPshaft_power
        outputs["data:propulsion:EMshaft_power"] = flight_point.EMshaft_power
        outputs["data:propulsion:FC_power"] = flight_point.FC_power
        # outputs["data:propulsion:BAT_power"]=flight_point.BAT_power
        outputs["data:propulsion:TP_power_rate"] = flight_point.TP_power_rate
        outputs["data:propulsion:EM_power_rate"] = flight_point.EM_power_rate
        outputs["data:propulsion:H2_fc"] = flight_point.H2_fc
        # outputs["data:propulsion:BAT_SOC"]= flight_point.BAT_SOC
        outputs[
            "data:propulsion:electric_systems:fuel_cell:FC_efficiency"
        ] = flight_point.FC_efficiency
        outputs[
            "data:propulsion:electric_systems:fuel_cell:FC_airflow"
        ] = flight_point.FC_airflow
        outputs[
            "data:propulsion:electric_systems:fuel_cell:V_cell"
        ] = flight_point.V_cell
        outputs[
            "data:propulsion:electric_systems:fuel_cell:V_stack"
        ] = flight_point.V_stack
        outputs[
            "data:propulsion:electric_systems:fuel_cell:V_pack"
        ] = flight_point.V_pack
        outputs[
            "data:propulsion:electric_systems:fuel_cell:V_core"
        ] = flight_point.V_core
        outputs[
            "data:propulsion:electric_systems:fuel_cell:I_cell"
        ] = flight_point.I_cell
        outputs[
            "data:propulsion:electric_systems:fuel_cell:I_stack"
        ] = flight_point.I_stack
        outputs[
            "data:propulsion:electric_systems:fuel_cell:I_pack"
        ] = flight_point.I_pack
        outputs[
            "data:propulsion:electric_systems:fuel_cell:I_core"
        ] = flight_point.I_core

    @staticmethod
    def get_wrapper() -> OMPHFCEngineML_L1Wrapper:
        return OMPHFCEngineML_L1Wrapper()


@RegisterPropulsion("rhea.wrapper.propulsion.PHFC_engine_L0")
class OMPHFCEngineL0Wrapper(IOMPropulsionWrapper):
    """

    Wrapper class for L0 parallel hybrid fuel cell engine model.

    It is made to allow a direct call to :class:`~.hybrid_engine.HybridEngine_L0` in an OpenMDAO
    component.

    """

    def setup(self, component: Component):
        component.add_input("data:propulsion:RTO_power", np.nan, units="W")
        component.add_input("data:propulsion:Design_Thermo_Power", np.nan, units="W")
        component.add_input("data:propulsion:Power_Offtake", np.nan, units="W")
        component.add_input("data:propulsion:gearbox_eta", np.nan)
        component.add_input(
            "data:geometry:propulsion:propeller:diameter", np.nan, units="m"
        )

        component.add_input("data:propulsion:electric_systems:P_nom", np.nan, units="W")
        component.add_input(
            "data:propulsion:electric_systems:power_electronics:power_electronics_eta",
            np.nan,
        )
        component.add_input("data:propulsion:electric_systems:motor:motor_eta", np.nan)
        # component.add_input("data:propulsion:electric_systems:battery:battery_eta", np.nan)
        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:fuel_cell_eta", np.nan
        )
        # component.add_output("data:propulsion:shaft_power", units="W")
        # component.add_output("data:propulsion:power_rate")

        # tuning factors
        component.add_input("tuning:propulsion:k_psfc", np.nan)
        component.add_input("tuning:propulsion:k_prop", np.nan)

        # rating settings
        component.add_input("settings:propulsion:ratings:RTO:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:NTO:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:MCL:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:MCT:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:MCR:k_gb", np.nan)
        component.add_input("settings:propulsion:ratings:FID:k_gb", np.nan)

        component.add_input("data:geometry:propulsion:motor:count", np.nan)

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
            "d_prop": inputs["data:geometry:propulsion:propeller:diameter"],
            "Elec_nom_power": inputs["data:propulsion:electric_systems:P_nom"],
            "power_electronics_eta": inputs[
                "data:propulsion:electric_systems:power_electronics:power_electronics_eta"
            ],
            "motor_eta": inputs["data:propulsion:electric_systems:motor:motor_eta"],
            # "battery_eta": inputs["data:propulsion:electric_systems:battery:battery_eta"],
            "fuel_cell_eta": inputs[
                "data:propulsion:electric_systems:fuel_cell:fuel_cell_eta"
            ],
            "k_gb_RTO": inputs["settings:propulsion:ratings:RTO:k_gb"],
            "k_gb_NTO": inputs["settings:propulsion:ratings:NTO:k_gb"],
            "k_gb_MCL": inputs["settings:propulsion:ratings:MCL:k_gb"],
            "k_gb_MCT": inputs["settings:propulsion:ratings:MCT:k_gb"],
            "k_gb_MCR": inputs["settings:propulsion:ratings:MCR:k_gb"],
            "k_gb_FID": inputs["settings:propulsion:ratings:FID:k_gb"],
            "k_psfc": inputs["tuning:propulsion:k_psfc"],
            "k_prop": inputs["tuning:propulsion:k_prop"],
            "Nmotors": inputs["data:geometry:propulsion:motor:count"],
        }

        return PHFCEngine_L0(**engine_params)


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
class OMPHFCEngineL0Component(BaseOMPropulsionComponent):
    """
    Parametric engine model as OpenMDAO component

    See :class:`PHFCEngine_L0` for more information.
    """

    def setup(self):
        super().setup()
        shape = self.options["flight_point_count"]
        self.add_output("data:propulsion:shaft_power", shape=shape, units="W")
        self.add_output("data:propulsion:power_rate", shape=shape, lower=0.0, upper=1.0)
        self.add_output(
            "data:propulsion:max_thermodynamic_power", shape=shape, units="W"
        )

        self.add_output("data:propulsion:TPshaft_power", shape=shape, units="W")
        self.add_output("data:propulsion:EMshaft_power", shape=shape, units="W")
        self.add_output("data:propulsion:FC_power", shape=shape, units="W")
        self.add_output("data:propulsion:H2_fc", shape=shape, units="kg/s")
        # self.add_output("data:propulsion:BAT_power", shape=shape, units="W")
        # self.add_output("data:propulsion:BAT_SOC", shape=shape)
        self.add_output(
            "data:propulsion:TP_power_rate", shape=shape, lower=0.0, upper=1.0
        )
        self.add_output(
            "data:propulsion:EM_power_rate", shape=shape, lower=0.0, upper=1.0
        )

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
        outputs["data:propulsion:shaft_power"] = flight_point.shaft_power
        outputs["data:propulsion:power_rate"] = flight_point.power_rate
        outputs["data:propulsion:max_thermodynamic_power"] = flight_point.thermo_power

        outputs["data:propulsion:TPshaft_power"] = flight_point.TPshaft_power
        outputs["data:propulsion:EMshaft_power"] = flight_point.EMshaft_power
        outputs["data:propulsion:FC_power"] = flight_point.FC_power
        # outputs["data:propulsion:BAT_power"]=flight_point.BAT_power
        outputs["data:propulsion:TP_power_rate"] = flight_point.TP_power_rate
        outputs["data:propulsion:EM_power_rate"] = flight_point.EM_power_rate
        outputs["data:propulsion:H2_fc"] = flight_point.H2_fc
        # outputs["data:propulsion:BAT_SOC"]= flight_point.BAT_SOC

    @staticmethod
    def get_wrapper() -> OMPHFCEngineL0Wrapper:
        return OMPHFCEngineL0Wrapper()


@RegisterPropulsion("rhea.wrapper.propulsion.PHFC_engine_L1")
class OMPHFCEngineL1Wrapper(IOMPropulsionWrapper):
    """

    Wrapper class for L0 parallel hybrid fuel cell engine model.

    It is made to allow a direct call to :class:`~.hybrid_engine.HybridEngine_L1` in an OpenMDAO
    component.

    """

    def setup(self, component: Component):
        component.add_input("data:propulsion:RTO_power", np.nan, units="W")
        component.add_input("data:propulsion:Design_Thermo_Power", np.nan, units="W")
        component.add_input("data:propulsion:Power_Offtake", np.nan, units="W")
        component.add_input("data:propulsion:gearbox_eta", np.nan)
        component.add_input(
            "data:geometry:propulsion:propeller:diameter", np.nan, units="m"
        )

        component.add_input("data:propulsion:L1_engine:fuel", np.nan)
        # design values
        component.add_input(
            "data:propulsion:L1_engine:turbine_inlet_temperature", np.nan, units="K"
        )
        component.add_input("data:propulsion:L1_engine:HP_bleed", np.nan)
        component.add_input("data:propulsion:L1_engine:LP_bleed", np.nan)

        component.add_input("data:propulsion:L1_engine:inlet:inlet_eta_pol", np.nan)
        component.add_input(
            "data:propulsion:L1_engine:inlet:inlet_pressure_ratio", np.nan
        )
        component.add_input("data:propulsion:L1_engine:lpc:lpc_eta_pol", np.nan)
        component.add_input("data:propulsion:L1_engine:lpc:lpc_pressure_ratio", np.nan)
        component.add_input("data:propulsion:L1_engine:hpc:hpc_eta_pol", np.nan)
        component.add_input("data:propulsion:L1_engine:hpc:hpc_pressure_ratio", np.nan)
        component.add_input("data:propulsion:L1_engine:combustor:combustor_eta", np.nan)
        component.add_input(
            "data:propulsion:L1_engine:combustor:combustor_pressure_ratio", np.nan
        )
        component.add_input("data:propulsion:L1_engine:hpt:hpt_eta_pol", np.nan)
        component.add_input("data:propulsion:L1_engine:hpt:hpt_eta_mech", np.nan)
        component.add_input("data:propulsion:L1_engine:lpt:lpt_eta_pol", np.nan)
        component.add_input("data:propulsion:L1_engine:lpt:lpt_eta_mech", np.nan)
        component.add_input("data:propulsion:L1_engine:pt:pt_eta_pol", np.nan)
        component.add_input("data:propulsion:L1_engine:pt:pt_eta_mech", np.nan)
        component.add_input("data:propulsion:L1_engine:nozzle:nozzle_eta_pol", np.nan)
        component.add_input(
            "data:propulsion:L1_engine:nozzle:nozzle_pressure_ratio", np.nan
        )
        component.add_input(
            "data:propulsion:L1_engine:nozzle:nozzle_area_ratio", np.nan
        )

        # sizing factors
        component.add_input("data:propulsion:L1_engine:sizing:k0", np.nan)
        component.add_input("data:propulsion:L1_engine:sizing:k1", np.nan)
        component.add_input("data:propulsion:L1_engine:sizing:k2", np.nan)
        component.add_input("data:propulsion:L1_engine:sizing:tau_t_sizing", np.nan)
        component.add_input("data:propulsion:L1_engine:sizing:pi_t_sizing", np.nan)
        component.add_input("data:propulsion:L1_engine:sizing:M_out_sizing", np.nan)

        # rating settings
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

        # tuning factors
        component.add_input("tuning:propulsion:k_psfc", np.nan)
        component.add_input("tuning:propulsion:k_prop", np.nan)

        # electric inputs
        component.add_input("data:propulsion:electric_systems:P_nom", np.nan, units="W")
        component.add_input(
            "data:propulsion:electric_systems:power_electronics:power_electronics_eta",
            np.nan,
        )
        component.add_input("data:propulsion:electric_systems:motor:motor_eta", np.nan)
        # component.add_input("data:propulsion:electric_systems:battery:battery_eta", np.nan)
        # component.add_input("data:propulsion:electric_systems:fuel_cell:fuel_cell_eta", np.nan)

        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:polarization_curve:V",
            np.nan,
            shape=771,
        )
        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:polarization_curve:i",
            np.nan,
            shape=771,
        )
        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:layout:stacks", np.nan
        )
        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:layout:packs", np.nan
        )
        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:sizing:A_cell",
            np.nan,
            units="m**2",
        )

        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:layout:cells", np.nan
        )
        component.add_input("data:geometry:propulsion:motor:count", np.nan)
        # component.add_output("data:propulsion:shaft_power", units="W")
        # component.add_output("data:propulsion:power_rate")
        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:Power_Offtake",
            np.nan,
            units="W",
        )
        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:Gross_net_power_ratio", np.nan
        )

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
            "d_prop": inputs["data:geometry:propulsion:propeller:diameter"],
            "fuel": inputs["data:propulsion:L1_engine:fuel"],
            "turbine_inlet_temperature": inputs[
                "data:propulsion:L1_engine:turbine_inlet_temperature"
            ],
            "HP_bleed": inputs["data:propulsion:L1_engine:HP_bleed"],
            "LP_bleed": inputs["data:propulsion:L1_engine:LP_bleed"],
            "inlet_eta_pol": inputs["data:propulsion:L1_engine:inlet:inlet_eta_pol"],
            "inlet_pressure_ratio": inputs[
                "data:propulsion:L1_engine:inlet:inlet_pressure_ratio"
            ],
            "lpc_eta_pol": inputs["data:propulsion:L1_engine:lpc:lpc_eta_pol"],
            "lpc_pressure_ratio": inputs[
                "data:propulsion:L1_engine:lpc:lpc_pressure_ratio"
            ],
            "hpc_eta_pol": inputs["data:propulsion:L1_engine:hpc:hpc_eta_pol"],
            "hpc_pressure_ratio": inputs[
                "data:propulsion:L1_engine:hpc:hpc_pressure_ratio"
            ],
            "combustor_eta": inputs[
                "data:propulsion:L1_engine:combustor:combustor_eta"
            ],
            "combustor_pressure_ratio": inputs[
                "data:propulsion:L1_engine:combustor:combustor_pressure_ratio"
            ],
            "hpt_eta_pol": inputs["data:propulsion:L1_engine:hpt:hpt_eta_pol"],
            "hpt_eta_mech": inputs["data:propulsion:L1_engine:hpt:hpt_eta_mech"],
            "lpt_eta_pol": inputs["data:propulsion:L1_engine:lpt:lpt_eta_pol"],
            "lpt_eta_mech": inputs["data:propulsion:L1_engine:lpt:lpt_eta_mech"],
            "pt_eta_pol": inputs["data:propulsion:L1_engine:pt:pt_eta_pol"],
            "pt_eta_mech": inputs["data:propulsion:L1_engine:pt:pt_eta_mech"],
            "nozzle_eta_pol": inputs["data:propulsion:L1_engine:nozzle:nozzle_eta_pol"],
            "nozzle_pressure_ratio": inputs[
                "data:propulsion:L1_engine:nozzle:nozzle_pressure_ratio"
            ],
            "nozzle_area_ratio": inputs[
                "data:propulsion:L1_engine:nozzle:nozzle_area_ratio"
            ],
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
            "k_psfc": inputs["tuning:propulsion:k_psfc"],
            "k_prop": inputs["tuning:propulsion:k_prop"],
            "Elec_nom_power": inputs["data:propulsion:electric_systems:P_nom"],
            "power_electronics_eta": inputs[
                "data:propulsion:electric_systems:power_electronics:power_electronics_eta"
            ],
            "motor_eta": inputs["data:propulsion:electric_systems:motor:motor_eta"],
            # "battery_eta": inputs["data:propulsion:electric_systems:battery:battery_eta"],
            # "fuel_cell_eta": inputs["data:propulsion:electric_systems:fuel_cell:fuel_cell_eta"],
            "V_vec": inputs[
                "data:propulsion:electric_systems:fuel_cell:polarization_curve:V"
            ],
            "i_vec": inputs[
                "data:propulsion:electric_systems:fuel_cell:polarization_curve:i"
            ],
            "Nstacks": inputs[
                "data:propulsion:electric_systems:fuel_cell:layout:stacks"
            ],
            "Npacks": inputs["data:propulsion:electric_systems:fuel_cell:layout:packs"],
            "Acell": inputs["data:propulsion:electric_systems:fuel_cell:sizing:A_cell"],
            "Ncells": inputs["data:propulsion:electric_systems:fuel_cell:layout:cells"],
            "Nmotors": inputs["data:geometry:propulsion:motor:count"],
            "FC_Power_Offtake": inputs[
                "data:propulsion:electric_systems:fuel_cell:Power_Offtake"
            ],
            "Gross_net_power_ratio": inputs[
                "data:propulsion:electric_systems:fuel_cell:Gross_net_power_ratio"
            ],
        }

        return PHFCEngine_L1(**engine_params)


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
class OMPHFCEngineL1Component(BaseOMPropulsionComponent):
    """
    Parametric engine model as OpenMDAO component

    See :class:`PHFCEngine_L1` for more information.
    """

    def setup(self):
        super().setup()
        shape = self.options["flight_point_count"]
        self.add_output("data:propulsion:shaft_power", shape=shape, units="W")
        self.add_output("data:propulsion:power_rate", shape=shape, lower=0.0, upper=1.0)
        self.add_output(
            "data:propulsion:max_thermodynamic_power", shape=shape, units="W"
        )

        self.add_output("data:propulsion:TP_thermal_efficiency", shape=shape)
        self.add_output("data:propulsion:TP_residual_thrust", shape=shape, units="N")
        self.add_output("data:propulsion:TP_air_flow", shape=shape, units="kg/s")
        self.add_output("data:propulsion:TP_total_pressure", shape=shape)
        self.add_output("data:propulsion:TP_total_temperature", shape=shape, units="K")

        self.add_output("data:propulsion:TPshaft_power", shape=shape, units="W")
        self.add_output("data:propulsion:EMshaft_power", shape=shape, units="W")
        self.add_output("data:propulsion:FC_power", shape=shape, units="W")
        self.add_output("data:propulsion:FC_stack_power", shape=shape, units="W")

        self.add_output("data:propulsion:H2_fc", shape=shape, units="kg/s")
        # self.add_output("data:propulsion:BAT_power", shape=shape, units="W")
        # self.add_output("data:propulsion:BAT_SOC", shape=shape)
        self.add_output(
            "data:propulsion:TP_power_rate", shape=shape, lower=0.0, upper=1.0
        )
        self.add_output(
            "data:propulsion:EM_power_rate", shape=shape, lower=0.0, upper=1.0
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:FC_efficiency", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:FC_airflow",
            shape=shape,
            units="kg/s",
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:V_cell", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:V_stack", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:V_pack", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:V_core", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:I_cell", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:I_stack", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:I_pack", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:I_core", shape=shape
        )

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
        outputs["data:propulsion:shaft_power"] = flight_point.shaft_power
        outputs["data:propulsion:power_rate"] = flight_point.power_rate
        outputs["data:propulsion:max_thermodynamic_power"] = flight_point.thermo_power

        outputs[
            "data:propulsion:TP_thermal_efficiency"
        ] = flight_point.TP_thermal_efficiency
        outputs["data:propulsion:TP_residual_thrust"] = flight_point.TP_residual_thrust
        outputs["data:propulsion:TP_air_flow"] = flight_point.TP_air_flow
        outputs["data:propulsion:TP_total_pressure"] = flight_point.TP_total_pressure
        outputs[
            "data:propulsion:TP_total_temperature"
        ] = flight_point.TP_total_temperature

        outputs["data:propulsion:TPshaft_power"] = flight_point.TPshaft_power
        outputs["data:propulsion:EMshaft_power"] = flight_point.EMshaft_power
        outputs["data:propulsion:FC_power"] = flight_point.FC_power

        outputs["data:propulsion:FC_stack_power"] = flight_point.FC_stack_power

        # outputs["data:propulsion:BAT_power"]=flight_point.BAT_power
        outputs["data:propulsion:TP_power_rate"] = flight_point.TP_power_rate
        outputs["data:propulsion:EM_power_rate"] = flight_point.EM_power_rate
        outputs["data:propulsion:H2_fc"] = flight_point.H2_fc
        # outputs["data:propulsion:BAT_SOC"]= flight_point.BAT_SOC
        outputs[
            "data:propulsion:electric_systems:fuel_cell:FC_efficiency"
        ] = flight_point.FC_efficiency
        outputs[
            "data:propulsion:electric_systems:fuel_cell:FC_airflow"
        ] = flight_point.FC_airflow
        outputs[
            "data:propulsion:electric_systems:fuel_cell:V_cell"
        ] = flight_point.V_cell
        outputs[
            "data:propulsion:electric_systems:fuel_cell:V_stack"
        ] = flight_point.V_stack
        outputs[
            "data:propulsion:electric_systems:fuel_cell:V_pack"
        ] = flight_point.V_pack
        outputs[
            "data:propulsion:electric_systems:fuel_cell:V_core"
        ] = flight_point.V_core
        outputs[
            "data:propulsion:electric_systems:fuel_cell:I_cell"
        ] = flight_point.I_cell
        outputs[
            "data:propulsion:electric_systems:fuel_cell:I_stack"
        ] = flight_point.I_stack
        outputs[
            "data:propulsion:electric_systems:fuel_cell:I_pack"
        ] = flight_point.I_pack
        outputs[
            "data:propulsion:electric_systems:fuel_cell:I_core"
        ] = flight_point.I_core

    @staticmethod
    def get_wrapper() -> OMPHFCEngineL1Wrapper:
        return OMPHFCEngineL1Wrapper()


@RegisterPropulsion("rhea.wrapper.propulsion.FHFC_engine_L1")
class OMFHFCEngineL1Wrapper(IOMPropulsionWrapper):
    """

    Wrapper class for L1 full hydrogen fuel cell engine model.

    It is made to allow a direct call to :class:`~.hybrid_engine.HybridEngine_L1` in an OpenMDAO
    component.

    """

    def setup(self, component: Component):

        component.add_input("data:propulsion:gearbox_eta", np.nan)
        component.add_input(
            "data:geometry:propulsion:propeller:diameter", np.nan, units="m"
        )

        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:Power_Offtake",
            np.nan,
            units="W",
        )
        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:Gross_net_power_ratio", np.nan
        )
        # design values

        # tuning factors
        component.add_input("tuning:propulsion:k_prop", np.nan)

        # electric inputs
        component.add_input("data:propulsion:electric_systems:P_nom", np.nan, units="W")
        component.add_input(
            "data:propulsion:electric_systems:power_electronics:power_electronics_eta",
            np.nan,
        )
        component.add_input("data:propulsion:electric_systems:motor:motor_eta", np.nan)
        # component.add_input("data:propulsion:electric_systems:battery:battery_eta", np.nan)
        # component.add_input("data:propulsion:electric_systems:fuel_cell:fuel_cell_eta", np.nan)

        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:polarization_curve:V",
            np.nan,
            shape=771,
        )
        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:polarization_curve:i",
            np.nan,
            shape=771,
        )
        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:layout:stacks", np.nan
        )
        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:layout:packs", np.nan
        )
        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:sizing:A_cell",
            np.nan,
            units="m**2",
        )

        component.add_input(
            "data:propulsion:electric_systems:fuel_cell:layout:cells", np.nan
        )
        component.add_input("data:geometry:propulsion:motor:count", np.nan)
        # component.add_output("data:propulsion:shaft_power", units="W")
        # component.add_output("data:propulsion:power_rate")

    @staticmethod
    def get_model(inputs) -> IPropulsion:
        """

        :param inputs: input parameters that define the engine
        :return: an :class:`TPEngine_L0` instance
        """
        engine_params = {
            "FC_Power_Offtake": inputs[
                "data:propulsion:electric_systems:fuel_cell:Power_Offtake"
            ],
            "Gross_net_power_ratio": inputs[
                "data:propulsion:electric_systems:fuel_cell:Gross_net_power_ratio"
            ],
            "gearbox_eta": inputs["data:propulsion:gearbox_eta"],
            "d_prop": inputs["data:geometry:propulsion:propeller:diameter"],
            "k_prop": inputs["tuning:propulsion:k_prop"],
            "Elec_nom_power": inputs["data:propulsion:electric_systems:P_nom"],
            "power_electronics_eta": inputs[
                "data:propulsion:electric_systems:power_electronics:power_electronics_eta"
            ],
            "motor_eta": inputs["data:propulsion:electric_systems:motor:motor_eta"],
            # "battery_eta": inputs["data:propulsion:electric_systems:battery:battery_eta"],
            # "fuel_cell_eta": inputs["data:propulsion:electric_systems:fuel_cell:fuel_cell_eta"],
            "V_vec": inputs[
                "data:propulsion:electric_systems:fuel_cell:polarization_curve:V"
            ],
            "i_vec": inputs[
                "data:propulsion:electric_systems:fuel_cell:polarization_curve:i"
            ],
            "Nstacks": inputs[
                "data:propulsion:electric_systems:fuel_cell:layout:stacks"
            ],
            "Npacks": inputs["data:propulsion:electric_systems:fuel_cell:layout:packs"],
            "Acell": inputs["data:propulsion:electric_systems:fuel_cell:sizing:A_cell"],
            "Ncells": inputs["data:propulsion:electric_systems:fuel_cell:layout:cells"],
            "Nmotors": inputs["data:geometry:propulsion:motor:count"],
        }

        return FHFCEngine_L1(**engine_params)


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
class OMFHFCEngineL1Component(BaseOMPropulsionComponent):
    """
    Parametric engine model as OpenMDAO component

    See :class:`PHFCEngine_L1` for more information.
    """

    def setup(self):
        super().setup()
        shape = self.options["flight_point_count"]
        self.add_output("data:propulsion:shaft_power", shape=shape, units="W")
        self.add_output("data:propulsion:power_rate", shape=shape, lower=0.0, upper=1.0)

        self.add_output("data:propulsion:EMshaft_power", shape=shape, units="W")
        self.add_output("data:propulsion:FC_power", shape=shape, units="W")
        self.add_output("data:propulsion:FC_stack_power", shape=shape, units="W")

        self.add_output("data:propulsion:H2_fc", shape=shape, units="kg/s")
        # self.add_output("data:propulsion:BAT_power", shape=shape, units="W")
        # self.add_output("data:propulsion:BAT_SOC", shape=shape)
        self.add_output(
            "data:propulsion:EM_power_rate", shape=shape, lower=0.0, upper=1.0
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:FC_efficiency", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:FC_airflow",
            shape=shape,
            units="kg/s",
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:V_cell", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:V_stack", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:V_pack", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:V_core", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:I_cell", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:I_stack", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:I_pack", shape=shape
        )
        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:I_core", shape=shape
        )

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

        outputs["data:propulsion:thrust_rate"] = flight_point.thrust_rate
        outputs["data:propulsion:thrust"] = flight_point.thrust
        outputs["data:propulsion:shaft_power"] = flight_point.shaft_power
        outputs["data:propulsion:power_rate"] = flight_point.power_rate

        outputs["data:propulsion:EMshaft_power"] = flight_point.EMshaft_power
        outputs["data:propulsion:FC_power"] = flight_point.FC_power

        outputs["data:propulsion:FC_stack_power"] = flight_point.FC_stack_power

        # outputs["data:propulsion:BAT_power"]=flight_point.BAT_power
        outputs["data:propulsion:EM_power_rate"] = flight_point.EM_power_rate
        outputs["data:propulsion:H2_fc"] = flight_point.H2_fc
        # outputs["data:propulsion:BAT_SOC"]= flight_point.BAT_SOC
        outputs[
            "data:propulsion:electric_systems:fuel_cell:FC_efficiency"
        ] = flight_point.FC_efficiency
        outputs[
            "data:propulsion:electric_systems:fuel_cell:FC_airflow"
        ] = flight_point.FC_airflow
        outputs[
            "data:propulsion:electric_systems:fuel_cell:V_cell"
        ] = flight_point.V_cell
        outputs[
            "data:propulsion:electric_systems:fuel_cell:V_stack"
        ] = flight_point.V_stack
        outputs[
            "data:propulsion:electric_systems:fuel_cell:V_pack"
        ] = flight_point.V_pack
        outputs[
            "data:propulsion:electric_systems:fuel_cell:V_core"
        ] = flight_point.V_core
        outputs[
            "data:propulsion:electric_systems:fuel_cell:I_cell"
        ] = flight_point.I_cell
        outputs[
            "data:propulsion:electric_systems:fuel_cell:I_stack"
        ] = flight_point.I_stack
        outputs[
            "data:propulsion:electric_systems:fuel_cell:I_pack"
        ] = flight_point.I_pack
        outputs[
            "data:propulsion:electric_systems:fuel_cell:I_core"
        ] = flight_point.I_core

    @staticmethod
    def get_wrapper() -> OMFHFCEngineL1Wrapper:
        return OMFHFCEngineL1Wrapper()
