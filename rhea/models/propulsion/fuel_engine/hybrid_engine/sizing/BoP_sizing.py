"""
Weight computation (mass and CG)
"""
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

import openmdao.api as om

# from fastoad.models.weight.cg.cg import ComputeAircraftCG
# from rhea.models.weight.cg.cg_RHEA import CG_RHEA
# from rhea.models.weight.mass_breakdown import MassBreakdown_RHEA
import numpy as np
from openmdao.core.explicitcomponent import ExplicitComponent

# from models.propulsion.fuel_engine.hybrid_engine.engine_components.Fuel_cell_L1 import Fuel_cell
import pandas as pd
import os

script_path = os.path.abspath(__file__)  # i.e. /path/to/dir/foobar.py
rhea_path = script_path.split("\\models")[0]


class BoP_sizing(om.Group):
    """
    Weight estimation for hybrid propulsion systems


    """

    def setup(self):

        self.add_subsystem("fc_system", evaluate_fc_system(), promotes=["*"])
        self.add_subsystem(
            "H2_storage_distribution_system", evaluate_storage_system(), promotes=["*"]
        )


class evaluate_fc_system(ExplicitComponent):
    def setup(self):
        self.add_input("data:propulsion:electric_systems:P_nom", val=np.nan, units="W")
        self.add_input("data:propulsion:electric_systems:motor:motor_eta", val=np.nan)
        self.add_input(
            "data:propulsion:electric_systems:power_electronics:power_electronics_eta",
            val=np.nan,
        )
        self.add_input(
            "data:propulsion:electric_systems:fuel_cell:Power_Offtake",
            np.nan,
            units="W",
        )
        self.add_input(
            "data:propulsion:electric_systems:fuel_cell:Gross_net_power_ratio", np.nan
        )
        self.add_input("tuning:propulsion:electric_systems:fuel_cell:k_sp", 1.0)

        self.add_output(
            "data:propulsion:electric_systems:fuel_cell:specific_power",
            val=1.0,
            units="W/kg",
        )

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        P_nom = inputs["data:propulsion:electric_systems:P_nom"]
        motor_eta = inputs["data:propulsion:electric_systems:motor:motor_eta"]
        power_elec_eta = inputs[
            "data:propulsion:electric_systems:power_electronics:power_electronics_eta"
        ]
        Power_offtake = inputs[
            "data:propulsion:electric_systems:fuel_cell:Power_Offtake"
        ]
        Gross_net_power_ratio = inputs[
            "data:propulsion:electric_systems:fuel_cell:Gross_net_power_ratio"
        ]
        k_sp_fc = inputs["tuning:propulsion:electric_systems:fuel_cell:k_sp"]

        P_fc = P_nom / motor_eta / power_elec_eta  # fc gross power
        P_stacks = P_fc / Gross_net_power_ratio + Power_offtake  # fc net power

        df = pd.read_csv(
            os.path.join(
                rhea_path, "resources/hybrid_systems/FC_system/data/Psp_vs_P.csv"
            )
        )
        Psp = np.interp(P_stacks / 1000, df.Net_Power, df.P_sp_net)

        outputs["data:propulsion:electric_systems:fuel_cell:specific_power"] = (
            Psp * k_sp_fc
        )


class evaluate_storage_system(ExplicitComponent):
    def setup(self):
        self.add_input("data:mission:sizing:H2", val=np.nan, units="kg")
        self.add_input(
            "data:propulsion:electric_systems:fuel_cell:H2_flow_rate",
            val=np.nan,
            units="kg/s",
        )
        self.add_input("tuning:propulsion:electric_systems:H2_storage:k_length", 1.0)
        self.add_input("tuning:propulsion:electric_systems:H2_storage:k_eta", 1.0)

        self.add_output(
            "data:propulsion:electric_systems:H2_storage:efficiency", val=0.3
        )
        self.add_output(
            "data:propulsion:electric_systems:H2_storage:volume", val=0.0, units="m**3"
        )
        self.add_output(
            "data:propulsion:electric_systems:H2_storage:length", val=0.0, units="m"
        )
        self.add_output(
            "data:propulsion:electric_systems:H2_storage:diameter", val=0.0, units="m"
        )

        self.add_output(
            "data:propulsion:electric_systems:H2_distribution:mass_ratio", val=1.0
        )

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        m_H2 = inputs["data:mission:sizing:H2"]
        k_l_tank = inputs["tuning:propulsion:electric_systems:H2_storage:k_length"]
        k_eta_tank = inputs["tuning:propulsion:electric_systems:H2_storage:k_eta"]

        H2_flow_rate = inputs["data:propulsion:electric_systems:fuel_cell:H2_flow_rate"]

        df = pd.read_csv(
            os.path.join(
                rhea_path,
                "resources/hybrid_systems/H2_storage_distribution/data/Tank_geom1/eta_g_vs_m_H2.csv",
            )
        )
        eta_g = np.interp(m_H2, df.m_H2, df.eta_g)

        df = pd.read_csv(
            os.path.join(
                rhea_path,
                "resources/hybrid_systems/H2_storage_distribution/data/Tank_geom1/tank_vol_vs_m_H2.csv",
            )
        )
        tank_vol = np.interp(m_H2, df.m_H2, df.vol)

        df = pd.read_csv(
            os.path.join(
                rhea_path,
                "resources/hybrid_systems/H2_storage_distribution/data/Tank_geom1/tank_len_vs_m_H2.csv",
            )
        )
        tank_len = np.interp(m_H2, df.m_H2, df.tank_len)

        df = pd.read_csv(
            os.path.join(
                rhea_path,
                "resources/hybrid_systems/H2_storage_distribution/data/Tank_geom1/tank_dia_vs_m_H2.csv",
            )
        )
        tank_diam = np.interp(m_H2, df.m_H2, df.dia)

        df1 = pd.read_csv(
            os.path.join(
                rhea_path,
                "resources/hybrid_systems/H2_storage_distribution/data/m_pipes_vs_flow_rate.csv",
            )
        )
        mass_ratio = np.interp(H2_flow_rate, df1.H2_flow, df1.m_ratio)

        outputs["data:propulsion:electric_systems:H2_storage:efficiency"] = (
            eta_g * k_eta_tank
        )
        outputs["data:propulsion:electric_systems:H2_storage:volume"] = tank_vol
        outputs["data:propulsion:electric_systems:H2_storage:length"] = (
            tank_len * k_l_tank
        )
        outputs["data:propulsion:electric_systems:H2_storage:diameter"] = tank_diam
        outputs[
            "data:propulsion:electric_systems:H2_distribution:mass_ratio"
        ] = mass_ratio
