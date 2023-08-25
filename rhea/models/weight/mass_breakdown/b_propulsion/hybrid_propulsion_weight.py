# -*- coding: utf-8 -*-
"""
Created on Mon Sep 14 10:12:50 2020

@author: LA202059
"""

from fastoad.module_management.service_registry import RegisterSubmodel
import numpy as np
from openmdao.core.explicitcomponent import ExplicitComponent
from scipy import constants
import os
import joblib
import pandas as pd
from fastoad.model_base.atmosphere import Atmosphere
from scipy import interpolate
from fastoad.constants import FlightPhase
from rhea.models.propulsion.fuel_engine.hybrid_engine.engine_components.Fuel_cell_L1 import (
    Fuel_cell,
)
from rhea.models.propulsion.fuel_engine.constants import POWER_RATE_COUNT
import math

script_path = os.path.abspath(__file__)  # i.e. /path/to/dir/foobar.py
rhea_path = script_path.split("\\models")[0]
RHEA_path = script_path.split("\\rhea")[0]


class HybridPropulsionWeight(ExplicitComponent):
    """
    Weight estimation for hybrid propulsion systems


    """

    def __init__(
        self, out_file=os.path.join(RHEA_path, "workdir/sizing_conditions.csv")
    ):
        super().__init__()

        self.out_file = out_file

    def setup(self):
        self.add_input("data:propulsion:electric_systems:P_nom", val=np.nan, units="W")
        self.add_input(
            "data:propulsion:electric_systems:fuel_cell:sizing:sizing_point:P_sizing",
            0.0,
            units="W",
        )
        self.add_input(
            "data:propulsion:electric_systems:fuel_cell:sizing:T_op", 353.15, units="K"
        )
        self.add_input(
            "data:propulsion:electric_systems:fuel_cell:sizing:A_cell", units="m**2"
        )

        self.add_input(
            "data:propulsion:electric_systems:cables:length", 30.0, units="m"
        )
        self.add_input("data:propulsion:electric_systems:cables:material", 1)
        self.add_input(
            "data:propulsion:electric_systems:H2_distribution:pipes_length",
            50.0,
            units="m",
        )

        self.add_input("data:geometry:propulsion:motor:count", val=np.nan)
        self.add_input(
            "data:propulsion:electric_systems:power_electronics:specific_power",
            val=np.nan,
            units="W/kg",
        )
        self.add_input(
            "data:propulsion:electric_systems:motor:specific_power",
            val=np.nan,
            units="W/kg",
        )
        self.add_input(
            "data:propulsion:electric_systems:fuel_cell:specific_power",
            val=1000.0,
            units="W/kg",
        )
        self.add_input(
            "data:propulsion:electric_systems:H2_storage:efficiency", val=0.3
        )
        self.add_input(
            "data:propulsion:electric_systems:H2_distribution:mass_ratio", val=1
        )
        self.add_input("data:propulsion:electric_systems:fuel_cell:layout:packs", 1)
        self.add_input(
            "data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:i_pack"
        )

        self.add_input(
            "data:propulsion:electric_systems:fuel_cell:polarization_curve:V", shape=771
        )
        self.add_input(
            "data:propulsion:electric_systems:fuel_cell:polarization_curve:i", shape=771
        )
        self.add_input(
            "data:propulsion:electric_systems:fuel_cell:polarization_curve:efficiency",
            shape=771,
        )
        self.add_input("data:propulsion:electric_systems:fuel_cell:layout:stacks", 1)
        self.add_input("data:propulsion:electric_systems:fuel_cell:layout:cells", 1.0)

        self.add_input("data:mission:sizing:takeoff:EM_power_rate", 0.0)
        self.add_input("data:mission:sizing:initial_climb:EM_power_rate", 0.0)
        self.add_input(
            "data:mission:sizing:climb:EM_power_rate", 0.0, shape=POWER_RATE_COUNT
        )
        self.add_input("data:mission:sizing:cruise:EM_power_rate", 0.0)

        self.add_input("data:mission:sizing:main_route:cruise:altitude", units="ft")
        self.add_input("data:mission:sizing:climb:operational_ceiling", units="m")

        self.add_input("data:mission:sizing:climb:speed", np.nan, units="m/s")
        self.add_input("data:TLAR:cruise_mach", np.nan)
        self.add_input("data:mission:sizing:cruise:max_mach", np.nan)

        self.add_input(
            "data:propulsion:electric_systems:battery:specific_power",
            val=np.nan,
            units="W/kg",
        )
        self.add_input(
            "data:propulsion:electric_systems:battery:specific_energy",
            val=np.nan,
            units="W*h/kg",
        )
        self.add_input(
            "data:propulsion:electric_systems:battery:battery_SOCmin", val=np.nan
        )
        self.add_input(
            "data:propulsion:electric_systems:battery:voltage", val=800, units="V"
        )
        self.add_input("data:propulsion:electric_systems:battery:k_int", val=1.35)

        self.add_input(
            "data:propulsion:electric_systems:cooling:DISA", val=0.0, units="degC"
        )
        self.add_input(
            "data:propulsion:electric_systems:cooling:T_cool", val=75.0, units="degC"
        )

        self.add_input(
            "data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:efficiency_cell",
            val=1,
        )
        self.add_input("data:propulsion:electric_systems:motor:motor_eta", val=np.nan)
        self.add_input(
            "data:propulsion:electric_systems:power_electronics:power_electronics_eta",
            val=np.nan,
        )

        self.add_input("data:mission:sizing:H2", val=0, units="kg")
        self.add_input("data:mission:sizing:BAT_energy", val=0, units="J")
        self.add_input(
            "data:propulsion:electric_systems:fuel_cell:Power_Offtake", 0, units="W"
        )
        self.add_input(
            "data:propulsion:electric_systems:fuel_cell:Gross_net_power_ratio", 0.89
        )
        # self.add_input("data:weight:propulsion:electric_systems:battery:mass", val=np.nan, units="kg")

        ########## inputs uncertainty ############

        self.add_input("tuning:propulsion:electric_systems:cooling:k_mass", 1.0)

        ######################

        self.add_output(
            "data:weight:propulsion:electric_systems:fuel_cell:mass", units="kg"
        )
        self.add_output(
            "data:weight:propulsion:electric_systems:battery:mass", units="kg"
        )
        self.add_output(
            "data:weight:propulsion:electric_systems:power_electronics:mass", units="kg"
        )
        self.add_output(
            "data:weight:propulsion:electric_systems:motor:mass", units="kg"
        )
        self.add_output(
            "data:weight:propulsion:electric_systems:H2_storage:mass", units="kg"
        )
        self.add_output(
            "data:weight:propulsion:electric_systems:cooling:mass", units="kg"
        )
        self.add_output(
            "data:weight:propulsion:electric_systems:H2_distribution:mass", units="kg"
        )

        self.add_output(
            "data:weight:propulsion:electric_systems:cables:mass", units="kg"
        )

        self.add_output(
            "data:propulsion:electric_systems:cooling:radiators:total_frontal_area",
            units="m**2",
        )

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        P_nom = inputs["data:propulsion:electric_systems:P_nom"]

        P_sizing_fc = inputs[
            "data:propulsion:electric_systems:fuel_cell:sizing:sizing_point:P_sizing"
        ]
        n_motors = inputs["data:geometry:propulsion:motor:count"]
        motor_eta = inputs["data:propulsion:electric_systems:motor:motor_eta"]
        power_elec_eta = inputs[
            "data:propulsion:electric_systems:power_electronics:power_electronics_eta"
        ]
        fc_stacks_eta = inputs[
            "data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:efficiency_cell"
        ]
        # H2_cooling_capacity = 0.13 #13%

        cables_length = inputs["data:propulsion:electric_systems:cables:length"]
        cables_material = inputs["data:propulsion:electric_systems:cables:material"]
        pipes_length = inputs[
            "data:propulsion:electric_systems:H2_distribution:pipes_length"
        ]

        H2_pipes_m_ratio = inputs[
            "data:propulsion:electric_systems:H2_distribution:mass_ratio"
        ]
        n_stacks = inputs[
            "data:propulsion:electric_systems:fuel_cell:layout:stacks"
        ]  # per pack
        n_packs = inputs["data:propulsion:electric_systems:fuel_cell:layout:packs"]
        sizing_current = inputs[
            "data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:i_pack"
        ]

        Power_Offtake = inputs[
            "data:propulsion:electric_systems:fuel_cell:Power_Offtake"
        ]
        Gross_net_power_ratio = inputs[
            "data:propulsion:electric_systems:fuel_cell:Gross_net_power_ratio"
        ]

        Psp_bat = inputs["data:propulsion:electric_systems:battery:specific_power"]
        Esp_bat = inputs["data:propulsion:electric_systems:battery:specific_energy"]
        SOCmin = inputs["data:propulsion:electric_systems:battery:battery_SOCmin"]
        k_int = inputs["data:propulsion:electric_systems:battery:k_int"]

        Psp_fc = (
            inputs["data:propulsion:electric_systems:fuel_cell:specific_power"] * 1000
        )  # W/kg
        Psp_mot = inputs["data:propulsion:electric_systems:motor:specific_power"]
        Psp_electronics = inputs[
            "data:propulsion:electric_systems:power_electronics:specific_power"
        ]
        eta_g = inputs["data:propulsion:electric_systems:H2_storage:efficiency"]
        m_H2 = inputs["data:mission:sizing:H2"]
        energy_bat = inputs["data:mission:sizing:BAT_energy"]
        bat_voltage = inputs["data:propulsion:electric_systems:battery:voltage"]
        ########## inputs uncertainty ############

        k_m_cool = inputs["tuning:propulsion:electric_systems:cooling:k_mass"]

        ######################

        P_tot = P_nom * n_motors
        P_tot_power_elec = (
            P_nom / motor_eta * n_motors
        )  # specific power alrady akes into account he gross/net ratio
        P_tot_source = P_nom / motor_eta / power_elec_eta * n_motors

        # P_tot_fc=0

        # cooling mass
        if P_nom != 0:
            if m_H2 == 0:
                print("PHEB")
                sizing_current = P_tot_source / n_motors / bat_voltage
                if cables_material == 1:
                    df = pd.read_csv(
                        os.path.join(
                            rhea_path,
                            "resources/hybrid_systems/cables/data/m_cable_vs_Amp.csv",
                        )
                    )
                elif cables_material == 2:
                    df = pd.read_csv(
                        os.path.join(
                            rhea_path,
                            "resources/hybrid_systems/cables/data/m_cable_vs_Amp_Alu.csv",
                        )
                    )
                cables_m_ratio = np.interp(sizing_current, df.Ampere, df.m_ratio)
                m_cable = (
                    cables_m_ratio * cables_length
                )  # total length=30m  #15m per side
                m_cooling = 0.0
                frontal_area = 0.0
            else:

                P_stacks = (
                    P_tot_source / n_packs / Gross_net_power_ratio + Power_Offtake
                )  # fc net power one side

                # altitudes,speeds,radiator_sizing = self.evaluate_cooling_requirement(inputs,P_stacks)
                df = self.evaluate_cooling_requirement(inputs, P_stacks)
                df.to_csv(self.out_file)

                m_cooling = df.weight.max() * n_motors
                frontal_area = df.Area.max() * n_motors
                sizing_conditions = df.iloc[df.weight.idxmax()]
                if df.Area.max() != sizing_conditions.Area:
                    print(
                        "Warning: different sizing point for radiator frontal area and weight"
                    )
                # cable mass
                if cables_material == 1:
                    df = pd.read_csv(
                        os.path.join(
                            rhea_path,
                            "resources/hybrid_systems/cables/data/m_cable_vs_Amp.csv",
                        )
                    )
                elif cables_material == 2:
                    df = pd.read_csv(
                        os.path.join(
                            rhea_path,
                            "resources/hybrid_systems/cables/data/m_cable_vs_Amp_Alu.csv",
                        )
                    )
                cables_m_ratio = np.interp(sizing_current, df.Ampere, df.m_ratio)
                m_cable = (
                    cables_m_ratio * cables_length
                )  # total length=30m  #15m per side
        else:
            m_cable = 0.0
            m_cooling = 0.0
            frontal_area = 0.0

        if m_H2 != 0:
            m_distr = (
                H2_pipes_m_ratio * pipes_length + 7 * n_stacks * n_packs
            )  # 50 meters of pipes in total;
        else:
            m_distr = 0.0

        outputs["data:weight:propulsion:electric_systems:fuel_cell:mass"] = (
            P_sizing_fc / Psp_fc
        )
        outputs["data:weight:propulsion:electric_systems:motor:mass"] = P_tot / Psp_mot
        outputs["data:weight:propulsion:electric_systems:power_electronics:mass"] = (
            P_tot_power_elec / Psp_electronics
        )
        outputs["data:weight:propulsion:electric_systems:H2_storage:mass"] = (
            m_H2 - eta_g * m_H2
        ) / eta_g
        outputs["data:weight:propulsion:electric_systems:battery:mass"] = (
            energy_bat / constants.hour / Esp_bat / (1.0 - SOCmin) * k_int
        )  # , P_tot/Psp_bat*k_int)
        outputs["data:weight:propulsion:electric_systems:cooling:mass"] = (
            m_cooling * k_m_cool
        )
        outputs[
            "data:weight:propulsion:electric_systems:H2_distribution:mass"
        ] = m_distr
        outputs["data:weight:propulsion:electric_systems:cables:mass"] = m_cable
        outputs[
            "data:propulsion:electric_systems:cooling:radiators:total_frontal_area"
        ] = frontal_area

    def radiator_sizing(self, Q_req, Altitude, Mach, Disa, T_load, N):

        filename = os.path.join(
            rhea_path,
            "resources/hybrid_systems/thermal_management/Metamodels/Radiator_weight_model.sav",
        )
        weight_model = joblib.load(open(filename, "rb"))
        filename = os.path.join(
            rhea_path,
            "resources/hybrid_systems/thermal_management/Metamodels/Radiator_area_model.sav",
        )
        area_model = joblib.load(open(filename, "rb"))

        # Thermo/aero parameters

        T_inf = Atmosphere(Altitude, delta_t=Disa, altitude_in_feet=False).temperature
        T_tot = T_inf * (1 + 0.2 * Mach * Mach)  # 0.2=(gamma-1)/2
        delta_T = T_load - T_tot  # [K]

        # Weight and frontal area predictions
        unit_weight = weight_model.predict(
            np.array(Q_req / 1000 / N / delta_T).reshape(-1, 1)
        )
        total_weight = N * unit_weight
        unit_area = area_model.predict(
            np.array(Q_req / 1000 / N / delta_T).reshape(-1, 1)
        )
        total_area = N * unit_area
        if Q_req == 0:
            unit_weight = [0.0]
            total_weight = [0]
            unit_area = [0]
            total_area = [0]

        return unit_weight[0], total_weight[0], unit_area[0], total_area[0]

    def evaluate_cooling_requirement(self, inputs, P_stacks):
        V_pol_fc = inputs[
            "data:propulsion:electric_systems:fuel_cell:polarization_curve:V"
        ]
        I_pol_fc = inputs[
            "data:propulsion:electric_systems:fuel_cell:polarization_curve:i"
        ]
        eta_pol_fc = inputs[
            "data:propulsion:electric_systems:fuel_cell:polarization_curve:efficiency"
        ]
        target_altitude = (
            inputs["data:mission:sizing:main_route:cruise:altitude"] * constants.foot
        )
        ceiling_altitude = inputs["data:mission:sizing:climb:operational_ceiling"]

        cruise_altitude = min(target_altitude, ceiling_altitude)
        climb_altitudes = np.linspace(
            1500.0 * constants.foot, float(target_altitude), 7
        )

        if cruise_altitude == ceiling_altitude:
            climb_altitudes[-1] = ceiling_altitude

        A_cell = inputs["data:propulsion:electric_systems:fuel_cell:sizing:A_cell"]
        n_cells = inputs["data:propulsion:electric_systems:fuel_cell:layout:cells"]
        n_stacks = inputs["data:propulsion:electric_systems:fuel_cell:layout:stacks"]
        n_packs = inputs["data:propulsion:electric_systems:fuel_cell:layout:packs"]
        n_motors = inputs["data:geometry:propulsion:motor:count"]
        T_fc = inputs["data:propulsion:electric_systems:fuel_cell:sizing:T_op"]

        EAS_climb = inputs["data:mission:sizing:climb:speed"]
        target_cruise_mach = inputs["data:TLAR:cruise_mach"]
        max_cruise_mach = inputs["data:mission:sizing:cruise:max_mach"]
        if max_cruise_mach > target_cruise_mach or math.isnan(max_cruise_mach):
            cruise_mach = target_cruise_mach
        else:
            cruise_mach = max_cruise_mach

        fuel_cell = Fuel_cell(
            V_pol_fc, I_pol_fc, n_stacks, n_packs, A_cell, n_cells, n_motors
        )

        phases = [
            FlightPhase.TAKEOFF,
            FlightPhase.INITIAL_CLIMB,
            FlightPhase.CLIMB,
            FlightPhase.CRUISE,
        ]

        electric_power_rates = {
            FlightPhase.TAKEOFF: inputs["data:mission:sizing:takeoff:EM_power_rate"],
            FlightPhase.INITIAL_CLIMB: inputs[
                "data:mission:sizing:initial_climb:EM_power_rate"
            ],
            FlightPhase.CLIMB: inputs["data:mission:sizing:climb:EM_power_rate"],
            FlightPhase.CRUISE: inputs["data:mission:sizing:cruise:EM_power_rate"],
        }

        altitudes = {
            FlightPhase.TAKEOFF: np.array([0.0]),
            FlightPhase.INITIAL_CLIMB: np.array([0.0]),
            # FlightPhase.CLIMB: np.linspace(1500.*constants.foot, float(cruise_altitude) ,8),
            FlightPhase.CLIMB: climb_altitudes,
            FlightPhase.CRUISE: cruise_altitude,
        }

        speeds = {
            FlightPhase.TAKEOFF: np.array([0.15]),
            FlightPhase.INITIAL_CLIMB: np.array([0.2]),
            FlightPhase.CLIMB: np.array(
                [
                    Atmosphere(alt / constants.foot).get_true_airspeed(EAS_climb)
                    / Atmosphere(alt / constants.foot).speed_of_sound
                    for alt in altitudes[FlightPhase.CLIMB]
                ]
            ),  # np.array([0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25]),
            FlightPhase.CRUISE: cruise_mach,  # np.array([0.45]),
        }

        Disa = inputs["data:propulsion:electric_systems:cooling:DISA"]
        T_load = (
            273.15 + inputs["data:propulsion:electric_systems:cooling:T_cool"]
        )  # Kelvin

        N = 1  # number of radiators [-] for AIAA STUDY was 10
        weight_list = []
        area_list = []
        Q_req_list = []
        P_out_list = []
        eta_list = []
        H2_flow_list = []  # one side
        air_flow_list = []  # both sides

        H2_flow_climb = []
        eta_climb = []
        air_flow_climb = []

        for phase in phases:
            P_out = P_stacks * electric_power_rates[phase]
            if phase.name == "CLIMB":
                for P in P_out:
                    FC_H2, eta_fc, air_flow_tot = fuel_cell.get_fc_perfo(P)[:3]

                    H2_flow_climb.append(FC_H2)
                    eta_climb.append(eta_fc)
                    air_flow_climb.append(air_flow_tot)
                eta_list.append(eta_climb)
                H2_flow_list.append(H2_flow_climb)
                air_flow_list.append(air_flow_climb)
                Q_H2_heating = self.evaluate_H2_cooling_capacity(
                    T_fc,
                    np.array(
                        [float(val) for sublist in H2_flow_climb for val in sublist]
                    ),
                )  # heat used to heat hydrogen
                eta_fc = np.array(
                    [float(val) for sublist in eta_climb for val in sublist]
                )
            else:
                FC_H2, eta_fc, air_flow_tot = fuel_cell.get_fc_perfo(P_out)[:3]
                Q_H2_heating = self.evaluate_H2_cooling_capacity(
                    T_fc, FC_H2
                )  # heat used to heat hydrogen

                eta_list.append(eta_fc)
                H2_flow_list.append(FC_H2)
                air_flow_list.append(air_flow_tot)

            Excess_H2 = 0.05 * P_out / eta_fc
            Q_tot = P_out / eta_fc - Excess_H2 - P_out
            Q_tot_cool = Q_tot * 35 / 45  # heat to be rejected by cooling system
            # Q_H2_heating = H2_cooling_capacity*Q_tot_cool #heat used to heat hydrogen
            Q_req = Q_tot_cool - Q_H2_heating  # Remaining heat to be rejected [W]

            if phase.name == "CLIMB":

                for idx in range(len(electric_power_rates[phase])):
                    (
                        unit_weight,
                        total_weight,
                        unit_area,
                        total_area,
                    ) = self.radiator_sizing(
                        Q_req[idx],
                        altitudes[phase][idx],
                        speeds[phase][idx],
                        Disa,
                        T_load,
                        N,
                    )
                    weight_list.append(total_weight)
                    area_list.append(total_area)
                    Q_req_list.append(Q_req[idx])
                    P_out_list.append(P_out[idx])

            else:
                unit_weight, total_weight, unit_area, total_area = self.radiator_sizing(
                    Q_req, altitudes[phase], speeds[phase], Disa, T_load, N
                )
                Q_req_list.append(float(Q_req))
                P_out_list.append(float(P_out))
                weight_list.append(total_weight)
                area_list.append(total_area)

        df = pd.DataFrame(
            columns=[
                "phase",
                "altitude",
                "mach",
                "P_out",
                "power_rate",
                "fc_eta",
                "H2_flow",
                "fc_air_flow",
                "Q_req",
                "weight",
                "Area",
            ]
        )
        df["phase"] = [
            FlightPhase.TAKEOFF,
            FlightPhase.INITIAL_CLIMB,
            FlightPhase.CLIMB,
            FlightPhase.CLIMB,
            FlightPhase.CLIMB,
            FlightPhase.CLIMB,
            FlightPhase.CLIMB,
            FlightPhase.CLIMB,
            FlightPhase.CLIMB,
            FlightPhase.CRUISE,
        ]
        df["altitude"] = [val for sublist in altitudes.values() for val in sublist]
        df["mach"] = [val for sublist in speeds.values() for val in sublist]
        df["P_out"] = P_out_list
        df["power_rate"] = [
            val for sublist in electric_power_rates.values() for val in sublist
        ]
        df["fc_eta"] = [float(val) for sublist in eta_list for val in sublist]
        df["H2_flow"] = [float(val) for sublist in H2_flow_list for val in sublist]
        df["fc_air_flow"] = [float(val) for sublist in air_flow_list for val in sublist]
        df["Q_req"] = Q_req_list
        df["weight"] = weight_list
        df["Area"] = area_list
        return df

    def evaluate_H2_cooling_capacity(self, T_fc, H2_flow):

        # Constant parameters
        L_H2 = 455  # Latent heat of vaporization [kJ/kg]
        cp_H2 = 14.3  # Specific heat capacity [kJ/kg/K]
        T_cold = 22  # temperature of hydrogen leaving the tank [K]
        T_hot = T_fc  # temperature of hydrogen entering the fuel cells [K]

        # Cooling capacity
        Q_H2 = H2_flow * L_H2 + H2_flow * cp_H2 * (T_hot - T_cold)  # [kW]

        return 1000 * Q_H2  # [W]
