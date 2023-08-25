"""
Thermodynamic sizing of turboshaft
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

import numpy as np
from ..engine_components.Ram import Ram
from ..engine_components.Compression_Nozzle import Compression_Nozzle
from ..engine_components.Low_Pressure_Compressor import LPC
from ..engine_components.High_Pressure_Compressor import HPC
from ..engine_components.Combustor import Combustor
from ..engine_components.Low_Pressure_Turbine import LPT
from ..engine_components.High_Pressure_Turbine import HPT
from ..engine_components.Power_Turbine import Power_Turbine
from ..engine_components.Expansion_Nozzle import Expansion_Nozzle
from ..engine_components.Thrust import Thrust
from ..engine_components.Fuel_Data import Fuel_data
from fastoad.model_base.atmosphere import Atmosphere
from openmdao.core.explicitcomponent import ExplicitComponent
from fastoad.module_management.constants import ModelDomain
from fastoad.module_management.service_registry import RegisterOpenMDAOSystem


@RegisterOpenMDAOSystem("rhea.propulsion.turboprop_sizing", domain=ModelDomain.PROPULSION)
class TP_sizing(ExplicitComponent):
    """
    Performs sizing of the turboprop based on input thermodynamic and mechanical power. Determines off-design parameters.
    """

    def setup(self):
        self.add_input("data:propulsion:RTO_power", np.nan, units="W")
        self.add_input("data:propulsion:Design_Thermo_Power", np.nan, units="W")
        self.add_input("data:propulsion:Power_Offtake", np.nan, units="W")
        self.add_input("data:propulsion:gearbox_eta", np.nan)
        self.add_input("data:propulsion:L1_engine:fuel", np.nan)
        self.add_input("data:propulsion:L1_engine:turbine_inlet_temperature", np.nan, units="K")

        self.add_input("data:propulsion:L1_engine:inlet:inlet_eta_pol", np.nan)
        self.add_input("data:propulsion:L1_engine:inlet:inlet_pressure_ratio", np.nan)
        self.add_input("data:propulsion:L1_engine:lpc:lpc_eta_pol", np.nan)
        self.add_input("data:propulsion:L1_engine:lpc:lpc_pressure_ratio", np.nan)
        self.add_input("data:propulsion:L1_engine:hpc:hpc_eta_pol", np.nan)
        self.add_input("data:propulsion:L1_engine:hpc:hpc_pressure_ratio", np.nan)
        self.add_input("data:propulsion:L1_engine:combustor:combustor_eta", np.nan)
        self.add_input("data:propulsion:L1_engine:combustor:combustor_pressure_ratio", np.nan)
        self.add_input("data:propulsion:L1_engine:hpt:hpt_eta_pol", np.nan)
        self.add_input("data:propulsion:L1_engine:hpt:hpt_eta_mech", np.nan)
        self.add_input("data:propulsion:L1_engine:lpt:lpt_eta_pol", np.nan)
        self.add_input("data:propulsion:L1_engine:lpt:lpt_eta_mech", np.nan)
        self.add_input("data:propulsion:L1_engine:pt:pt_eta_pol", np.nan)
        self.add_input("data:propulsion:L1_engine:pt:pt_eta_mech", np.nan)
        self.add_input("data:propulsion:L1_engine:nozzle:nozzle_eta_pol", np.nan)
        self.add_input("data:propulsion:L1_engine:nozzle:nozzle_pressure_ratio", np.nan)
        self.add_input("data:propulsion:L1_engine:nozzle:nozzle_area_ratio", np.nan)

        self.add_output("data:propulsion:L1_engine:sizing:k0")
        self.add_output("data:propulsion:L1_engine:sizing:k1")
        self.add_output("data:propulsion:L1_engine:sizing:k2")
        self.add_output("data:propulsion:L1_engine:sizing:tau_t_sizing")
        self.add_output("data:propulsion:L1_engine:sizing:pi_t_sizing")
        self.add_output("data:propulsion:L1_engine:sizing:M_out_sizing")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        Design_Thermo_Power = inputs["data:propulsion:Design_Thermo_Power"]
        inlet_pressure_ratio = inputs["data:propulsion:L1_engine:inlet:inlet_pressure_ratio"]
        combustor_pressure_ratio = inputs["data:propulsion:L1_engine:combustor:combustor_pressure_ratio"]
        T4 = inputs["data:propulsion:L1_engine:turbine_inlet_temperature"]
        # temperature, density, pressure, visc, a = atmosphere(0., 0)
        altitude = 0.
        atmosphere = Atmosphere(altitude, altitude_in_feet=False)

        shaft_power_adim, fuel_to_air_ratio, specific_energy, jet_power_adim, residual_thrust_nd, tau_ram, tau_lpc, tau_hpc, tau_b, pi_pt, tau_pt, pi_hpc, pi_lpc, pi_ram, pi_hpt, pi_lpt, tau_t, M_out_sizing = self.compute_static_power_adim(
            inputs)

        air_flow = (Design_Thermo_Power) / (shaft_power_adim)
        total_flow = air_flow * (1 + fuel_to_air_ratio)
        fuel_flow = fuel_to_air_ratio * air_flow

        # calculation of gas turbine dimensions
        D_gt = 0.25 * (Design_Thermo_Power) ** 0.12
        A_gt = np.pi * (D_gt / 2) ** 2
        # L_gt = 0.12*(design_TO_Shaft_Power/1000 + 22400./1000)**0.373

        # calculation of off-design constants
        # M_out_sizing= self.nozzle.M_out_sizing  
        pi_t_sizing = pi_pt / (pi_hpt * pi_lpt)
        tau_t_sizing = tau_t
        k1 = (tau_hpc - 1) * (tau_lpc * tau_ram) / tau_b
        k2 = (tau_lpc - 1) * tau_ram / tau_b
        k0 = air_flow * T4 ** 0.5 / (
                    atmosphere.pressure * pi_ram * pi_lpc * pi_hpc * inlet_pressure_ratio * combustor_pressure_ratio)

        #         # calculation of thermal efficiency
        thermal_power = fuel_flow * specific_energy
        ESHP = shaft_power_adim * air_flow + jet_power_adim * air_flow
        jet_thrust = residual_thrust_nd * air_flow
        thermal_efficiency = (shaft_power_adim * air_flow + jet_power_adim * air_flow) / thermal_power

        outputs["data:propulsion:L1_engine:sizing:k0"] = k0
        outputs["data:propulsion:L1_engine:sizing:k1"] = k1
        outputs["data:propulsion:L1_engine:sizing:k2"] = k2
        outputs["data:propulsion:L1_engine:sizing:tau_t_sizing"] = tau_t_sizing
        outputs["data:propulsion:L1_engine:sizing:pi_t_sizing"] = pi_t_sizing
        outputs["data:propulsion:L1_engine:sizing:M_out_sizing"] = M_out_sizing

    def compute_static_power_adim(self, inputs):
        """
        Computation of maximum available power.

        Uses model described in 

        :param atmosphere: Atmosphere instance at intended altitude (should be <=20km)
        :param mach: Mach number(s) (should be between 0.05 and 1.0)
        :param phase: flight phase which influences engine rating (max mechanical power)
        :param delta_t4: (unit=K) difference between T4 at flight point and design T4
        
        :return: maximum power (in W)
        """
        '''    TAXI_IN = 0
                TAKEOFF = 1
                CLIMB = 2
                CRUISE = 3
                DESCENT = 5
                LANDING = 6
                TAXI_OUT = 7'''

        Power_Offtake = inputs["data:propulsion:Power_Offtake"]
        fuel_type = inputs["data:propulsion:L1_engine:fuel"]

        T4 = inputs["data:propulsion:L1_engine:turbine_inlet_temperature"]
        # HP_bleed= inputs["data:propulsion:L1_engine:HP_bleed"]
        # LP_bleed= inputs["data:propulsion:L1_engine:LP_bleed"]

        inlet_eta_pol = inputs["data:propulsion:L1_engine:inlet:inlet_eta_pol"]
        inlet_pressure_ratio = inputs["data:propulsion:L1_engine:inlet:inlet_pressure_ratio"]
        lpc_eta_pol = inputs["data:propulsion:L1_engine:lpc:lpc_eta_pol"]
        lpc_pressure_ratio = inputs["data:propulsion:L1_engine:lpc:lpc_pressure_ratio"]

        hpc_eta_pol = inputs["data:propulsion:L1_engine:hpc:hpc_eta_pol"]
        hpc_pressure_ratio = inputs["data:propulsion:L1_engine:hpc:hpc_pressure_ratio"]

        combustor_eta = inputs["data:propulsion:L1_engine:combustor:combustor_eta"]
        combustor_pressure_ratio = inputs["data:propulsion:L1_engine:combustor:combustor_pressure_ratio"]

        hpt_eta_pol = inputs["data:propulsion:L1_engine:hpt:hpt_eta_pol"]
        hpt_eta_mech = inputs["data:propulsion:L1_engine:hpt:hpt_eta_mech"]

        lpt_eta_pol = inputs["data:propulsion:L1_engine:lpt:lpt_eta_pol"]
        lpt_eta_mech = inputs["data:propulsion:L1_engine:lpt:lpt_eta_mech"]

        pt_eta_pol = inputs["data:propulsion:L1_engine:pt:pt_eta_pol"]
        pt_eta_mech = inputs["data:propulsion:L1_engine:pt:pt_eta_mech"]

        nozzle_eta_pol = inputs["data:propulsion:L1_engine:nozzle:nozzle_eta_pol"]
        nozzle_pressure_ratio = inputs["data:propulsion:L1_engine:nozzle:nozzle_pressure_ratio"]
        nozzle_area_ratio = inputs["data:propulsion:L1_engine:nozzle:nozzle_area_ratio"]

        altitude = 0.  # atmosphere.get_altitude(altitude_in_feet=False)
        mach = 0.01  # np.asarray(mach)
        atmosphere = Atmosphere(altitude, altitude_in_feet=False)

        ram = Ram()
        ram.compute(atmosphere, mach)
        #        print 'ram', ram.stagnation_temperature,ram.stagnation_pressure

        # link inlet nozzle to ram
        inlet = Compression_Nozzle()
        inlet.stagnation_temperature_in = ram.stagnation_temperature
        inlet.stagnation_pressure_in = ram.stagnation_pressure

        # Flow through the inlet nozzle
        inlet.compute(atmosphere, inlet_eta_pol, inlet_pressure_ratio)
        #        print 'inlet',inlet.stagnation_temperature_out,inlet.stagnation_pressure_out,inlet.stagnation_enthalpy_out

        # link low pressure compressor to the inlet nozzle
        lpc = LPC()
        lpc.stagnation_temperature_in = inlet.stagnation_temperature_out
        lpc.stagnation_pressure_in = inlet.stagnation_pressure_out
        # Flow through the low pressure compressor
        lpc.compute_design(lpc_pressure_ratio, lpc_eta_pol)
        #        print 'lpc', lpc.stagnation_temperature_out,lpc.stagnation_pressure_out,lpc.stagnation_enthalpy_out

        # link the high pressure compressor to the low pressure compressor
        hpc = HPC()
        hpc.stagnation_temperature_in = lpc.stagnation_temperature_out
        hpc.stagnation_pressure_in = lpc.stagnation_pressure_out
        # Flow through the high pressure compressor
        hpc.compute_design(hpc_pressure_ratio, hpc_eta_pol)
        #        print 'hpc', hpc.stagnation_temperature_out,hpc.stagnation_pressure_out,hpc.stagnation_enthalpy_out

        # configure chosen fuel
        fuel = Fuel_data()
        fuel.configure(fuel_type)

        # link the combustor to the high pressure compressor
        combustor = Combustor()
        combustor.stagnation_temperature_in = hpc.stagnation_temperature_out
        combustor.stagnation_pressure_in = hpc.stagnation_pressure_out
        combustor.htf = fuel.specific_energy
        # combustor.TIT                                      = T4
        # flow through the high pressor comprresor
        combustor.compute_design(T4, combustor_pressure_ratio, combustor_eta, atmosphere.temperature)
        #        print 'combustor', self.TP['T4'],combustor.stagnation_pressure_out,combustor.stagnation_enthalpy_out,combustor.fuel_to_air_ratio

        # link the high pressure turbine to the combustor
        hpt = HPT()
        hpt.stagnation_temperature_in = T4
        hpt.stagnation_pressure_in = combustor.stagnation_pressure_out
        hpt.fuel_to_air_ratio = combustor.fuel_to_air_ratio
        # link the high pressure turbine to the high pressure compressor
        hpt.compressor_work = hpc.work_done
        hpt.bleed_offtake = 1.

        # flow through the high pressure turbine
        hpt.compute(Power_Offtake, hpt_eta_mech, hpt_eta_pol)
        #        print 'hpt',hpt.stagnation_pressure_in/hpt.stagnation_pressure_out #hpt.stagnation_temperature_out,hpt.stagnation_pressure_out,hpt.stagnation_enthalpy_out

        # link the low pressure turbine to the high pressure turbine
        lpt = LPT()
        lpt.stagnation_temperature_in = hpt.stagnation_temperature_out
        lpt.stagnation_pressure_in = hpt.stagnation_pressure_out
        lpt.fuel_to_air_ratio = combustor.fuel_to_air_ratio
        lpt.bleed_offtake = 1.

        # link the low pressure turbine to the low_pressure_compresor
        lpt.compressor_work = lpc.work_done

        # flow through the low pressure turbine
        lpt.compute(lpt_eta_pol, lpt_eta_mech)
        #        print 'lpt', lpt.stagnation_pressure_in/lpt.stagnation_pressure_out,lpt.stagnation_temperature_out,lpt.stagnation_pressure_out,lpt.stagnation_enthalpy_out
        # print('ITT', lpt.stagnation_temperature_out)

        # link low-pressure turbine output to thrust component
        thrust = Thrust()
        thrust.fuel_to_air_ratio = combustor.fuel_to_air_ratio
        thrust.stagnation_temperature_in = lpt.stagnation_temperature_out
        thrust.stagnation_pressure_in = lpt.stagnation_pressure_out

        # compute the thrust

        thrust.compute_design(atmosphere, mach, lpt.bleed_offtake * hpt.bleed_offtake, pt_eta_mech, pt_eta_pol,
                              nozzle_pressure_ratio, nozzle_eta_pol)

        # link the power turbine to the low pressure turbine
        # link the power turbine to the low pressure turbine
        pt = Power_Turbine()
        pt.stagnation_enthalpy_out = thrust.power_turbine_ht_out
        pt.stagnation_pressure_out = thrust.pow_turb_Pt_out
        pt.stagnation_temperature_out = thrust.pow_turb_Tt_out
        pt.stagnation_temperature_in = lpt.stagnation_temperature_out
        pt.stagnation_pressure_in = lpt.stagnation_pressure_out
        pt.fuel_to_air_ratio = lpt.fuel_to_air_ratio
        pt.nozzle_u_out = thrust.nozzle_u_out
        pt.bleed_offtake = 1.

        # flow through the power turbine
        pt.compute_design(pt_eta_mech, nozzle_area_ratio)
        #        print 'pt',pt.stagnation_pressure_in/pt.stagnation_pressure_out ,pt.stagnation_temperature_out,pt.stagnation_pressure_out,pt.stagnation_enthalpy_out

        # link the expansion nozzle to the power turbine
        nozzle = Expansion_Nozzle()
        nozzle.stagnation_enthalpy_in = pt.stagnation_enthalpy_out
        nozzle.stagnation_temperature_in = pt.stagnation_temperature_out
        nozzle.stagnation_temperature_out = thrust.nozzle_Tt_out
        nozzle.stagnation_pressure_in = pt.stagnation_pressure_out
        nozzle.u_out = thrust.nozzle_u_out
        nozzle.M_out = thrust.M_out_nozzle
        nozzle.static_temperature_out = thrust.nozzle_T_out
        nozzle.residual_thrust_nd = thrust.residual_thrust_nd
        # flow through theexpansion nozzle
        nozzle.compute_design(atmosphere, nozzle_pressure_ratio)
        #        print 'nozzle', nozzle.stagnation_temperature_out,nozzle.stagnation_pressure_out, nozzle.M_out

        shaft_power_adim = pt.shaft_takeoff  # dimensionless shaft horsepower
        fuel_to_air_ratio = combustor.fuel_to_air_ratio
        specific_energy = fuel.specific_energy
        jet_power_adim = nozzle.jet_power
        residual_thrust_nd = thrust.residual_thrust_nd
        tau_ram = ram.temperature_ratio
        tau_lpc = lpc.temperature_ratio
        tau_hpc = hpc.temperature_ratio
        tau_b = combustor.tau_b
        pi_pt = pt.pressure_ratio
        tau_pt = pt.temperature_ratio
        pi_hpc = hpc.pressure_ratio
        pi_lpc = lpc.pressure_ratio
        pi_ram = ram.pressure_ratio
        pi_hpt = hpt.stagnation_pressure_in / hpt.stagnation_pressure_out
        pi_lpt = lpt.stagnation_pressure_in / lpt.stagnation_pressure_out
        tau_t = pt.stagnation_temperature_out / hpt.stagnation_temperature_in
        M_out_sizing = nozzle.M_out_sizing
        return shaft_power_adim, fuel_to_air_ratio, specific_energy, jet_power_adim, residual_thrust_nd, tau_ram, tau_lpc, tau_hpc, tau_b, pi_pt, tau_pt, pi_hpc, pi_lpc, pi_ram, pi_hpt, pi_lpt, tau_t, M_out_sizing
