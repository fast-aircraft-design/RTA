import numpy as np
import scipy as sp


class HPT(object):
    def compute(self, shaft_takeoff, eta_mech, etapolt):

        gamma = 1.321
        cp_t = gamma * 287.87 / (gamma - 1)
        # unpack from inputs
        Tt_in = self.stagnation_temperature_in
        Pt_in = self.stagnation_pressure_in

        f = self.fuel_to_air_ratio
        compressor_work = self.compressor_work
        bleed_offtake = self.bleed_offtake

        #        if self.inputs.shaft_power_off_take is not None:
        #            shaft_takeoff = self.inputs.shaft_power_off_take.work_done
        #        else:
        #            shaft_takeoff = 0.
        # shaft_takeoff = turboprop['Power_Offtake']

        # unpack from TP
        # eta_mech        =  turboprop['hpt_eta_mech']
        # etapolt         =  turboprop['hpt_eta_pol']
        # method to compute turbine properties

        # Using the work done by the compressors/fan and the fuel to air ratio to compute the energy drop across the turbine
        deltah_ht = (
            -1
            / ((1 + f) * bleed_offtake)
            * 1
            / eta_mech
            * (compressor_work + shaft_takeoff)
        )

        # Compute the output stagnation quantities from the inputs and the energy drop computed above
        #        cp_t =1120.
        ht_in = cp_t * Tt_in
        Tt_out = Tt_in + deltah_ht / cp_t
        Pt_out = Pt_in * (Tt_out / Tt_in) ** (gamma / ((gamma - 1) * etapolt))
        ht_out = cp_t * Tt_out  # h(Tt4_5)

        # pack the computed values into outputs
        self.stagnation_temperature_out = Tt_out
        self.stagnation_pressure_out = Pt_out
        self.stagnation_enthalpy_in = ht_in
        self.stagnation_enthalpy_out = ht_out
