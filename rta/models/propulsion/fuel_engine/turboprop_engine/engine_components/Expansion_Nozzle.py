import numpy as np


class Expansion_Nozzle(object):
    def compute_design(self, atmosphere, pid):

        gamma = 1.321
        Cp = gamma * 287.87 / (gamma - 1)
        #        Cp                     = 1.4*287.87/(1.4-1)

        # unpack from conditions
        p0 = atmosphere.pressure

        # unpack from inputs

        Tt_out = self.stagnation_temperature_out
        T_out = self.static_temperature_out
        Pt_in = self.stagnation_pressure_in
        u_out = self.u_out
        M_out = self.M_out
        jet_thrust = self.residual_thrust_nd

        # unpack from TP
        # pid      = turboprop['nozzle_pressure_ratio']

        # Method to compute the output variables

        # --Getting the output stagnation quantities
        Pt_out = Pt_in * pid
        ht_out = Cp * Tt_out

        # in case pressures go too low
        """if np.any(Pt_out<Po):
            warn('Pt_out goes too low',RuntimeWarning)
            Pt_out[Pt_out<Po] = Po[Pt_out<Po]"""

        # compute the output Mach number, static quantities and the output velocity
        # calcola tutto a partire da u_out

        if Pt_out / p0 < 1.893:
            P_out = p0
        else:
            P_out = 1.893 * Pt_out

        P_jet = jet_thrust * 1000 / 14.92

        # pack computed quantities into outputs
        self.stagnation_pressure_out = Pt_out
        self.stagnation_enthalpy_out = ht_out
        self.static_pressure_out = P_out
        self.jet_power = P_jet
        self.M_out_sizing = M_out

    def compute_offdesign(self, etapold, pid):

        # unpack from inputs
        Tt_in = self.stagnation_temperature_in
        Pt_in = self.stagnation_pressure_in
        M_out = self.M_out

        # unpack from self
        # pid      =  turboprop['nozzle_pressure_ratio']
        # etapold  =  turboprop['nozzle_eta_pol']

        # Method to compute the output variables
        gamma = 1.321
        Pt_out = Pt_in * pid
        Tt_out = Tt_in * pid ** ((gamma - 1) / (gamma * etapold))
        T_out = Tt_out / (1 + ((gamma - 1) / 2 * M_out**2))
        u_out = M_out * (gamma * 287 * T_out) ** 0.5
        # in case pressures go too low

        self.stagnation_pressure_out = Pt_out
        self.stagnation_temperature_out = Tt_out
        self.velocity_out = u_out
        #        self.stagnation_enthalpy_out     = ht_out
        # self.outputs.static_enthalpy         = h_out
        #        self.static_pressure_out         = P_out
        self.static_temperature_out = T_out
