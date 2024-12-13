import numpy as np


class Thrust(object):
    def compute_design(
        self,
        atmosphere,
        M,
        bleed_offtake,
        eta_mech,
        etapolt_turb,
        pid_nozzle,
        etapolt_nozz,
    ):
        # unpack from conditions
        p0 = atmosphere.pressure
        a0 = atmosphere.speed_of_sound
        u0 = M * a0

        # computing the working fluid properties
        gamma = 1.4
        Cp = 1.4 * 287.87 / (1.4 - 1)
        R = 287.87

        # unpacking from inputs
        f = self.fuel_to_air_ratio
        Tt_in = self.stagnation_temperature_in
        Pt_in = self.stagnation_pressure_in

        # unpack from TP
        # bleed_offtake   = turboprop['HP_bleed'] * turboprop['LP_bleed']
        # eta_mech                        =  turboprop['pt_eta_mech']
        # etapolt_turb                    =  turboprop['pt_eta_pol']
        # pid_nozzle                      =  turboprop['nozzle_pressure_ratio']
        # etapolt_nozz                    =  turboprop['nozzle_eta_pol']
        # area_ratio                      =self.inputs.area_ratio
        # unpacking from self
        # thrust_split                    = self.thrust_split
        eta_prop = 0.88  # assumption for thrsut split calculation

        # Initialize
        Pt_out = float()
        Tt_out = float()
        T_core_nozzle = float()
        T_prop = float()
        Pt_in = Pt_in.item()
        Tt_in = Tt_in.item()

        # defintion of enthalpy
        gamma_t = 1.321
        cp_t = gamma_t * 287.87 / (gamma_t - 1)
        #        cp_t=1120.
        ht_in = Cp * Tt_in

        # initialize
        ht_out = ht_in
        dT = 10000.0

        while abs(dT) > 20:

            Pt_out = Pt_in * (ht_out / (cp_t * Tt_in)) ** (
                gamma_t / ((gamma_t - 1) * etapolt_turb)
            )

            if Pt_out / p0 * pid_nozzle < 1.893:
                M_out_nozzle = np.sqrt(
                    2.0
                    / (gamma_t - 1)
                    * ((Pt_out / p0 * pid_nozzle) ** ((gamma_t - 1) / gamma_t) - 1)
                )
                Tt_out = ht_out / cp_t
                Tt_out_nozzle = Tt_out * pid_nozzle ** (
                    (gamma_t - 1) / (gamma_t * etapolt_nozz)
                )
                T_out_nozzle = Tt_out_nozzle / (
                    1 + ((gamma_t - 1) / 2 * M_out_nozzle**2)
                )
                u_out_nozzle = np.sqrt(gamma_t * R * T_out_nozzle) * M_out_nozzle
            else:
                M_out_nozzle = 1
                Tt_out = ht_out / cp_t
                Tt_out_nozzle = Tt_out * pid_nozzle ** (
                    (gamma_t - 1) / (gamma_t * etapolt_nozz)
                )
                T_out_nozzle = Tt_out_nozzle / (
                    1 + ((gamma_t - 1) / 2 * M_out_nozzle**2)
                )
                u_out_nozzle = np.sqrt(gamma_t * R * T_out_nozzle) * M_out_nozzle

            T_prop = (
                eta_prop / u0 * ((ht_in - ht_out) * (1 + f) * eta_mech) * bleed_offtake
            )
            T_core_nozzle = (1 + f) * (u_out_nozzle) - u0
            shaft_pow = -(ht_out - ht_in) * eta_mech * (1 + f) * bleed_offtake

            dT = T_core_nozzle - (0.0025 * shaft_pow)  # 0.00105
            ht_out -= 100.0
        ht_out = ht_out + 100.0

        # pack outputs
        self.prop_thrust_nd = T_prop
        self.residual_thrust_nd = T_core_nozzle
        self.power_turbine_ht_out = ht_out
        self.pow_turb_Tt_out = Tt_out
        self.nozzle_Tt_out = Tt_out_nozzle
        self.nozzle_T_out = T_out_nozzle
        self.nozzle_u_out = u_out_nozzle
        self.M_out_nozzle = M_out_nozzle
        self.pow_turb_Pt_out = Pt_out

    # __call__ = compute

    def compute_offdesign(self, atmosphere, mach, HP_bleed, LP_bleed, eta_mech):

        # unpack from conditions
        a0 = atmosphere.speed_of_sound
        M = mach
        u0 = M * a0

        # unpacking from inputs
        f = self.fuel_to_air_ratio
        Tt_out = self.Tt_PTout
        Tt_in = self.Tt_PTin
        u_out = self.u_out

        # unpack from TP
        bleed_offtake = HP_bleed * LP_bleed
        # eta_mech        =  turboprop['pt_eta_mech']

        eta_prop = 0.88
        gamma_t = 1.321
        cp_t = gamma_t * 287.87 / (gamma_t - 1)
        # initialize
        ht_in = cp_t * Tt_in
        ht_out = cp_t * Tt_out
        T_prop = eta_prop / u0 * ((ht_in - ht_out) * (1 + f) * eta_mech) * bleed_offtake
        T_core_nozzle = (1 + f) * u_out - u0
        if u0 > 51.44:
            P_jet = T_core_nozzle * u0 / 0.8
        else:
            P_jet = T_core_nozzle / 14.92 * 1000
        # pack outputs
        self.prop_thrust_nd = T_prop
        self.residual_thrust_nd = T_core_nozzle
        self.jet_power = P_jet
        self.delta_h = ht_in - ht_out
