class Combustor(object):
    def compute_design(self, Tt4, pib, eta_b, To):
        # computing the working fluid properties
        Cp = 1.4 * 287.87 / (1.4 - 1)
        # To                     = conditions[0]

        # unpacking the values form inputs
        Tt_in = self.stagnation_temperature_in
        Pt_in = self.stagnation_pressure_in
        htf = self.htf

        # unpacking values from tp

        # Tt4    = turboprop['T4']
        # pib    = turboprop['combustor_pressure_ratio']
        # eta_b  = turboprop['combustor_eta']
        # method to compute combustor properties
        cp_t = 1120.0

        # method - computing the stagnation enthalpies from stagnation temperatures
        ht_out = cp_t * Tt4
        ht_in = Cp * Tt_in

        # Using the Turbine exit temperature, the fuel properties and freestream temperature to compute the fuel to air ratio f
        f = (ht_out - ht_in) / (eta_b * htf - ht_out)
        # Computing the exit static and stagnation conditions
        Pt_out = Pt_in * pib

        # pack computed quantities into outputs
        self.tau_b = Tt4 / To
        self.stagnation_pressure_out = Pt_out
        self.pressure_ratio = pib
        self.stagnation_enthalpy_out = ht_out
        self.fuel_to_air_ratio = f

    # __call__ = compute
    def compute_offdesign(self, eta_b, pib):
        # computing the working fluid properties
        Cp = 1.4 * 287.87 / (1.4 - 1)

        # unpacking the values form inputs
        Tt_in = self.stagnation_temperature_in
        Pt_in = self.stagnation_pressure_in
        htf = self.htf
        Tt4 = self.TIT

        # unpacking values from tp
        # pib    = turboprop['combustor_pressure_ratio']
        # eta_b  = turboprop['combustor_eta']

        # method to compute combustor properties
        cp_t = 1120.0
        # method - computing the stagnation enthalpies from stagnation temperatures
        ht_out = cp_t * Tt4
        ht_in = Cp * Tt_in

        # Using the Turbine exit temperature, the fuel properties and freestream temperature to compute the fuel to air ratio f
        f = (ht_out - ht_in) / (eta_b * htf - ht_out)

        # Computing the exit static and stagnation conditions
        Pt_out = Pt_in * pib

        # pack computed quantities into outputs
        self.stagnation_temperature_out = Tt4
        self.stagnation_pressure_out = Pt_out
        self.stagnation_enthalpy_out = ht_out
        self.fuel_to_air_ratio = f
