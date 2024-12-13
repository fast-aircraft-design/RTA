class Ram(object):
    def compute(self, atmosphere, mach):
        # unpack from conditions
        Po = atmosphere.pressure
        To = atmosphere.temperature
        M = mach

        # computing the working fluid properties
        gamma = 1.4
        Cp = 1.4 * 287.87 / (1.4 - 1)

        # Compute the stagnation quantities from the input static quantities
        stagnation_temperature = To * (1 + ((gamma - 1) / 2 * M * M))
        stagnation_pressure = Po * ((1 + (gamma - 1) / 2 * M * M) ** 3.5)
        stagnation_enthalpy = Cp * stagnation_temperature

        # pack variables
        self.pressure_ratio = stagnation_pressure / Po
        self.temperature_ratio = stagnation_temperature / To
        self.stagnation_temperature = stagnation_temperature
        self.stagnation_pressure = stagnation_pressure
        self.stagnation_enthalpy = stagnation_enthalpy
