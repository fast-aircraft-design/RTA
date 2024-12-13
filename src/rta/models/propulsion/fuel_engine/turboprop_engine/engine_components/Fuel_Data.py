class Fuel_data(object):
    def configure(self, fuel_type):
        if fuel_type == 0:
            self.Jet_A()
        elif fuel_type == 1:
            self.Biofuel_JetA_mix()

    def Jet_A(self):
        """This sets the default values."""
        self.reactant = "O2"
        self.density = 820.0  # kg/m^3 (15 C, 1 atm)
        self.specific_energy = 42.798e6  # J/kg
        self.energy_density = 35276.4e6  # J/m^3
        self.max_mass_fraction = {
            "Air": 0.0633,
            "O2": 0.3022,
        }  # kg propellant / kg oxidizer

        # critical temperatures
        self.temperatures_flash = 311.15  # K
        self.temperatures_autoignition = 483.15  # K
        self.temperatures_freeze = 233.15  # K
        self.temperatures_boiling = 0.0  # K

    def Biofuel_JetA_mix(self):
        """This sets the default values."""
        self.reactant = "O2"
        self.specific_energy = 37.8e6 * 0.45 + 42.79e6 * 0.55  # J/Kg
