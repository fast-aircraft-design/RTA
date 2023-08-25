"""
Module for calculating the parameters of Inverter / DC transformer / Converter
of hybrid propulsion
"""
import scipy.constants as const


class Fuel_cell(object):
    def __init__(self, fuel_cell_eta):
        self.H2_specific_energy = 33330.0 * const.hour  # Ws/kg
        self.fuel_cell_eta = fuel_cell_eta

    def get_fc_perfo(self, P_elec, power_flow):

        FC_H2 = P_elec / self.H2_specific_energy / self.fuel_cell_eta  # kg/s

        return FC_H2
