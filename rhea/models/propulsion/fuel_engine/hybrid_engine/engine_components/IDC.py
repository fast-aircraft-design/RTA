"""
Module for calculating the parameters of Inverter / DC transformer / Converter
of hybrid propulsion
"""
import numpy as np


class IDC(object):
    def __init__(self, power_electronics_eta):
        self.power_electronics_eta = power_electronics_eta

    # def get_idc_weight(self,all_segments):
    #     '''
    #     Get the weight of all the inverters, DC transformers and converters

    #     Returns:
    #         (float): (kg) weight of idc
    #     '''
    #     P_vec= []
    #     for segm in all_segments:
    #         P_vec.extend(segm[16][:])

    #     P_max = np.amax(P_vec)

    #     w_idc = P_max / self.aircraft.vars_propulsion_hybrid_TP['inverter_power_mass']
    #     return w_idc

    def get_idc_power(self, P_elec, power_flow):
        """ """
        if power_flow == "downstream":
            P_idc = P_elec / self.power_electronics_eta
        else:
            P_idc = P_elec * self.power_electronics_eta

        return P_idc
