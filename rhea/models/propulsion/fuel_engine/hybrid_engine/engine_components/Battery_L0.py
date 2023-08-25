"""
Module for calculating the parameter of battery in hybrid propulsion
"""
import numpy as np
import math
import matplotlib.pyplot as plt


class Battery(object):
    def __init__(self, bat_eta):
        self.bat_eta = bat_eta

    def get_bat_perfo(self, P_elec):

        BAT_ec = P_elec / self.bat_eta  # W

        return BAT_ec
