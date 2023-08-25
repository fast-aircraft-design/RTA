"""
    FAST - Copyright (c) 2016 ONERA ISAE
"""

# =================================================================================
#   Model of the Electric Motor
#
#   INPUTS
#   -Input power [kW]
#   -Efficiency
#   -Power to mass ratio [kW/kg]
#   -Motor rotational speed [rpm]
#
#   OUTPUTS
#   -Mechanical power [kW]
#   -Electric motor weight [kg]
#   -Torque [Nm]
# =================================================================================
import numpy as np


class ElectricMotor(object):
    """
    Class of Electric Motor
    """

    def __init__(self, motor_eta):
        self.motor_eta = motor_eta

    # def get_motor_weight(self, all_segments):

    #     P_vec= []
    #     for segm in all_segments:
    #         P_vec.extend(segm[16][:])

    #     P_max = np.amax(P_vec)

    #     w_em = P_max / self.aircraft.vars_propulsion_hybrid_TP['motor_power_mass']

    #     return w_em

    # def get_motor_parameters(self, elec_power, rotational_speed):

    #    mech_power = elec_power * self.aircraft.vars_propulsion_hybrid_dep['motor_efficiency']

    #    torque = mech_power/rotational_speed

    #    return mech_power, torque

    def get_motor_power(self, elec_power, power_flow):
        if power_flow == "downstream":
            mech_power = elec_power / self.motor_eta
        else:
            mech_power = elec_power * self.motor_eta
        #       torque = mech_power/rotational_speed

        return mech_power  # , torque
