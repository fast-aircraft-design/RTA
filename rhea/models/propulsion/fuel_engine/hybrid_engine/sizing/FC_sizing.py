"""
Weight computation (mass and CG)
"""
#  This file is part of FAST : A framework for rapid Overall Aircraft Design
#  Copyright (C) 2020  ONERA & ISAE-SUPAERO
#  FAST is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.

import openmdao.api as om
#from fastoad.models.options import OpenMdaoOptionDispatcherGroup
#from fastoad.models.weight.cg.cg import ComputeAircraftCG
# from rhea.models.weight.cg.cg_RHEA import CG_RHEA
# from rhea.models.weight.mass_breakdown import MassBreakdown_RHEA
import numpy as np
#from fastoad.utils.physics import Atmosphere
from openmdao.core.explicitcomponent import ExplicitComponent
from ..engine_components.ElectricMotor import ElectricMotor
# from models.propulsion.fuel_engine.hybrid_engine.engine_components.Fuel_cell_L1 import Fuel_cell
from ..engine_components.IDC import IDC
import scipy.constants as const
from scipy.interpolate import interp1d

class FC_sizing(ExplicitComponent):
    """
    Performs sizing of the fuel cell based on input nominal power. Determine optimal operational point, active cell surface and stack volume 
    based on the input nominal power, weight parameter, number of stacks and number of cells for each fuel cell stack.
    
    Model based on:
    [1] PIE-053, Validation et justification des paramètres
    [2] Practical application limits of fuel cells and batteries for zero emission vessels, Minnehan and Pratt
    [3] Design Considerations for the Electrical Power Supply of Future Civil Aircraft with Active High-Lift Systems, J.-K. Mueller
    [4] Fuel Cell Systems Explained, J. Larminie
    """
    def __init__(self):
        super().__init__()
        self.T_ref       = 298.     # Units['K']
        self.P_ref       = 1.013    # Units['bar']
        self.M_H2        = 2.02e-3  # Units['kg/mol']
        self.hf_H2Ol     = -285826. # Units['J/mol'] #  if you use this you get HHV
        self.hf_H2Og     = -241826. # Units['J/mol'] #  if you use this you get LHV
        self.hf_H2       = 0 # Units['J/mol'] # H2 enthalpy of formation at standard state
        self.hf_O2       = 0        # Units['J/mol'] # O2 enthalpy of formation at standard state
        self.sf_H2Ol     = 69.92    # Units['J/mol'] # if you use this you get HHV
        self.sf_H2Og     = 188.7   # Units['J/mol'] #  if you use this you get LHV
        self.sf_H2       = 130.68   # Units['J/mol'] # H2 entropy of formation at standard state
        self.sf_O2       = 205.14   # Units['J/mol'] # O2 entropy of formation at standard state
        self.LHV         = 119.98e6 # Units['J/kg'] # H2 Specific energy
        self.F           = const.N_A * const.e   # Faraday constant
        self.C           = 0.06 # Experimental coefficient for the pressure influence on the potential E_r

        # LIMIT CURRENT DENSITY parameters
        self.i_lA =1.45      # Units['A/cm**2']
        self.T_A  =273.15+50 # Units['K']
        self.i_lB =1.57      # Units['A/cm**2']
        self.T_B  =90+273.15 # Units['K']
        self.beta =(self.i_lA-self.i_lB)/(1/self.T_B-1/self.T_A)
        self.alpha=self.i_lA+(1/self.T_A)*(self.i_lA-self.i_lB)/(1/self.T_B-1/self.T_A)
        
        
        # EXCHANGE CURRENT DENSITY parameters
        self.Ea  = 56.9*(10**3)              # Units['J.mol'] # Activation energy
        self.A   = 3.89*(10**(5))            # Units['A/cm**2']
        

    def setup(self):
        
        self.add_input("data:propulsion:electric_systems:fuel_cell:Power_Offtake", np.nan, units="W") 
        self.add_input("data:propulsion:electric_systems:fuel_cell:Gross_net_power_ratio", np.nan) 
        self.add_input("tuning:propulsion:electric_systems:fuel_cell:k_eta", 1.)

        self.add_input("data:propulsion:electric_systems:P_nom", np.nan, units="W") 
        self.add_input("data:geometry:propulsion:motor:count",np.nan)
        self.add_input("data:propulsion:electric_systems:power_electronics:power_electronics_eta", np.nan)
        self.add_input("data:propulsion:electric_systems:motor:motor_eta", np.nan)
        self.add_input("data:propulsion:electric_systems:fuel_cell:layout:stacks", np.nan) 
        self.add_input("data:propulsion:electric_systems:fuel_cell:layout:packs", np.nan)
        self.add_input("data:propulsion:electric_systems:fuel_cell:layout:rows", 2.)
        
        self.add_input("data:propulsion:electric_systems:fuel_cell:sizing:w_p", np.nan) 
        self.add_input("data:propulsion:electric_systems:fuel_cell:sizing:A_cell", np.nan,units="m**2") 
        self.add_input("data:propulsion:electric_systems:fuel_cell:sizing:T_op", np.nan, units="K") 
        self.add_input("data:propulsion:electric_systems:fuel_cell:sizing:P_op", np.nan, units="bar")         
        
        self.add_output("data:propulsion:electric_systems:fuel_cell:layout:cells")
        self.add_output("data:propulsion:electric_systems:fuel_cell:polarization_curve:V", shape=771)
        self.add_output("data:propulsion:electric_systems:fuel_cell:polarization_curve:i", shape=771)
        self.add_output("data:propulsion:electric_systems:fuel_cell:polarization_curve:efficiency", shape=771)
        
        self.add_output("data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:V_cell")
        self.add_output("data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:V_stack")        
        self.add_output("data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:V_pack")
        #self.add_output("data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:V_core")
        
        self.add_output("data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:i_cell")
        self.add_output("data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:i_stack")        
        self.add_output("data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:i_pack")
        #self.add_output("data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:i_core")  
        self.add_output("data:propulsion:electric_systems:fuel_cell:sizing:sizing_point:P_sizing",10.,units='W')  

        self.add_output("data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:efficiency_cell")        
        self.declare_partials("*", "*", method="fd")                        




    def compute(self, inputs, outputs):   
        Power_offtake = inputs["data:propulsion:electric_systems:fuel_cell:Power_Offtake"]
        Gross_net_power_ratio = inputs["data:propulsion:electric_systems:fuel_cell:Gross_net_power_ratio"]
        motor_power=inputs["data:propulsion:electric_systems:P_nom"]
        motor_eta=inputs["data:propulsion:electric_systems:motor:motor_eta"]
        IDC_eta=inputs["data:propulsion:electric_systems:power_electronics:power_electronics_eta"]
        n_motor = inputs["data:geometry:propulsion:motor:count"]
        n_stacks= inputs["data:propulsion:electric_systems:fuel_cell:layout:stacks"]
        n_packs= inputs["data:propulsion:electric_systems:fuel_cell:layout:packs"] 
        n_rows=inputs["data:propulsion:electric_systems:fuel_cell:layout:rows"] 
        
        wp= inputs["data:propulsion:electric_systems:fuel_cell:sizing:w_p"]
        A_cell= inputs["data:propulsion:electric_systems:fuel_cell:sizing:A_cell"]
        
        
        T= inputs["data:propulsion:electric_systems:fuel_cell:sizing:T_op"] 
        
        P= inputs["data:propulsion:electric_systems:fuel_cell:sizing:P_op"]    
        
        k_eta_fc =inputs["tuning:propulsion:electric_systems:fuel_cell:k_eta"]
        
        # print('Wp',wp)
        motor_power_tot= motor_power*n_motor
        
        motor= ElectricMotor(motor_eta)
        motor_power_in = motor.get_motor_power(motor_power_tot,'downstream')
        
        power_electronics=IDC(IDC_eta)
        power_electronics_power =motor_power_in
        power_electronics_power_in = power_electronics.get_idc_power(power_electronics_power, power_flow='downstream')
        
        P_sizing_fc=power_electronics_power_in/Gross_net_power_ratio  +Power_offtake  #from gross power to net power
        
        
        b          = 8.314*T/self.F # Units['V'] # Tafel slope (for activation losses)                       
        i_l  =self.alpha-self.beta/T  # Units['A/cm**2'] # limiting current density
        i_0 = self.A*np.exp(-self.Ea/(const.R*T)) # Units['A/cm**2'] # Exchange current density
        R_ohm            = 0.245  # Units['ohm*cm**2']    
        
        # INITIALIZATION
        i                       = 0.001 # Units['A/cm**2']  # to be varied between 0 and i_l for polarization curve
        Power_vec               = [] 
        eta_vec                 = []
        V_act_vec               = [] 
        V_ohm_vec               = [] 
        V_conc_vec              = [] 
        V_pression_vec          = []
        V_tot_vec               = [] 
        f_vec                   = []
        # i_vec                   = []
        
        # THERMODYNAMIC EQUATIONS
        ########## HHV #########
        # Delta_S                 = sf_H2Ol - (sf_H2+0.5*sf_O2) # Units['J/K'] # Entropy change
        # Delta_H                 = hf_H2Ol-(hf_H2+0.5*hf_O2)   # Units['J']   # Enthalpy change
        ########## LHV ###########
        Delta_S                 = self.sf_H2Og - (self.sf_H2+0.5*self.sf_O2) # Units['J/K'] # Entropy change
        Delta_H                 = self.hf_H2Og-(self.hf_H2+0.5*self.hf_O2)   # Units['J']   # Enthalpy change
        
        Delta_G                 = Delta_H-self.T_ref*Delta_S       # Units['J']   # Gibbs free energy
        
        E_r                     = - Delta_G/(2.*self.F)  # Units['V'] # Reversible potential at standard conditions
        delta_E_r_T             = Delta_S/(2.*self.F)*(T-self.T_ref) # Units['V'] # Potential variation at non standard temperature
        delta_E_r_P_ref         = 0
        delta_E_r_P             = self.C * np.log(P/self.P_ref)+ delta_E_r_P_ref # Units['V'] # Potential variation at non standard pressure
        V_rev                   = E_r + delta_E_r_T + delta_E_r_P # Units['V'] # Reversible potential at non standard conditions
        
        i_vec = np.linspace(i,float(i_l), num=771).tolist()
        # while (i < i_l):  
        # print(i_l,i_vec[-1],T,len(i_vec))
        for i in i_vec:
            # VOLTAGE LOSSES (Units['V'])
            v_act          = b*np.arcsinh(i/(2*i_0))  # activation losses
            v_ohm          = i*R_ohm                  # ohmic losses
            v_conc         =8.314*T/(2*self.F)*(1+1./0.5)*np.log10(i_l/(i_l-i)) # concentration losses
            V_irrev        = v_act+v_ohm+v_conc # total voltage losses
            V              = (V_rev-V_irrev)*k_eta_fc  # actual voltage     
            if V<0:
                V=0.0001        
            V_act_vec.append(v_act)
            V_ohm_vec.append(v_ohm)
            V_conc_vec.append(v_conc)
            V_tot_vec.append(float(V))
           
            # POWER (Units['W/cm^2'])
            P_adim = V*i  
            Power_vec.append(P_adim)
        
            # EFFICIENCY
            #eta = Delta_G/Delta_H*V/V_rev
            eta = -V*2*self.F/Delta_H
            eta_vec.append(eta) 
          
            # Objective function for trade off between efficiency and power
            f = wp*P_adim+(1-wp)*eta
            f_vec.append(float(f))
           
            # repeat for next current
            # i_vec.append(i)
            # i+= 0.002          

        ### OPTIMUM POINT correspond to the maximisation of the objective function f ###
        f_max = max(f_vec)
        # print(np.shape(f_vec))
        # intensity density corresponding to f_max
        # f_vec=np.array(f_vec).reshape((771,))
        # print(np.shape(f_vec))
        f_i       = interp1d(f_vec, i_vec)
        i_optimum = f_i(f_max)
        
        # voltage corresponding to i_optimum
        V_i       = interp1d(i_vec, V_tot_vec)
        V_optimum = V_i(i_optimum)
        
        # Arguments eta and power at optimum point
        #eta_optimum            = Delta_G/Delta_H*V_optimum/V_rev
        eta_optimum = -V_optimum*2*self.F/Delta_H
        P_optimum_adim = V_optimum*i_optimum
        
        ### FUEL CELL SIZING ###
        P_mass_sizing= max(Power_vec)/P_optimum_adim * P_sizing_fc
        
        
        #i_tot=i_optimum*(A_cell*10000)*n_packs
        i_pack = i_optimum*(A_cell*10000)*n_rows
        #i_pack=i_tot/n_packs
        i_stack=i_pack/n_rows
        i_cell=i_stack
        
        
        #V_tot = P_sizing_fc/i_tot
        V_pack=P_sizing_fc / n_packs / i_pack
        V_stack= V_pack/n_stacks*n_rows
        n_cells=V_stack/V_optimum
        
        
        outputs["data:propulsion:electric_systems:fuel_cell:layout:cells"]=n_cells
        outputs["data:propulsion:electric_systems:fuel_cell:polarization_curve:V"]= V_tot_vec
        outputs["data:propulsion:electric_systems:fuel_cell:polarization_curve:i"] = i_vec
        outputs["data:propulsion:electric_systems:fuel_cell:polarization_curve:efficiency"] = eta_vec
        
        outputs["data:propulsion:electric_systems:fuel_cell:sizing:sizing_point:P_sizing"]=P_mass_sizing
        
        outputs["data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:V_cell"]=V_optimum
        outputs["data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:V_stack"]=  V_stack
        outputs["data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:V_pack"]=V_pack
        #outputs["data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:V_core"]=V_tot
        
        outputs["data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:i_cell"]=i_cell
        outputs["data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:i_stack"]=i_stack
        outputs["data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:i_pack"]=i_pack
        #outputs["data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:i_core"]= i_core
        
        outputs["data:propulsion:electric_systems:fuel_cell:sizing:nominal_operational_point:efficiency_cell"]= eta_optimum       
