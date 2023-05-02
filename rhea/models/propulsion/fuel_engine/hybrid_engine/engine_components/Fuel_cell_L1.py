"""
Module for calculating the parameters of Inverter / DC transformer / Converter
of hybrid propulsion
"""
import numpy as np
import scipy.constants as const 
from scipy.interpolate import interp1d

class Fuel_cell(object):
    def __init__(self, V_vec,i_vec,Nstacks,Npacks, Acell, Ncells,Nmotors):#, k_eta=1):
        
        self.F           = const.N_A * const.e   # Faraday constant
        self.hf_H2Og     = -241826. # Units['J/mol'] #  if you use this you get LHV
        self.hf_H2       = 0 # Units['J/mol'] # H2 enthalpy of formation at standard state
        self.hf_O2       = 0        # Units['J/mol'] # O2 enthalpy of formation at standard state
        self.Delta_H     = self.hf_H2Og-(self.hf_H2+0.5*self.hf_O2) 
        self.Ncells       =Ncells
        self.Acell       =Acell*10000 #cm**2
        self.Npacks      =Npacks
        self.Nstacks     =Nstacks
        self.Nmotors     =Nmotors
        self.M_H2        = 2.02e-3  # Units['kg/mol']
        self.Power_vec   = np.array(V_vec)*np.array(i_vec)
        self.V_vec       = V_vec

    def get_fc_perfo(self,P_elec):

        # Initialisation
        P_req= P_elec*self.Nmotors
        
        P_vec_univ=[]
        V_vec_univ=[]
        Pmax = max(list(self.Power_vec))
        index_Pmax= list(self.Power_vec).index(Pmax)
        i=0
        while i<=index_Pmax:
            P_vec_univ.append(self.Power_vec[i])
            i+=1
            V_vec_univ.append(self.V_vec[i])
             
        # Voltage, current, efficiency, fuel flow calculation
        P_cell_adim      = P_req/(self.Npacks*self.Ncells*self.Acell*self.Nstacks)           
        V_req_cell  = np.interp(P_cell_adim,P_vec_univ,V_vec_univ)   
        V_req_stack = V_req_cell*self.Ncells
        V_req_pack=V_req_stack*self.Nstacks
        V_req_core=V_req_pack
        
        I_req_adim  = P_cell_adim/V_req_cell
        I_req_cell  = I_req_adim*self.Acell        
        I_req_stack = I_req_cell
        I_req_pack = I_req_stack
        I_req_core =I_req_pack*self.Npacks
        
        eta_req     = -V_req_cell*2*self.F/self.Delta_H 
        FC_H2_tot   = self.M_H2  * P_req/(2* self.F*V_req_cell)          # Units['kg/s']   # From reference[4] (eq. A2.8)
        air_flow_tot    = 3.57*(10**(-7))*2*P_req/V_req_cell     # Units['kg/s']   # From reference[4] (eq. A2.4)        

        FC_H2= FC_H2_tot/self.Nmotors
        
         
        return [FC_H2, eta_req, air_flow_tot,V_req_cell,V_req_stack,V_req_pack,V_req_core,I_req_adim,I_req_cell,I_req_stack,I_req_core]

        
      
        
