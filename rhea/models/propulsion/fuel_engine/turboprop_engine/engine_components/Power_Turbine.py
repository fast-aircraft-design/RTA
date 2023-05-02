


import numpy as np
import scipy as sp
from scipy.optimize import fsolve

class Power_Turbine(object):
   
    def compute_design(self,eta_mech,area_ratio):
       
        #unpack from inputs
        Tt_in           = self.stagnation_temperature_in
        Tt_out          =self.stagnation_temperature_out 
        Pt_in           = self.stagnation_pressure_in
        Pt_out           = self.stagnation_pressure_out         
        ht_out          = self.stagnation_enthalpy_out 
        f               = self.fuel_to_air_ratio
        u_in            =self.nozzle_u_out
        bleed_offtake   = self.bleed_offtake
        
        
#        if self.inputs.shaft_power_off_take is not None:
#            shaft_takeoff = self.inputs.shaft_power_off_take.work_done
#        else:
#            shaft_takeoff = 0.
        shaft_takeoff = 0.

        #unpack from TP
        # eta_mech        =  turboprop['pt_eta_mech']
        # area_ratio = turboprop['nozzle_area_ratio']

#        bleed_offtake   = turboprop['HP_bleed'] * turboprop['LP_bleed']
        #method to compute turbine properties
#        cp_t =1120.  
        gamma           = 1.321        
        cp_t            = gamma*287.87/(gamma-1)        
        
        #Compute the output stagnation quantities from the inputs and the energy drop computed above
        ht_in     =  cp_t*Tt_in   
   
        
        #Using the work done by the compressors/fan and the fuel to air ratio to compute the energy drop across the turbine
        shaft_takeoff = - (ht_out-ht_in) * eta_mech * (1 + f) * bleed_offtake  
        u_out = u_in/area_ratio
    
        #pack the computed values into outputs
        self.shaft_takeoff            = shaft_takeoff
        self.pressure_ratio           = Pt_out/Pt_in
        self.temperature_ratio        = Tt_out/Tt_in
        self.pow_turb_u_out           = u_out
        self.stagnation_enthalpy_in   = ht_in 
    #__call__ = compute
        
    def compute_offdesign(self,etapolt_turb,eta_mech):


        
        #unpack from conditions
        gamma           = 1.321        
        cp_t            = gamma*287.87/(gamma-1) #        Cp                              = conditions.freestream.specific_heat_at_constant_pressure
#        u0                              = conditions.freestream.velocity[iter]
#        p0                              = conditions.freestream.pressure[iter]  
#        R                               = conditions.freestream.universal_gas_constant         
        #unpack from inputs
        Tt_in           = self.stagnation_temperature_in
#        Tt_in           = Tt_in.reshape(-1,)
        Pt_in           = self.stagnation_pressure_in
#        Pt_in           = Pt_in.reshape(-1,)        
        f               = self.fuel_to_air_ratio
        pi_r            =self.pi_r 
        pi_d            =self.pi_d 
        pi_c            =self.pi_c
#        pi_c            = pi_c.reshape(-1,)
        pi_b            =self.pi_b 
        pi_hlt          =self.pi_t  #lpt*hpt
#        pi_hlt          = pi_hlt.reshape(-1,)         
        tau_hlt         =self.tau_t      #lpt*hpt
#        tau_hlt         = tau_hlt.reshape(-1,)
        pi_n            =self.pi_n  
        M9_siz          =self.M9_sizing
        pi_hlpt_siz     =self.pi_t_sizing  #lpt*hpt*pt
        tau_hlpt_siz    =self.tau_t_sizing  #lpt*hpt*pt 



#        if self.inputs.shaft_power_off_take is not None:
#            shaft_takeoff = self.inputs.shaft_power_off_take.work_done
#        else:
#            shaft_takeoff = 0.
        shaft_takeoff = 0.

        #unpack from self
        #eta_mech        =  turboprop['pt_eta_mech']
        #etapolt_turb    =  turboprop['pt_eta_pol']
#        bleed_offtake   = turboprop['HP_bleed'] * turboprop['LP_bleed']
        bleed_offtake   = self.bleed_offtake


        #Compute the output stagnation quantities from the inputs and the energy drop computed above
#        cp_t = 1120.
        ht_in     =  cp_t*Tt_in   
        M_out_nozzle = 3*M9_siz   #if initial M_out_nozzle too low, then convergence problem
        M=float()
        delta_M = 100
        cont=0               
        while delta_M > 0.001: 
            pi_pt= (1+(gamma-1)/2*M_out_nozzle**2)**(gamma/(gamma-1))/(pi_r*pi_d*pi_c*pi_b*pi_n*pi_hlt) 
            tau_pt = pi_pt**((gamma-1)*etapolt_turb/gamma) #tau_pt
            def equation(param):
                M = param                
                return (M-M9_siz*((tau_hlt*tau_pt)/tau_hlpt_siz)**0.5*pi_hlpt_siz/(pi_pt*pi_hlt)*((1+(gamma-1)/2*M**2)/(1+(gamma-1)/2*M9_siz**2))**((gamma+1)/(2*(gamma-1))))
            param_0 = (M9_siz)
            M =  fsolve(equation, param_0)  
            delta_M=abs(M_out_nozzle-M)
            M_out_nozzle=M
            cont=cont+1
            if cont==90:
                print('Power turbine off-design calculation did not converge')

        #Using the work done by the compressors/fan and the fuel to air ratio to compute the energy drop across the turbine
        Tt_out= tau_pt*Tt_in
        ht_out=cp_t*Tt_out
        Pt_out=pi_pt*Pt_in
        shaft_takeoff = - (ht_out-ht_in) * eta_mech * (1 + f) * bleed_offtake  
        

        #pack the computed values into outputs

        self.shaft_takeoff           = shaft_takeoff
        self.stagnation_temperature_out  = Tt_out        
        self.stagnation_pressure_out     = Pt_out
        self.pressure_ratio          = pi_pt
        #self.outputs.u_out_nozzle            = u_out_nozzle
        self.M_out_nozzle            = M_out_nozzle
        #self.outputs.T_out_nozzle            = T_out_nozzle
        #self.outputs.Tt_out_nozzle            = Tt_out_nozzle        
        self.stagnation_enthalpy_in      = ht_in     
        #self.outputs.pow_turb_u_out          = u_out_PT        