
import numpy as np
from warnings import warn

class Compression_Nozzle(object):

    def compute(self,atmosphere,etapold,pid):
        
        #unpack from conditions
        Po = atmosphere.pressure

        
        #computing the working fluid properties
        gamma                  = 1.4
        Cp                     = 1.4*287.87/(1.4-1)
        
        
        #unpack from inputs
        Tt_in   = self.stagnation_temperature_in
        Pt_in   = self.stagnation_pressure_in
        
        #unpack from TP
        #pid     =  turboprop['inlet_pressure_ratio']
        #etapold =  turboprop['inlet_eta_pol']
        
        
        #--Getting the output stagnation quantities
        Pt_out  = Pt_in*pid
        Tt_out  = Tt_in*pid**((gamma-1)/(gamma*etapold))
        ht_out  = Cp*Tt_out
        
        # in case pressures go too low
        if np.any(Pt_out<Po):
            warn('Pt_out goes too low',RuntimeWarning)
            Pt_out[Pt_out<Po] = Po[Pt_out<Po]
        
        
        #compute the output Mach number, static quantities and the output velocity
        Mach    = np.sqrt( (((Pt_out/Po)**((gamma-1.)/gamma))-1.) *2./(gamma-1.))
        T_out   = Tt_out/(1+(gamma-1)/2*Mach*Mach)
        h_out   = Cp*T_out
        u_out   = np.sqrt(2*(ht_out-h_out))
          
        #pack computed quantities into outputs
        self.stagnation_temperature_out  = Tt_out
        self.stagnation_pressure_out     = Pt_out
        self.stagnation_enthalpy_out     = ht_out
        self.mach_number_out             = Mach
        self.static_temperature_out      = T_out
        self.static_enthalpy_out         = h_out
        self.velocity_out                = u_out

