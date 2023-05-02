
import numpy as np
from warnings import warn

class HPC(object):

    def compute_design(self,pid,etapold):

        
        #computing the working fluid properties
        gamma                  = 1.4
        Cp                     = gamma*287.87/(gamma-1)
        
        #unpack from inputs
        Tt_in    = self.stagnation_temperature_in
        Pt_in    = self.stagnation_pressure_in
        
        #unpack from TP
        # pid      = turboprop['hpc_pressure_ratio']
        # etapold  =  turboprop['hpc_eta_pol']
        
        #Method to compute compressor properties
        
        #Compute the output stagnation quantities based on the pressure ratio of the component
        ht_in     = Cp*Tt_in
        Pt_out    = Pt_in*pid
        Tt_out    = Tt_in*pid**((gamma-1)/(gamma*etapold))
        ht_out    = Cp*Tt_out
        #print 'pid', pid
        #print 'Tin',Tt_in
        #compute the work done by the compressor(for matching with the turbine)
        work_done = (ht_out- ht_in)

        #pack computed quantities into the outputs
        self.stagnation_enthalpy_in      = ht_in
        self.stagnation_temperature_out  = Tt_out
        self.temperature_ratio  = Tt_out/Tt_in
        self.pressure_ratio  = pid
        self.stagnation_pressure_out     = Pt_out
        self.stagnation_enthalpy_out     = ht_out
        self.work_done               = work_done
    
    
    def compute_offdesign(self,etapold):

        
        #computing the working fluid properties
        gamma                  = 1.4
        Cp                     = gamma*287.87/(gamma-1)
        
        #unpack from inputs
        Tt_in    = self.stagnation_temperature_in
        Pt_in    = self.stagnation_pressure_in

        #unpack from self
        tau      = self.temperature_ratio
        #etapold  =  turboprop['hpc_eta_pol']
        
        #Method to compute compressor properties
        
        #Compute the output stagnation quantities based on the pressure ratio of the component
        ht_in     = Cp*Tt_in
        Tt_out    = Tt_in*tau
        Pt_out    = Pt_in*tau**((gamma*etapold)/(gamma-1))
        ht_out    = Cp*Tt_out
        
        #compute the work done by the compressor(for matching with the turbine)
        work_done = (ht_out- ht_in)
        
        #pack computed quantities into the outputs
        self.stagnation_enthalpy_in      = ht_in
        self.pressure_ratio              = Pt_out/Pt_in
        self.stagnation_temperature_out  = Tt_out
        self.stagnation_pressure_out     = Pt_out
        self.stagnation_enthalpy_out     = ht_out
        self.work_done                   = work_done

