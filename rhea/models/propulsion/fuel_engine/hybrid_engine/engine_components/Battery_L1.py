"""
Module for calculating the parameter of battery in hybrid propulsion
"""
import numpy as np
import math
import matplotlib.pyplot as plt


class Battery(object):

    # def __init__(self, aircraft):
    #     self.aircraft = aircraft

    #     self.V_nom            = 3.75 #Nominal cell voltage
    #     self.C_nom            = 0.5 #Ah  #Nominal cell capacity
    #     self.V_cutoff         = 2.8 #V cutoff cell voltage
    #     self.n_series         = 100
    #     self.n_parallel       = 100
    #     self.V_target       = 2000.
    #     self.SOC_min = 0.2

    # def initialize_battery(self, energy_mass, power_mass, density):
    #     energy_density = energy_mass * density
    #     power_density = power_mass * density

    #     return energy_density, power_density

    # def get_soc(self, energy_cons):

    #     #soc_final = 1.0 - energy_cons/1000.#self.aircraft.vars_propulsion_hybrid_dep['battery_energy']
    #     soc_final=0.

    #     return soc_final

    #     def get_battery_weight(self, all_segments):

    #         fidelity =0.#CHOOSE FIDELITY LEVEL

    #         En_vec=[]
    #         P_vec= []
    #         time_vec=[]
    #         for segm in all_segments:
    #             En_vec.extend(segm[12][:])
    #             P_vec.extend(segm[13][:])

    #         for i in range(len(all_segments)):
    #             segm = all_segments[i]
    #             for x in range(len(segm[0][:])):
    #                 if i>0:
    #                     prev_segm = all_segments[i-1]
    #                     segm[0][x] += prev_segm[0][-1]
    #             time_vec.extend(segm[0][:])

    #         P_bat_max = np.amax(P_vec)

    #         if fidelity == 0:
    #             En_tot = np.amax(En_vec)*1000. #Wh
    #             W_batt_P = P_bat_max/self.aircraft.vars_propulsion_hybrid_TP['battery_eta'] / self.aircraft.vars_propulsion_hybrid_TP['battery_power_mass']
    #             W_batt_E = En_tot/ self.aircraft.vars_propulsion_hybrid_TP['battery_eta']/ self.aircraft.vars_propulsion_hybrid_TP['battery_energy_mass']/(1-self.aircraft.vars_propulsion_hybrid_TP['battery_SOCmin'])
    #             W_batt = max(W_batt_P,W_batt_E)
    #         else:
    #             time_vec = [x/60/60 for x in time_vec]
    #             dt_vec=range(len(time_vec))
    #             x=range(len(time_vec))
    #             for i in x:
    #                 if i==len(time_vec)-1:
    #                     dt_vec[i]=0
    #                 else:
    #                     dt_vec[i] = time_vec[i+1]-time_vec[i]
    # #            print dt_vec
    #             P_vec = [1000*i for i in P_vec] #kW to W
    #             W_batt_E, eta_min = self.evaluate_battery_perfo(dt_vec,P_vec,time_vec)
    #             W_batt_P = P_bat_max/eta_min / self.aircraft.vars_propulsion_hybrid_TP['battery_power_mass']
    #             W_batt = max(W_batt_P,W_batt_E)

    #         if W_batt== W_batt_P:
    #             print('Batteries sized by power')
    #         return W_batt

    #    def get_battery_power(self,P_elec):
    #        fidelity = 0.
    #        if fidelity == 0:
    #            bat_power =  P_elec/self.aircraft.vars_propulsion_hybrid_TP['battery_eta']
    #
    #        else:
    #            V = self.V_nom*self.n_series
    ##            C_rate = I/self.C_nom
    ##            self.evaluate_cell_voltage(SoC,C_rate)
    #
    #
    ##        else:
    ##            bat_power =  self.aircraft.vars_propulsion_hybrid_TP['P_nom']*self.aircraft.vars_propulsion_hybrid_TP['battery_eta']
    #
    ##        max_power = power_density * volume
    ##        self.aircraft.vars_propulsion_hybrid_dep['battery_power'] = max_power
    #        return bat_power

    # def get_battery_energy(self, volume, energy_density):

    #     energy = volume * energy_density * 1000.

    #     return energy

    def battery_sizing(self, ec, power):

        v_batt1 = (
            ec
            / self.aircraft.vars_propulsion_hybrid_dep["battery_energy_density"]
            / 1000.0
        )
        v_batt2 = (
            power
            / self.aircraft.vars_propulsion_hybrid_dep["battery_power_density"]
            / 1000.0
        )

        v_batt = max(v_batt1, v_batt2)

        return v_batt


#     def evaluate_cell_voltage(self, SoC, Crate):

#         Cnom = self.C_nom
#         #Open circuit voltage and losses
#         Voc   = -1.031 *math.exp(-35.*SoC) + 3.685 + 0.2156 *SoC - 0.1178 *SoC**2 + 0.321*SoC**3
#         Ri    = 0.1562 *math.exp(-24.37*SoC) + 0.07446
#         Rts   = 0.3208 *math.exp(-29.14*SoC) + 0.04669
#         Rtl   = 6.6030 *math.exp(-155.2*SoC) + 0.04984
#         Rtot = Ri + Rts + Rtl
#         I= Crate*Cnom
#         V_cell = Voc - Rtot*I

#         return V_cell, Voc

#     def evaluate_battery_perfo(self, dt_vec, bat_power_vec,time_vec):

#         Cnom = self.C_nom
#         SOC_min = self.SOC_min
#         time_P_comb = zip(dt_vec, bat_power_vec)
#         n_cells = self.n_series
#         n_packs = self.n_parallel
#         V_cutoff = self.V_cutoff
#         V_target = self.V_target


#         V_bat_vec = [0]*len(dt_vec)
#         I_vec = [0]*len(dt_vec)
#         C_rate_vec = [0]*len(dt_vec)
#         SOC_vec = [0]*len(dt_vec)
#         C_released_vec =[0]*len(dt_vec)
#         eta_bat_vec=[0]*len(dt_vec)
#         V_cell_vec=[0]*len(dt_vec)
#         Voc_vec = [0]*len(dt_vec)
#         E_released_vec= [0]*len(dt_vec)
#         #initialize
#         SOC=1.
#         C_released=0.
#         d_SOC=10.
#         delta_P = 10
#         i=0
#         delta_V = 10
#         V_bat = V_target


#         while i< len(time_P_comb):
#             while delta_V>0.1:

#                 while (delta_P > 0.001):
#                     I = time_P_comb[i][1]/V_bat/n_packs
#                     Crate = I/Cnom
#                     V_cell, Voc= self.evaluate_cell_voltage(SOC,Crate)
#                     V_bat = V_cell*n_cells
#                     P_out = V_bat*I*n_packs
#                     delta_P= abs(P_out-time_P_comb[i][1])
#                     if V_cell<V_cutoff:
#                         n_packs = n_packs + 100*(V_cutoff - V_cell)
# #                        print V_cell,n_packs
#                         i=0
#                         delta_P = 10.
#                         SOC=1.
#                     if i==len(time_P_comb)-1:
#                         d_SOC = (SOC - SOC_min )
#                         if abs(d_SOC) > 0.001:
#                             n_packs = n_packs - 10*d_SOC
#                             delta_P=10.
#                             i=0
#                             SOC=1.
#                 if i==0:
#                     delta_V =  abs(V_bat-V_target)
#                     if delta_V>0.1:
#                         n_cells = n_cells +0.1*(V_target- V_bat)
# #                        print n_cells
#                         delta_P=10.
#                         i=0
#                         SOC=1.
#                 else:
#                     delta_V=0.
#             eta_bat = V_cell/Voc
# #            E_released = C_released*Voc #spent energy [Wh] per cell
#             C_released = I*time_P_comb[i][0]  #spent capacity [Ah] per module/cell
#             E_released = C_released*Voc #spent energy [Wh] per cell

#             #create outputs
#             Voc_vec[i] = Voc
#             V_bat_vec[i]=V_bat
#             C_released_vec[i]=C_released
#             E_released_vec[i]=E_released
#             C_rate_vec[i]=Crate
#             eta_bat_vec[i]=eta_bat
#             SOC_vec[i]=SOC
#             I_vec[i]=I
#             V_cell_vec[i]=V_cell

#             #new values for next time step


#             SOC = SOC - C_released/Cnom

#             i+=1
#             delta_P=10
#             delta_V=10

#         C_rel_cum=[]
#         for i in range(len(C_released_vec)):
#             if i==0:
#                 C_rel_cum.append(C_released_vec[i])
#             else:
#                 C_cum =  C_released_vec[i] + C_rel_cum[i-1]
#                 C_rel_cum.append(C_cum)

#         C_nom_bat =  Cnom*n_packs #Ah
#         E_nom_bat = Cnom*n_packs*n_cells*self.V_nom #Wh
#         W_batt = Cnom*n_packs*n_cells*self.V_nom/ self.aircraft.vars_propulsion_hybrid_TP['battery_energy_mass']
#         print('Nominal battery capacity (one side):', C_nom_bat, 'Ah')
#         print('Nominal battery energy (one side)', E_nom_bat, 'Wh')
#         print('Battery mass (one side)', W_batt  )
# #        fig, ax1 = plt.subplots()
# #        ax1.set_xlabel('time')
# #        ax2=ax1.twinx()
# #        ax3 = ax1.twinx()
# #        ax4 = ax1.twinx()
# ##        ax5 = ax1.twinx()
# ##        ax6 = ax1.twinx()
# ##        ax7 = ax1.twinx()
# ##
# ##        ax2.spines["right"].set_position(("axes", 1.2))
# #        ax3.spines["right"].set_position(("axes", 1.2))
# #        ax4.spines["right"].set_position(("axes", 1.4))
# ##        ax5.spines["right"].set_position(("axes", 1.6))
# ##        ax6.spines["right"].set_position(("axes", 1.8))
# ##        ax7.spines["right"].set_position(("axes", 2.0))
# ##
# ##
# #        V, = ax1.plot(time_vec, V_cell_vec,'b',label="V")
# #        SOC, = ax2.plot(time_vec, SOC_vec,'r--',label =" SOC")
# #        eta, = ax3.plot(time_vec, eta_bat_vec,'g--',label ="eta")
# ##        I, = ax5.plot(time_vec, I_vec,'y-',label ="I")
# #        Crate, = ax4.plot(time_vec, C_rate_vec,'r-',label ="crate")
# ##        Preq, = ax6.plot(time_vec, bat_power_vec,'g',label ="Preq")
# ##        V_bat,= ax7.plot(time_vec, V_bat_vec,'b--',label ="Vpack")
# ##
# ###        ax2.set_ylim(0, 2300)
# ###        ax3.set_ylim(0, 2300)
# ###        #ax4.set_ylim(0, 2300)
# ###        ax5.set_ylim(0, 1.)
# ##
# #        ax1.set_ylabel('[Volts]')
# #        ax2.set_ylabel('[%]')
# #        ax3.set_ylabel('[-]')
# ##        ax5.set_ylabel('[A]')
# #        ax4.set_ylabel('[1/Ah]')
# ##        ax6.set_ylabel('[Watt]')
# ##        ax7.set_ylabel('[V_bat]')
# ##
# ###        ax3.set_ylabel('SHP Available [hp]')
# #        ax1.yaxis.label.set_color(V.get_color())
# #        ax2.yaxis.label.set_color(SOC.get_color())
# #        ax3.yaxis.label.set_color(eta.get_color())
# ##        ax5.yaxis.label.set_color(I.get_color())
# #        ax4.yaxis.label.set_color(Crate.get_color())
# ##
# #        plt.grid()
# ##        lines =[V, SOC, eta, I,Crate,Preq ,V_bat]
# #        lines =[V, SOC, eta, Crate]
# #        ax1.legend(lines, [l.get_label() for l in lines],bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left',
# #           ncol=3, mode="expand", borderaxespad=0.)
# #
# #        plt.show()

#         return W_batt, min(eta_bat_vec)
#     def  battery_soc_curve(Crate,Cnom):

#         #Initialize

#         SoC_vec = np.array([0.05,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.])
#         DoD_vec = (1 - SoC_vec)*100
#         Voc_vec   = []
#         Ri_vec    = []
#         Rts_vec   = []
#         Rtl_vec   = []
#         V_cell_vec= []

#         #Open circuit voltage and losses
#         I= Crate*Cnom

#         for SoC in SoC_vec:
#             Voc   = -1.031 *math.exp(-35.*SoC) + 3.685 + 0.2156 *SoC - 0.1178 *SoC**2 + 0.321*SoC**3
#             Ri    = 0.1562 *math.exp(-24.37*SoC) + 0.07446
#             Rts   = 0.3208 *math.exp(-29.14*SoC) + 0.04669
#             Rtl   = 6.6030 *math.exp(-155.2*SoC) + 0.04984
#             Rtot  = Ri + Rts + Rtl

#             Voc_vec.append(Voc)
#             Ri_vec.append(Ri)
#             Rts_vec.append(Rts)
#             Rtl_vec.append(Rtl)


#             V_cell = Voc - Rtot*I
#             V_cell_vec.append(V_cell)

#         return   V_cell_vec, DoD_vec
