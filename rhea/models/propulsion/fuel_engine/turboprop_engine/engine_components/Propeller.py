
import pickle
from sklearn.preprocessing import PolynomialFeatures
from scipy import constants
import numpy as np
from fastoad.constants import FlightPhase
import os

from fastoad.utils.physics import Atmosphere
from typing import Union, Sequence, Tuple, Optional

from scipy.optimize import fsolve
script_path = os.path.abspath(__file__) # i.e. /path/to/dir/foobar.py
rhea_path = script_path.split('\\models')[0]
RHEA_path=script_path.split('\\rhea')[0]

class Propeller(object):
    
    def select(self, function, fidelity,data, atmosphere,mach,phase,P_T):
        
        func=getattr(self, function+'_'+fidelity)

        return func(data,atmosphere,mach,phase,P_T)

    def power_to_thrust_ML(
        self,
        data,
        atmosphere: Atmosphere,
        mach: Union[float, Sequence[float]],
        phase: Union[FlightPhase, Sequence],
        shaft_power: Union[float, Sequence[float]],
        ) -> np.ndarray:
        """
        Computation of propeller thrust given propeller shaft power.

        :param shaft_power: shaft_power in W
        :param atmosphere: Atmosphere instance at intended altitude
        :param mach: Mach number(s)
        :return: thrust (in N)
        """
        
        shp_prop=shaft_power* data.gearbox_eta/constants.hp
        altitude = atmosphere.get_altitude(altitude_in_feet=True)
        a = atmosphere.speed_of_sound
        V_TAS = mach*a /constants.knot
        DISA = atmosphere._return_value(atmosphere.delta_t)
        # DISA=0
        #print(altitude,V_TAS,shp_prop,mach,a)
        RC, _ = data.phase_to_rc(phase)

                     
        #prop eta interp
        # if phase == 1 or phase==8 or phase==9:
        #     eta=0.
        #     poly = PolynomialFeatures(degree=3)
        #     x=poly.fit_transform(np.array([altitude,mach],dtype=object).reshape(1,-1))
        #     filename = os.path.join(rhea_path,'resources/gasturbine/PW100/Metamodels/PW100_TO_FN.sav')
        #     loaded_model = pickle.load(open(filename, 'rb'))
        #     T_prop =loaded_model.predict(x)[0] *10 #daN to N
        if phase == 1 or phase==8 or phase==9:
            eta=0.
            poly = PolynomialFeatures(degree=3)
            x = poly.fit_transform(np.array([altitude, mach, DISA], dtype=object).reshape(1, -1))
            T_prop = data.dict_metamodels[RC]['fn'].predict(x)[0]*10
            #T_prop =loaded_model.predict(x)[0] *10 #daN to N          
            
        else:
            poly = PolynomialFeatures(degree=4)
            x=poly.fit_transform(np.array([altitude,V_TAS,shp_prop],dtype=object).reshape(1,-1))
            #filename ='C:/Users/LA202059/Desktop/RHEA/rhea/resources/propeller/H568F/Metamodels/NP82/power_to_eta_4th_degree.sav'
            filename = os.path.join(rhea_path,'resources/propeller/H568F/Metamodels/NP82/power_to_eta_4th_degree.sav')
            loaded_model = pickle.load(open(filename, 'rb'))
            eta =data.k_prop * loaded_model.predict(x)[0]
            #print(eta)    
            T_prop =  eta * shp_prop * constants.hp / (V_TAS* constants.knot) #N 
        

        return T_prop,eta


    def thrust_to_power_ML(
        self,
        data,
        atmosphere: Atmosphere,
        mach: Union[float, Sequence[float]],
        phase: Union[FlightPhase, Sequence],
        thrust: Union[float, Sequence],
        ) -> np.ndarray:
        """
        Computation of propeller shaft power given propeller thrust WITHOUT FR.


        :param atmosphere: Atmosphere instance at intended altitude
        :param mach: Mach number(s)
        :return: thrust (in N)
        """
        
        altitude = atmosphere.get_altitude(altitude_in_feet=True)
        a = atmosphere.speed_of_sound
        V_TAS = mach*a/constants.knot


        def func(shp_prop,altitude,V_TAS,thrust):
            poly = PolynomialFeatures(degree=4)
            x=poly.fit_transform(np.array([altitude,V_TAS,shp_prop],dtype=object).reshape(1,-1))
            #filename ='C:/Users/LA202059/Desktop/RHEA/rhea/resources/propeller/H568F/Metamodels/NP82/power_to_eta_4th_degree.sav'
            filename = os.path.join(rhea_path,'resources/propeller/H568F/Metamodels/NP82/power_to_eta_4th_degree.sav')

            loaded_model = pickle.load(open(filename, 'rb'))
            eta = loaded_model.predict(x)[0]
            shp=thrust* (V_TAS* constants.knot)/eta/constants.hp 
            return shp_prop-shp   
        
        initial_value= (V_TAS*constants.knot)*thrust/0.9/constants.hp 
        shp_prop = fsolve(func, initial_value,args=(altitude,V_TAS,thrust))[0]                


        eta=V_TAS*constants.knot*thrust/(shp_prop*constants.hp)
                   

        shaft_power= shp_prop/data.gearbox_eta*constants.hp
 
      
        return shaft_power,eta


    
    def power_to_thrust_ADT(
        self,
        data,
        atmosphere: Atmosphere,
        mach: Union[float, Sequence[float]],
        phase: Union[FlightPhase, Sequence],
        shaft_power: Union[float, Sequence[float]],
        ) -> np.ndarray:
        """
        Computation of propeller thrust given propeller shaft power.

        :param shaft_power: shaft_power in W
        :param atmosphere: Atmosphere instance at intended altitude
        :param mach: Mach number(s)
        :return: thrust (in N)
        """
        shp_prop=shaft_power* data.gearbox_eta/constants.hp  #hp
        altitude = atmosphere.get_altitude(altitude_in_feet=True)  #ft
        a = atmosphere.speed_of_sound
        #print(altitude,V_TAS,shp_prop,mach,a)

        #inputs
        V_TAS = mach*a #m/s 
        rho= atmosphere.density
        d=data.d_prop
        k_corr=0.895
        #evaluate thrust from given power
        def P_to_T(T_prop,shp_prop,V_TAS,rho,d):
            return (T_prop*V_TAS/(shp_prop* constants.hp))-2/(1+(1+(T_prop/(0.5 * rho * constants.pi/4 * d**2 * V_TAS**2)))**0.5)
    
        if mach<0.2:
            # T_prop=40240.
            eta=0.
            #shp=data.k_gb_NTO *data.RTO_power/constants.hp
            #T_prop_0= 55000*shp/(1200*d/constants.foot)*constants.pound_force  #Skellett, A. M. National Advisory Committee for Aeronautics, Nineteenth Annual Report. Report n447
            T_prop_0= 55000*shp_prop/(1200*d/constants.foot)*constants.pound_force  #Skellett, A. M. National Advisory Committee for Aeronautics, Nineteenth Annual Report. Report n447
            
            #thrust at mach=0.2
            T_prop_ref =  fsolve(P_to_T, 1,args=(shp_prop,0.2*a,rho,d))[0]
            T_prop_ref =  T_prop_ref * k_corr * data.k_prop    
            
            x = [0,0.2]
            y = [float(T_prop_0), float(T_prop_ref)]
            
            T_prop = np.array([np.interp(float(mach), x, y)])
        else:
            
            T_prop =  fsolve(P_to_T, 1,args=(shp_prop,V_TAS,rho,d))[0]
            #print( T_prop * k_corr)
            T_prop =  T_prop * k_corr * data.k_prop 
            #print(T_prop) 
            eta = T_prop * V_TAS/(shp_prop* constants.hp) #N 
   
                      
        return T_prop,eta

    def thrust_to_power_ADT(
        self,
        data,
        atmosphere: Atmosphere,
        mach: Union[float, Sequence[float]],
        phase: Union[FlightPhase, Sequence],
        thrust: Union[float, Sequence],
        ) -> np.ndarray:
        """
        Computation of propeller shaft power given propeller thrust WITHOUT FR.


        :param atmosphere: Atmosphere instance at intended altitude
        :param mach: Mach number(s)
        :return: thrust (in N)
        """
        
        a = atmosphere.speed_of_sound
        altitude = atmosphere.get_altitude(altitude_in_feet=True)

        V_TAS = mach*a #m/s
        rho= atmosphere.density
        d=data.d_prop
        k_corr=0.895

        #evaluate required power from given thrust


        eta= 2/(1+(1+(thrust/(0.5 * rho * constants.pi/4 * d**2 * V_TAS**2)))**0.5)
        eta= k_corr * data.k_prop * eta
        
        shp_prop=thrust* V_TAS/eta/constants.hp 
        shaft_power= (shp_prop*constants.hp)/data.gearbox_eta
 
        return shaft_power,eta    
    

