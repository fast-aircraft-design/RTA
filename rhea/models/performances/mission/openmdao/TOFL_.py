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
from scipy.optimize import fsolve,root

import numpy as np
import openmdao.api as om
import pandas as pd
from fastoad.constants import FlightPhase
from fastoad import BundleLoader
from rhea.models.aerodynamics.constants import POLAR_POINT_COUNT,CT_POINT_COUNT,ALPHA_POINT_COUNT, H_POINT_COUNT
from rhea.models.propulsion.fuel_engine.constants import POWER_RATE_COUNT
from fastoad.models.performances.breguet import Breguet
# from fastoad.models.performances.mission.segments.hold import HoldSegment
from rhea.models.performances.mission.segments.holding import HoldingSegment

from fastoad.models.performances.mission.segments.taxi import TaxiSegment
from rhea.models.performances.mission.segments.takeoff_flight import TakeOffSegment
from rhea.models.performances.mission.segments.takeoff_speeds import TakeOffSpeeds

from rhea.models.performances.mission.segments.ASD_flight import ASDSegment

from rhea.models.performances.mission.segments.OEI_ceiling import ClimbPhaseOEI

#from fastoad.models.propulsion import EngineSet
from models.propulsion.fuel_engine.turboprop_engine.base import TPEngineSet, TPH2EngineSet
from models.propulsion.fuel_engine.hybrid_engine.base import HybridEngineSet

from scipy.constants import foot, nautical_mile,knot,hour,g

from fastoad.models.performances.mission.flight.base import RangedFlight
from ..flight.standard_regional_flight import StandardFlight, DiversionFlight
#from fastoad.models.performances.mission.flight_point import FlightPoint #v0.50a
from fastoad.base.flight_point import FlightPoint 
from fastoad.models.performances.mission.polar import Polar
from fastoad.base.dict import AddKeyAttributes,AddKeyAttribute
from fastoad.utils.physics import Atmosphere
from fastoad.constants import EngineSetting
from fastoad.models.performances.mission.segments.base import SEGMENT_KEYWORD_ARGUMENTS #v0.5.2b

AddKeyAttributes(["alpha",'max_mach'])(FlightPoint)

class TakeOffFlight(om.ExplicitComponent):
    """
    Simulates a complete flight sizing mission with diversion.
    """

    def __init__(self, **kwargs):
        """
        Computes thrust, SFC and thrust rate by direct call to engine model.

        Options:
          - propulsion_id: (mandatory) the identifier of the propulsion wrapper.
          - out_file: if provided, a csv file will be written at provided path with all computed
                      flight points. If path is relative, it will be resolved from working
                      directory
        """
        super().__init__(**kwargs)
        self.flight_points = None
        self._engine_wrapper = None
        # self._engine_wrapper2 = None

    def initialize(self):
        self.options.declare("propulsion_id", default="", types=str)
        self.options.declare("out_file", default="", types=str)
        self.options.declare("prop_fid", default="ADT", types=str)

    def setup(self):
        self._engine_wrapper = BundleLoader().instantiate_component(self.options["propulsion_id"])
        self._engine_wrapper.setup(self)
        

        # Inputs -----------------------------------------------------------------------------------

        self.add_input("conv:mdao",  np.nan)
        self.add_input("data:geometry:propulsion:engine:count",  np.nan)
        
        
        
        self.add_input("data:geometry:wing:area", np.nan, units="m**2")

        self.add_input("data:aerodynamics:aircraft:cruise:CL", np.nan, shape=POLAR_POINT_COUNT)
        self.add_input("data:aerodynamics:aircraft:cruise:CD", np.nan, shape=POLAR_POINT_COUNT)
        self.add_input("data:aerodynamics:aircraft:low_speed:CL", np.nan, shape=POLAR_POINT_COUNT)
        self.add_input("data:aerodynamics:aircraft:low_speed:CD", np.nan, shape=POLAR_POINT_COUNT)
        self.add_input("data:aerodynamics:aircraft:takeoff:CL", np.nan, shape=POLAR_POINT_COUNT)
        self.add_input("data:aerodynamics:aircraft:takeoff:CD", np.nan, shape=POLAR_POINT_COUNT)
        self.add_input("data:aerodynamics:aircraft:takeoff:CL_alpha", np.nan, units="1/rad")
        self.add_input("data:aerodynamics:aircraft:takeoff:CL0", np.nan)
        self.add_input("data:aerodynamics:aircraft:takeoff:CL_max", np.nan)
        self.add_input("data:aerodynamics:aircraft:low_speed:H",np.nan ,units='m' ,shape=H_POINT_COUNT)
        self.add_input("data:aerodynamics:aircraft:low_speed:alpha", np.nan,units='deg',shape=ALPHA_POINT_COUNT)
        self.add_input("data:aerodynamics:aircraft:low_speed:CT", np.nan,shape=CT_POINT_COUNT)
        # self.add_input("data:aerodynamics:aircraft:low_speed:DCD_ext", np.nan)
        # self.add_input("data:aerodynamics:aircraft:low_speed:DCD_feather", np.nan)
        
        self.add_input("data:aerodynamics:aircraft:takeoff:CT_effect:DCL0", 0, copy_shape="data:aerodynamics:aircraft:low_speed:CT")
        self.add_input("data:aerodynamics:aircraft:takeoff:CT_effect:DCL_alpha", 0,units="1/rad",copy_shape="data:aerodynamics:aircraft:low_speed:CT")
        self.add_input("data:aerodynamics:aircraft:takeoff:OEI_effect:DCD", np.nan,copy_shape="data:aerodynamics:aircraft:low_speed:CT")
        self.add_input("data:aerodynamics:aircraft:takeoff:ground_effect:DCD",0,copy_shape="data:aerodynamics:aircraft:low_speed:alpha")
        self.add_input("data:aerodynamics:aircraft:takeoff:ground_effect:DCL", 0,copy_shape="data:aerodynamics:aircraft:low_speed:alpha")
        self.add_input("data:aerodynamics:aircraft:takeoff:ground_effect:K_H", 0,copy_shape="data:aerodynamics:aircraft:low_speed:H")
        self.add_input("data:aerodynamics:aircraft:takeoff:lg_effect:DCD", np.nan,copy_shape="data:aerodynamics:aircraft:low_speed:alpha")
        self.add_input("data:aerodynamics:aircraft:takeoff:lg_effect:DCL", np.nan,copy_shape="data:aerodynamics:aircraft:low_speed:alpha")
                
        
        self.add_input("data:weight:aircraft:MTOW", np.nan, units="kg")

        self.add_input("data:mission:sizing:taxi_out:fuel", np.nan, units="kg")
        self.add_input("data:mission:sizing:taxi_out:H2", 0., units="kg")

        self.add_input("data:mission:sizing:takeoff:V_MCA", np.nan, units="m/s")
        #self.add_input("data:mission:sizing:takeoff:V_EF", 100., units="m/s")

        self.add_input("data:mission:sizing:takeoff:Friction_coefficient_no_brake", np.nan)
        self.add_input("data:mission:sizing:takeoff:Friction_coefficient_brake", 0.6)
        
        self.add_input("data:mission:sizing:takeoff:DISA", 0)
        self.add_input("data:mission:sizing:takeoff:altitude", np.nan, units="m")
        self.add_input("data:mission:sizing:takeoff:fuel", np.nan, units="kg")
        self.add_input("data:mission:sizing:takeoff:H2", 0., units="kg")

        # self.add_output("data:mission:sizing:takeoff:fuel", np.nan, units="kg")
        self.add_input("data:mission:sizing:takeoff:thrust_rate", np.nan)
        self.add_input("data:mission:sizing:takeoff:EM_power_rate", 0.)
        self.add_input("data:mission:sizing:takeoff:TP_power_rate", 1.)        

        
        # Inputs needed in order to use the sizing output file as input .xml for the DOC mission evaluation-----------------------------------------------------------------------------------        
 
        
        self.add_input("data:mission:DOC:TOW", np.nan, units="kg")


        # self.add_input("data:mission:DOC:takeoff:V2", np.nan, units="m/s")

        self.add_input("data:mission:DOC:takeoff:altitude", np.nan, units="m")
        self.add_input("data:mission:DOC:takeoff:fuel", np.nan, units="kg")
        self.add_input("data:mission:DOC:takeoff:H2", 0., units="kg")
        


        # Outputs ----------------------------------------------------------------------------------
        self.add_output("data:mission:sizing:takeoff:TOFL",  units="m",val=0.)
        self.add_output("data:mission:sizing:takeoff:distance",  units="m",val=0.)
        self.add_output("data:mission:sizing:takeoff:V_rotate",  units="m/s",val=0.)
        self.add_output("data:mission:sizing:takeoff:V_liftoff", units="m/s",val=0.)
        self.add_output("data:mission:sizing:takeoff:V_2", units="m/s",val=0.)
        self.add_output("data:mission:sizing:takeoff:V_1", units="m/s",val=0.)
        self.add_output("data:mission:sizing:takeoff:V_EF", units="m/s",val=0.)

        
        self.declare_partials(["*"], ["*"])

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        TOW_input = inputs["data:weight:aircraft:MTOW"]
        mission_type= 'sizing'  


           
        if 'PH'  in self.options["propulsion_id"] or 'FH' in self.options["propulsion_id"]:    
            propulsion_model = HybridEngineSet(self._engine_wrapper.get_model(inputs), inputs["data:geometry:propulsion:engine:count"])   
            propulsion_model_OEI = HybridEngineSet(self._engine_wrapper.get_model(inputs), inputs["data:geometry:propulsion:engine:count"]-1)  
        elif 'PW100_H2' in self.options["propulsion_id"]:
            propulsion_model =  TPH2EngineSet(self._engine_wrapper.get_model(inputs), inputs["data:geometry:propulsion:engine:count"])
            propulsion_model_OEI =  TPH2EngineSet(self._engine_wrapper.get_model(inputs), inputs["data:geometry:propulsion:engine:count"]-1)
                
        else:
            propulsion_model = TPEngineSet(self._engine_wrapper.get_model(inputs), inputs["data:geometry:propulsion:engine:count"])
            propulsion_model_OEI = TPEngineSet(self._engine_wrapper.get_model(inputs), inputs["data:geometry:propulsion:engine:count"]-1)
        
        reference_area = inputs["data:geometry:wing:area"]

        TOargs = {
            'vmca':inputs["data:mission:sizing:takeoff:V_MCA"],
            #'vef':100.,
            'CL_alpha':inputs["data:aerodynamics:aircraft:takeoff:CL_alpha"],
            'CL0':inputs["data:aerodynamics:aircraft:takeoff:CL0"],
            'CL_max': inputs["data:aerodynamics:aircraft:takeoff:CL_max"],
                'H_list':inputs["data:aerodynamics:aircraft:low_speed:H"],
                'alpha_list':inputs["data:aerodynamics:aircraft:low_speed:alpha"],
                'CT_list':inputs["data:aerodynamics:aircraft:low_speed:CT"],
                # 'DCd_ext':inputs["data:aerodynamics:aircraft:low_speed:DCD_ext"],
                # 'DCd_feather':inputs["data:aerodynamics:aircraft:low_speed:DCD_feather"],     
                'DCl0_CT':inputs["data:aerodynamics:aircraft:takeoff:CT_effect:DCL0"],
                'DCl_alpha_CT':inputs["data:aerodynamics:aircraft:takeoff:CT_effect:DCL_alpha"],
                'DCd_OEI':inputs["data:aerodynamics:aircraft:takeoff:OEI_effect:DCD"],
                'DCd_gd':inputs["data:aerodynamics:aircraft:takeoff:ground_effect:DCD"],
                'DCl_gd':inputs["data:aerodynamics:aircraft:takeoff:ground_effect:DCL"],
                'K_H' :inputs["data:aerodynamics:aircraft:takeoff:ground_effect:K_H"],
                'DCd_lg':inputs["data:aerodynamics:aircraft:takeoff:lg_effect:DCD"],
                'DCl_lg':inputs["data:aerodynamics:aircraft:takeoff:lg_effect:DCL"],
            'kf':inputs["data:mission:sizing:takeoff:Friction_coefficient_no_brake"],
            'kf_brake':inputs["data:mission:sizing:takeoff:Friction_coefficient_brake"],
        }        

        
      

        take_off_polar = Polar(
            inputs["data:aerodynamics:aircraft:takeoff:CL"],
            inputs["data:aerodynamics:aircraft:takeoff:CD"],
        )      
      
        
        
        

 
        # Take off segment =====================================================
        alpha_initial = -1./180*np.pi
        initial_takeoff = FlightPoint(
            mass=TOW_input-inputs["data:mission:"+mission_type+":taxi_out:fuel"],
            true_airspeed=0.001,
            altitude=inputs["data:mission:"+mission_type+":takeoff:altitude"],
            ground_distance=0.0,
            alpha=alpha_initial,
            EM_power_rate = inputs["data:mission:"+mission_type+":takeoff:EM_power_rate"],
            TP_power_rate = inputs["data:mission:"+mission_type+":takeoff:TP_power_rate"],
            DISA = inputs["data:mission:"+mission_type+":takeoff:DISA"]
        )
        
        def TOD():
            # print('start TOD')
            #TOargs['vef']= vef
        
            take_off_calculator = TakeOffSpeeds(
                **TOargs,
                polar=take_off_polar,
                reference_area=reference_area,
                propulsion=propulsion_model,
                thrust_rate=inputs["data:mission:sizing:takeoff:thrust_rate"],
                name="take off",
                time_step=0.01,
                EM_power_rate = inputs["data:mission:"+mission_type+":takeoff:EM_power_rate"],
                TP_power_rate = inputs["data:mission:"+mission_type+":takeoff:TP_power_rate"],
                # vr = 1.05*inputs["data:mission:"+mission_type+":takeoff:V_MCA"],
                # vlo =1.1* inputs["data:mission:"+mission_type+":takeoff:V_MCA"] ,   
                
            )        
            
            take_off_flight_points,vr,vlo,v2 = take_off_calculator.compute_from(initial_takeoff)
            
            
            # print('     Take off completed')
            try:
                delta_time=[take_off_flight_points.time.iloc[1]-take_off_flight_points.time.iloc[0]] +[take_off_flight_points.time.iloc[i+1]-take_off_flight_points.time.iloc[i] for i in range(len(take_off_flight_points)-1)]
                take_off_flight_points['delta_time']=np.array(delta_time,dtype=object)
                fuel_delta_mass = take_off_flight_points.psfc * take_off_flight_points.TPshaft_power *take_off_flight_points.delta_time
                take_off_flight_points['fuel_mass']= fuel_delta_mass.cumsum()      
        
                end_of_takeoff = FlightPoint(take_off_flight_points.loc[take_off_flight_points.name == "take off"].iloc[-1])
                
            except:
                end_of_takeoff=initial_takeoff
            # outputs["data:mission:"+mission_type+":takeoff:fuel"] = (
            #     end_of_takeoff.fuel_mass 
            # ) 
            
             
            outputs["data:mission:"+mission_type+":takeoff:distance"] = end_of_takeoff.ground_distance 
            # outputs["data:mission:"+mission_type+":takeoff:V_rotate"] = (
            #     FlightPoint(take_off_flight_points.loc[take_off_flight_points.alpha == alpha_initial].iloc[-1]).true_airspeed 
            # )         
            # outputs["data:mission:"+mission_type+":takeoff:V_liftoff"] = (
            #     FlightPoint(take_off_flight_points.loc[take_off_flight_points.altitude == float(inputs["data:mission:"+mission_type+":takeoff:altitude"])].iloc[-1]).true_airspeed
            # )     
            # outputs["data:mission:"+mission_type+":takeoff:V_2"] = (
            #     end_of_takeoff.true_airspeed 
            # )
            
            outputs["data:mission:"+mission_type+":takeoff:V_rotate"] = vr
            outputs["data:mission:"+mission_type+":takeoff:V_liftoff"] = vlo
            outputs["data:mission:"+mission_type+":takeoff:V_2"] = v2
            
            return take_off_flight_points,float(end_of_takeoff.ground_distance),float(outputs["data:mission:"+mission_type+":takeoff:V_rotate"])

        def TODn1(vef):
            # print('start TODn1',vef)
            TOargs['vef']= vef
        
            take_off_calculator = TakeOffSegment(
                **TOargs,
                polar=take_off_polar,
                reference_area=reference_area,
                propulsion=propulsion_model,
                thrust_rate=inputs["data:mission:sizing:takeoff:thrust_rate"],
                name="take off",
                time_step=0.01,
                EM_power_rate = inputs["data:mission:"+mission_type+":takeoff:EM_power_rate"],
                TP_power_rate = inputs["data:mission:"+mission_type+":takeoff:TP_power_rate"],
                vr =outputs["data:mission:"+mission_type+":takeoff:V_rotate"],
                vlo =outputs["data:mission:"+mission_type+":takeoff:V_liftoff"] ,                
            )        
            
            take_off_flight_points = take_off_calculator.compute_from(initial_takeoff)
            
            
            # print('     Take off completed')
            
            delta_time=[take_off_flight_points.time.iloc[1]-take_off_flight_points.time.iloc[0]] +[take_off_flight_points.time.iloc[i+1]-take_off_flight_points.time.iloc[i] for i in range(len(take_off_flight_points)-1)]
            take_off_flight_points['delta_time']=np.array(delta_time,dtype=object)
            fuel_delta_mass = take_off_flight_points.psfc * take_off_flight_points.TPshaft_power *take_off_flight_points.delta_time
            take_off_flight_points['fuel_mass']= fuel_delta_mass.cumsum()      
    
            end_of_takeoff = FlightPoint(take_off_flight_points.loc[take_off_flight_points.name == "take off"].iloc[-1])
            # print('TODn-1',end_of_takeoff.ground_distance )

            return take_off_flight_points,float(end_of_takeoff.ground_distance),float(outputs["data:mission:"+mission_type+":takeoff:V_rotate"])
        
        def ASD(vef):
            # print('start ASD',vef)
            # if vef> outputs["data:mission:"+mission_type+":takeoff:V_rotate"]:
            #     vef=outputs["data:mission:"+mission_type+":takeoff:V_rotate"]
            TOargs['vef']= float(vef)
            
            ASD_calculator = ASDSegment(
            **TOargs,
            polar=take_off_polar,
            reference_area=reference_area,
            propulsion=propulsion_model,
            thrust_rate=inputs["data:mission:sizing:takeoff:thrust_rate"],
            name="ASD",
            time_step=0.01,
            EM_power_rate=inputs["data:mission:"+mission_type+":takeoff:EM_power_rate"],
            TP_power_rate =inputs["data:mission:"+mission_type+":takeoff:TP_power_rate"],  
            )        
            
            ASD_flight_points = ASD_calculator.compute_from(initial_takeoff)
            # print('     ASD completed')
            # flight_point_vef = ASD_flight_points.loc[ASD_flight_points.equivalent_airspeed == TOargs['vef']]
            time_v1 = float(ASD_flight_points[ASD_flight_points.engine_setting == 8]['time'].iloc[0]+1)
            flight_point_v1 = ASD_flight_points.loc[ASD_flight_points.time >= time_v1].iloc[0]
            end_of_ASD = FlightPoint(ASD_flight_points.loc[ASD_flight_points.name == "ASD"].iloc[-1])
            ASD = end_of_ASD.ground_distance + flight_point_v1.equivalent_airspeed*2  #+2sec at v1
            # print('ASD',float(ASD) ) 
            #NB: there is no model thrust-power for flight idle, so in the braking segment the power 
            #    calculated is not to be taken into account. Anyway, only the thrust has an impact on the performance!
            #    (segment has thrust_rate)
            
            return ASD_flight_points,float(ASD)
        
        take_off_flight_points,tod_n,vr = TOD() #TODn
        print('TODn = ', tod_n,'1.15*TODn = ', 1.15*tod_n )
        print('VR = ',vr)
        # for vef in [outputs["data:mission:"+mission_type+":takeoff:V_rotate"] ]:
        #     take_off_flight_points,tod,vr = TOD(vef)
        #     ASD_flight_points,asd = ASD(vef)
        #     print('---------new vef----------')
        #     #print(vef,tod,asd)
        
        if vr>0:
        
            def func(vef):
                take_off_flight_points,tod,vr = TODn1(vef)
                ASD_flight_points,asd = ASD(vef) 
                return tod-asd
                
            vef,info,ier,msg = fsolve(func, vr, full_output=True,xtol=10e-4)#,epsfcn=10e2)
            # print('convergence',vef,info,ier,msg )
    
            take_off_flight_points,tod_n1,vr = TODn1(vef)    
            print('TODn-1 = ',tod_n1)
            ASD_flight_points,asd_n1 = ASD(vef)
            print('ASDn-1 = ', asd_n1)
            time_v1 = float(ASD_flight_points[ASD_flight_points.engine_setting == 8]['time'].iloc[0]+1)
            flight_point_v1 =ASD_flight_points.loc[ASD_flight_points.time >= time_v1].iloc[0]
            print('V1 = ', flight_point_v1.equivalent_airspeed[0], 'V_EF = ', vef)
            
            BFL=tod_n1 #or asd is the same
            
            if flight_point_v1.equivalent_airspeed[0]>vr:
                vef = vr- (flight_point_v1.equivalent_airspeed[0] - vef) #vef for which v1=vr (approximately)
                take_off_flight_points,tod_n1,vr = TODn1(vef)    
                print('TODn-1 = ',tod_n1)
                ASD_flight_points,asd_n1 = ASD(vef)
                print('ASDn-1 = ', asd_n1)
                BFL= max([float(tod_n),float(tod_n1),float(asd_n1)])
                print('V1>VR')
                v1=vr
            else:
                print('V1<VR')
                BFL=BFL
                v1=flight_point_v1.equivalent_airspeed[0]
            print(BFL)
        
        else:
            Tot_av_power = propulsion_model.engine.Elec_nom_power *inputs["data:mission:"+mission_type+":takeoff:EM_power_rate"]+propulsion_model.engine.RTO_power*inputs["data:mission:"+mission_type+":takeoff:TP_power_rate"]
            SHP_TOs=np.array([0.85,0.9,0.95,1,1.05,1.1,1.15,1.2])*1801000
            TOFLs=[ 2509,1712,1501,1384,1294,1212,1164,1162]
            
            from scipy import interpolate
            
            f = interpolate.interp1d(SHP_TOs, TOFLs,kind='linear',fill_value="extrapolate")
            BFL=f(Tot_av_power)
            vef=0
            v1=0
            ASD_flight_points=take_off_flight_points
            print('TO_power ',Tot_av_power/1000, 'BFL ',BFL )
            
            
        outputs["data:mission:"+mission_type+":takeoff:V_EF"] = vef
        outputs["data:mission:"+mission_type+":takeoff:V_1"] = v1
        outputs["data:mission:sizing:takeoff:TOFL"] = BFL
        
        # Final ================================================================
        try:
            if 'L1' in self.options["propulsion_id"] and 'PH' in self.options["propulsion_id"] :            
    
                self.flight_points = (
                    pd.concat(
                        [  take_off_flight_points.drop(columns=['TP_total_temperature','TP_total_pressure']),
                           ASD_flight_points.drop(columns=['TP_total_temperature','TP_total_pressure']),
                          ]
                    )
                    .reset_index(drop=True)
                    .applymap(lambda x: np.asscalar(np.asarray(x)))
                )
        
                thermo_data= (
                    pd.concat(
                        [ take_off_flight_points[['TP_total_temperature','TP_total_pressure']],
                         ASD_flight_points[['TP_total_temperature','TP_total_pressure']],
    
                        ]
                    )
                    .reset_index(drop=True)
                )
                if self.options["out_file"]:
                    self.flight_points.to_csv(self.options["out_file"])
                    thermo_data.to_csv(self.options["out_file"][:-4]+'_thermo.csv')
                    
            else:   
    
                self.flight_points = (
                    pd.concat(
                        [  take_off_flight_points,
                         ASD_flight_points
    
                        ]
                    )
                    .reset_index(drop=True)
                    .applymap(lambda x: np.asscalar(np.asarray(x)))
                )
        
                if self.options["out_file"]:
                    self.flight_points.to_csv(self.options["out_file"])
        except:
            pass
        
        
