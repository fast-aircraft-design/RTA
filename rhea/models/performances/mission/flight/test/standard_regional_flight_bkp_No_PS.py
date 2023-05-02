"""Definition of the standard flight missions."""
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

from typing import Dict, List, Union

from fastoad.constants import FlightPhase, EngineSetting
from fastoad.models.propulsion import IPropulsion
from scipy.constants import foot, knot,psi

from fastoad.models.performances.mission.flight.base import AbstractSimpleFlight
from fastoad.models.performances.mission.base import IFlightPart, AbstractManualThrustFlightPhase
#from fastoad.models.performances.mission.flight_point import FlightPoint #v0.50a
from fastoad.base.flight_point import FlightPoint #v0.5.2b
from fastoad.models.performances.mission.polar import Polar
from fastoad.models.performances.mission.segments.altitude_change import AltitudeChangeSegment
from fastoad.models.performances.mission.segments.cruise import CruiseSegment
from fastoad.models.performances.mission.segments.speed_change import SpeedChangeSegment
from rhea.models.performances.mission.segments.speed_change_cruise import SpeedChangeCruise
from rhea.models.performances.mission.segments.descent_fixed_slope import FixedSlopeDescent

import numpy as np
from scipy.optimize import root_scalar
from fastoad.utils.physics import AtmosphereSI

class InitialClimbPhase1(AbstractManualThrustFlightPhase):
    """
    Preset for initial climb phase.

    - Climbs up to 400ft at constant EAS
    - Accelerates to EAS =climb_speed at constant altitude
    - Climbs up to 1500ft at constant EAS
    """
    def __init__(self, **kwargs):
        self.climb_speed = kwargs.pop("climb_speed")
        super().__init__(**kwargs)        
    @property
    def flight_sequence(self) -> List[Union[IFlightPart, str]]:
        return [
            AltitudeChangeSegment(
                target=FlightPoint(equivalent_airspeed="constant", altitude=400.0 * foot),
                engine_setting=EngineSetting.TAKEOFF,
                **self.segment_kwargs,
            ),
            SpeedChangeSegment(
                target=FlightPoint(equivalent_airspeed=self.climb_speed[0]),
                engine_setting=EngineSetting.CLIMB,
                **self.segment_kwargs,
            ),
            AltitudeChangeSegment(
                target=FlightPoint(equivalent_airspeed="constant", altitude=1500.0 * foot),
                engine_setting=EngineSetting.CLIMB,
                **self.segment_kwargs,
            ),
        ]

class InitialClimbPhase2(AbstractManualThrustFlightPhase):
    """
    Preset for initial climb phase. 

    - Climbs up to 1500ft at constant EAS @TO setting
    """
    def __init__(self, **kwargs):
        self.climb_speed = kwargs.pop("climb_speed")
        super().__init__(**kwargs)        
    @property
    def flight_sequence(self) -> List[Union[IFlightPart, str]]:
        return [
            AltitudeChangeSegment(
                target=FlightPoint(equivalent_airspeed="constant", altitude=1500.0 * foot),
                engine_setting=EngineSetting.TAKEOFF,
                **self.segment_kwargs,
            ),
            

        ]


class ClimbPhaseICAOA(AbstractManualThrustFlightPhase):
    """
    Preset for climb phase.Filippone pag.282

    - Climbs up to 3000ft at constant EAS @MCL setting
    - Accelerates to EAS =climb_speed at constant altitude
    - Climbs up to target altitude at constant EAS
    - Accelerate to target cruise mach
    """

    def __init__(self, **kwargs):
        """
        Uses keyword arguments as for :meth:`AbstractManualThrustFlightPhase` with
        these additional keywords:

        :param maximum_mach: Mach number that won't be exceeded during climb
        :param target_altitude: target altitude in meters, can be a float or
                                AltitudeChangeSegment.OPTIMAL_ALTITUDE to target
                                altitude with maximum lift/drag ratio
        """

        # if "maximum_mach" in kwargs:
        #     self.maximum_mach = kwargs.pop("maximum_mach")
        self.target_mach = kwargs.pop("target_mach")
        self.target_altitude = kwargs.pop("target_altitude")
        self.climb_speed = kwargs.pop("climb_speed")
        super().__init__(**kwargs)

    @property
    def flight_sequence(self) -> List[Union[IFlightPart, str]]:
        self.segment_kwargs["engine_setting"] = EngineSetting.CLIMB

        return [
            AltitudeChangeSegment(
                target=FlightPoint(equivalent_airspeed="constant", altitude=3000.0 * foot),
                **self.segment_kwargs,
            ),
            SpeedChangeSegment(
                target=FlightPoint(equivalent_airspeed=self.climb_speed[0] ), **self.segment_kwargs,
            ),
            AltitudeChangeSegment(
                target=FlightPoint(equivalent_airspeed="constant", altitude=self.target_altitude),
                **self.segment_kwargs,           
                maximum_mach=self.target_mach,
            ), 

            SpeedChangeCruise(
                target=FlightPoint(mach=self.target_mach[0]), **self.segment_kwargs,
                       
            ),            
        ]
    
class ClimbPhase(AbstractManualThrustFlightPhase):
    """
    Preset for climb phase.


    - Climbs up to target altitude at constant EAS
    """

    def __init__(self, **kwargs):
        """
        Uses keyword arguments as for :meth:`AbstractManualThrustFlightPhase` with
        these additional keywords:

        :param maximum_mach: Mach number that won't be exceeded during climb
        :param target_altitude: target altitude in meters, can be a float or
                                AltitudeChangeSegment.OPTIMAL_ALTITUDE to target
                                altitude with maximum lift/drag ratio
        """

        # if "maximum_mach" in kwargs:
        #     self.maximum_mach = kwargs.pop("maximum_mach")
        self.target_mach = kwargs.pop("target_mach")
        self.target_altitude = kwargs.pop("target_altitude")
        self.climb_speed = kwargs.pop("climb_speed")
        super().__init__(**kwargs)

    @property
    def flight_sequence(self) -> List[Union[IFlightPart, str]]:
        self.segment_kwargs["engine_setting"] = EngineSetting.CLIMB

        return [

            AltitudeChangeSegment(
                target=FlightPoint(equivalent_airspeed="constant", altitude=self.target_altitude),
                **self.segment_kwargs,           
                maximum_mach=self.target_mach,
            ), 
            SpeedChangeCruise(
                target=FlightPoint(mach=self.target_mach[0]), **self.segment_kwargs,
                       
            ),
        ]
class DivClimbPhase(AbstractManualThrustFlightPhase):
    """
    Preset for climb phase.


    - Climbs up to target altitude at constant EAS
    """

    def __init__(self, **kwargs):
        """
        Uses keyword arguments as for :meth:`AbstractManualThrustFlightPhase` with
        these additional keywords:

        :param maximum_mach: Mach number that won't be exceeded during climb
        :param target_altitude: target altitude in meters, can be a float or
                                AltitudeChangeSegment.OPTIMAL_ALTITUDE to target
                                altitude with maximum lift/drag ratio
        """

        # if "maximum_mach" in kwargs:
        #     self.maximum_mach = kwargs.pop("maximum_mach")
        self.target_mach = kwargs.pop("target_mach")
        self.target_altitude = kwargs.pop("target_altitude")
        self.climb_speed = kwargs.pop("climb_speed")
        super().__init__(**kwargs)

    @property
    def flight_sequence(self) -> List[Union[IFlightPart, str]]:
        self.segment_kwargs["engine_setting"] = EngineSetting.CLIMB

        return [
        

            AltitudeChangeSegment(
                target=FlightPoint(equivalent_airspeed="constant", altitude=self.target_altitude),
                **self.segment_kwargs,           
                maximum_mach=self.target_mach,
            ), 
            # SpeedChangeSegment(
            #     target=FlightPoint(mach=self.target_mach[0]), **self.segment_kwargs,
                       
            # ),
            SpeedChangeCruise(
                target=FlightPoint(mach=self.target_mach[0]), **self.segment_kwargs,
                       
            ),            
        ]    
    
class AccelerationPhase(AbstractManualThrustFlightPhase):
    """
    Preset for AccelerationPhase phase.
    - Change speed to EAS = target_speed at constant Mach
    """

    def __init__(self, **kwargs):
        """
        Uses keyword arguments as for :meth:`AbstractManualThrustFlightPhase` with
        this additional keyword:

        :param target_altitude: target altitude in meters
        """

        self.descent_speed = kwargs.pop("target_speed")

        super().__init__(**kwargs)

    # def compute_from(self, start: FlightPoint) -> pd.DataFrame:
    #     self.find_min_descent_time(start)
    #     self.segment_kwargs['name']='take off'
    #     flight_points= super().compute_from(start) 
    #     return flight_points

    @property
    def flight_sequence(self) -> List[Union[IFlightPart, str]]:
        return [
            AltitudeChangeSegment(
                target=FlightPoint(equivalent_airspeed=self.descent_speed[0], mach="constant"),
                engine_setting=EngineSetting.CRUISE,
                **self.segment_kwargs,
                ),
        ]
    # def find_min_descent_time(self,start):
    #     Zp = start.altitude
    #     max_delta_pressure=  6.*psi
    #     Pc_target= start.pressure + max_delta_pressure
    #     def func(alt):
    #         Pc = AtmosphereSI(alt).pressure
    #         return Pc-Pc_target
    #     Zc = root_scalar(func,x0=Zp/2)
    #     max_cabin_rd = 500*foot/60 #m/s =500ft/min
    #     TTD_min = Zc/max_cabin_rd
            
    
class DescentPhase(AbstractManualThrustFlightPhase):
    """
    Preset for descent phase.

    - Change speed to EAS = descent_speed at constant Mach
    - Descends down to target altitude at constant EAS
    """

    def __init__(self, **kwargs):
        """
        Uses keyword arguments as for :meth:`AbstractManualThrustFlightPhase` with
        this additional keyword:

        :param target_altitude: target altitude in meters
        """

        self.target_altitude = kwargs.pop("target_altitude")
        self.descent_speed = kwargs.pop("descent_speed")

        super().__init__(**kwargs)

    @property
    def flight_sequence(self) -> List[Union[IFlightPart, str]]:
        #self.segment_kwargs["engine_setting"] = EngineSetting.IDLE
        return [

            # AltitudeChangeSegment(
            #     target=FlightPoint(altitude=self.target_altitude, equivalent_airspeed="constant"),
            #     engine_setting=EngineSetting.IDLE,
            #     **self.segment_kwargs,
            # ),
            # FixedSlopeDescent(
            #     target=FlightPoint(equivalent_airspeed=self.descent_speed[0], mach="constant"),
            #     #engine_setting=EngineSetting.CRUISE,
            #     **self.segment_kwargs,
            #     ),

            FixedSlopeDescent(
                target=FlightPoint(altitude=self.target_altitude, equivalent_airspeed="constant"),
                engine_setting=EngineSetting.CRUISE,
                **self.segment_kwargs,
            ),
        ]


class StandardFlight(AbstractSimpleFlight):
    """
    Defines and computes a standard flight mission.

    The flight sequence is:
    - initial climb
    - climb
    - cruise at constant altitude
    - descent
    """

    def __init__(
        self,
        propulsion: IPropulsion,
        reference_area: float,
        low_speed_climb_polar: Polar,
        high_speed_polar: Polar,
        cruise_mach: float,
        thrust_rates: Dict[FlightPhase, float],
        climb_speed: float= 150.*knot,
        descent_speed: float= 180.*knot,
        cruise_distance: float = 0.0,
        climb_target_altitude: float =  AltitudeChangeSegment.OPTIMAL_FLIGHT_LEVEL,
        descent_target_altitude: float = 1500.0 * foot,
        time_step=None,

    ):
        """

        :param propulsion:
        :param reference_area:
        :param low_speed_climb_polar:
        :param high_speed_polar:
        :param cruise_mach:
        :param thrust_rates:
        :param cruise_distance:
        :param climb_target_altitude: (in m) altitude where cruise will begin. If value is
                                      AltitudeChangeSegment.OPTIMAL_ALTITUDE (default), climb will
                                      stop when maximum lift/drag ratio is achieved. Cruise will go
                                      on at the same altitude.
        :param descent_target_altitude: (in m) altitude where descent will end in meters
                                        Default is 457.2m (1500ft)
        :param time_step: if provided, this time step will be applied for all segments.
        """

        self.flight_phase_kwargs = {
            "propulsion": propulsion,
            "reference_area": reference_area,
            "time_step": time_step,
            
        }

        self.low_speed_climb_polar = low_speed_climb_polar
        self.high_speed_polar = high_speed_polar
        self.cruise_mach = cruise_mach
        self.thrust_rates = thrust_rates
        self.climb_target_altitude = climb_target_altitude
        self.descent_target_altitude = descent_target_altitude
        self.time_step = time_step
        self.climb_speed=climb_speed
        self.descent_speed=descent_speed
        
        

        kwargs = {
            "propulsion": propulsion,
            "reference_area": reference_area,
            "time_step": time_step,
            #qui aggiungi tutte le variabili input di cui hai bisogno per TO
        }
               
        initial_climb = InitialClimbPhase1(
            **kwargs,
            polar=low_speed_climb_polar, 
            thrust_rate=1.0,
            name="initial climb",
            climb_speed=self.climb_speed,
        )
        climb = ClimbPhase(
            **kwargs,
            polar=high_speed_polar,
            thrust_rate=thrust_rates[FlightPhase.CLIMB],
            target_altitude=self.climb_target_altitude,
            # maximum_mach=np.array([50]),
            target_mach=self.cruise_mach,
            name="climb",
            climb_speed=self.climb_speed,           
        )
        cruise = CruiseSegment(
            **kwargs,
            target=FlightPoint(),
            polar=high_speed_polar,
            engine_setting=EngineSetting.CRUISE,
            name="cruise",
        )
        acceleration = AccelerationPhase(
            **kwargs,
            polar=high_speed_polar,
            thrust_rate=thrust_rates[FlightPhase.DESCENT], #0.3 for ref
            name="acceleration descent",
            target_speed=self.descent_speed,
        )
        descent = DescentPhase(
            **kwargs,
            polar=high_speed_polar,
            thrust_rate=thrust_rates[FlightPhase.DESCENT],
            target_altitude=self.descent_target_altitude,
            name="descent",
            descent_speed=self.descent_speed,
        )
        super().__init__(
            cruise_distance, [initial_climb, climb], cruise, [acceleration,descent], #[descent], # [acceleration,descent], #
        )

class DiversionFlight(AbstractSimpleFlight):
    """
    Defines and computes a standard flight mission.

    The flight sequence is:
    - initial climb
    - climb
    - cruise at constant altitude
    - descent
    """

    def __init__(
        self,
        propulsion: IPropulsion,
        reference_area: float,
        low_speed_climb_polar: Polar,
        high_speed_polar: Polar,
        cruise_mach: float,
        thrust_rates: Dict[FlightPhase, float],
        climb_speed: float= 150.*knot,
        descent_speed: float= 180.*knot,
        cruise_distance: float = 0.0,
        climb_target_altitude: float =  AltitudeChangeSegment.OPTIMAL_FLIGHT_LEVEL,
        descent_target_altitude: float = 1500.0 * foot,
        time_step=None,

    ):
        """

        :param propulsion:
        :param reference_area:
        :param low_speed_climb_polar:
        :param high_speed_polar:
        :param cruise_mach:
        :param thrust_rates:
        :param cruise_distance:
        :param climb_target_altitude: (in m) altitude where cruise will begin. If value is
                                      AltitudeChangeSegment.OPTIMAL_ALTITUDE (default), climb will
                                      stop when maximum lift/drag ratio is achieved. Cruise will go
                                      on at the same altitude.
        :param descent_target_altitude: (in m) altitude where descent will end in meters
                                        Default is 457.2m (1500ft)
        :param time_step: if provided, this time step will be applied for all segments.
        """

        self.flight_phase_kwargs = {
            "propulsion": propulsion,
            "reference_area": reference_area,
            "time_step": time_step,
        }

        self.low_speed_climb_polar = low_speed_climb_polar
        self.high_speed_polar = high_speed_polar
        self.cruise_mach = cruise_mach
        self.thrust_rates = thrust_rates
        self.climb_target_altitude = climb_target_altitude
        self.descent_target_altitude = descent_target_altitude
        self.time_step = time_step
        self.climb_speed=climb_speed
        self.descent_speed=descent_speed
        
        

        kwargs = {
            "propulsion": propulsion,
            "reference_area": reference_area,
            "time_step": time_step,
        }


        climb = DivClimbPhase(
            **kwargs,
            polar=high_speed_polar,
            thrust_rate=thrust_rates[FlightPhase.CLIMB],
            target_altitude=self.climb_target_altitude,
            target_mach=self.cruise_mach,
            name="diversion climb",
            climb_speed=self.climb_speed,
            
        )
        cruise = CruiseSegment(
            **kwargs,
            target=FlightPoint(),
            polar=high_speed_polar,
            engine_setting=EngineSetting.CRUISE,
            name="diversion cruise",
        )
        descent = DescentPhase(
            **kwargs,
            polar=high_speed_polar,
            thrust_rate=thrust_rates[FlightPhase.DESCENT],
            target_altitude=self.descent_target_altitude,
            name="diversion descent",
            descent_speed=self.descent_speed,
        )
        # super().__init__(
        #     cruise_distance, [initial_climb, climb], cruise, [descent],
        # )
        super().__init__(
            cruise_distance,[climb], cruise, [descent],
        )