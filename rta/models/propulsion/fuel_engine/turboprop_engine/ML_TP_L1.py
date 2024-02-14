"""Parametric turboprop engine."""
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
import joblib
from sklearn.preprocessing import PolynomialFeatures
from scipy import constants
import logging
import math
from typing import Union, Sequence, Tuple, Optional
import os
import numpy as np
from fastoad.constants import FlightPhase
from fastoad.module_management.service_registry import RegisterPropulsion

# from fastoad.module_management.constants import ModelDomain
# from fastoad.models.propulsion import IPropulsion
# from models.propulsion import IPropulsion
from fastoad_cs25.models.propulsion.fuel_propulsion.rubber_engine.exceptions import (
    FastRubberEngineInconsistentInputParametersError,
)
from fastoad.model_base.atmosphere import Atmosphere
from fastoad.model_base.propulsion import IOMPropulsionWrapper
import pandas as pd
from fastoad.model_base.flight_point import FlightPoint
from .base import AbstractFuelPropulsion

from scipy.optimize import fsolve

# Logger for this module
_LOGGER = logging.getLogger(__name__)
# from fastoad.base.dict import AddKeyAttributes
from .engine_components.Propeller import Propeller

# AddKeyAttributes(["psfc","shaft_power", "power_rate","thermo_power","TP_thermal_efficiency","TP_residual_thrust","TP_air_flow","TP_total_pressure","TP_total_temperature","fuel_mass"
#  ,"H2_mass","TPshaft_power","EMshaft_power","FC_power","TP_power_rate","EM_power_rate","H2_fc","CT"])(FlightPoint)


script_path = os.path.abspath(__file__)  # i.e. /path/to/dir/foobar.py
rhea_path = script_path.split("models")[0]
RHEA_path = script_path.split("rta")[0]


class ML_TP_L1(AbstractFuelPropulsion):
    def __init__(
        self,
        RTO_power: float,
        Design_Thermo_Power: float,
        Power_Offtake: float,
        gearbox_eta: float,
        d_prop: float,
        k_gb_RTO: float,
        k_gb_NTO: float,
        k_gb_MCL: float,
        k_gb_MCT: float,
        k_gb_MCR: float,
        k_gb_FID: float,
        k_psfc: float,
        k_prop: float,
    ):
        """
        Parametric turboprop engine.

        It computes engine characteristics using analytical model from following
        sources:

         [1] F. Stagliano and H. Lobentanzer, "Impact of Novel Propulsion System Architectures Incorporating Diesel Engines on Mission Fuel Burn for a Tilt-Wing Transport Aircraft," in 12th AIAA Aviation Technology, Integration, and Operations (ATIO) Conference, Indianapolis, Indiana, 2012. AIAA 2012-5587.
         [2] Stückl, S. (2016). Methods for the Design and Evaluation of Future Aircraft Concepts Utilizing Electric Propulsion Systems (Doctoral dissertation, Technische Universität München).


        :param bypass_ratio:
        :param overall_pressure_ratio:
        :param turbine_inlet_temperature: (unit=K) also noted T4
        :param mto_thrust: (unit=N) Maximum TakeOff thrust, i.e. maximum thrust
                           on ground at speed 0, also noted F0
        :param maximum_mach:
        :param design_altitude: (unit=m)
        :param delta_t4_climb: (unit=K) difference between T4 during climb and design T4
        :param delta_t4_cruise: (unit=K) difference between T4 during cruise and design T4
        """
        # pylint: disable=too-many-arguments  # they define the engine

        self.RTO_power = RTO_power
        self.Design_Thermo_Power = Design_Thermo_Power
        self.Power_Offtake = Power_Offtake
        self.gearbox_eta = gearbox_eta
        self.d_prop = d_prop

        self.k_gb_RTO = k_gb_RTO
        self.k_gb_NTO = k_gb_NTO
        self.k_gb_MCL = k_gb_MCL
        self.k_gb_MCT = k_gb_MCT
        self.k_gb_MCR = k_gb_MCR
        self.k_gb_FID = k_gb_FID

        self.k_psfc = k_psfc
        self.k_prop = k_prop

    def compute_flight_points(self, flight_points: Union[FlightPoint, pd.DataFrame]):
        # pylint: disable=too-many-arguments  # they define the trajectory

        """
        Same as :meth:`compute_flight_points` except that delta_t4 is used directly
        instead of specifying flight phase.

        :param mach: Mach number
        :param altitude: (unit=m) altitude w.r.t. to sea level
        :param delta_t4: (unit=K) difference between operational and design values of
                         turbine inlet temperature in K
        :param use_thrust_rate: tells if thrust_rate or thrust should be used (works element-wise)
        :param thrust_rate: thrust rate (unit=none)
        :param thrust: required thrust (unit=N)
        :return: SFC (in kg/s/N), thrust rate, thrust (in N)
        """

        Prop_fid = "ADT"
        mach = np.asarray(flight_points.mach)
        altitude = np.asarray(flight_points.altitude)
        thrust_rate = np.asarray(flight_points.thrust_rate)
        thrust = np.asarray(flight_points.thrust)
        phase = flight_points.engine_setting
        thrust_is_regulated = flight_points.thrust_is_regulated
        atmosphere = Atmosphere(altitude, altitude_in_feet=False)
        a = atmosphere.speed_of_sound
        V_TAS = mach * a
        # V_EAS = atmosphere.get_equivalent_airspeed(V_TAS)/constants.knot
        # print(altitude,mach,V_TAS,phase)
        # if mach ==0:
        #     mach = 0.025

        if thrust_is_regulated is not None:
            thrust_is_regulated = np.asarray(
                np.round(thrust_is_regulated, 0), dtype=bool
            )

        thrust_is_regulated, thrust_rate, thrust = self._check_thrust_inputs(
            thrust_is_regulated, thrust_rate, thrust
        )
        thrust_is_regulated = np.asarray(np.round(thrust_is_regulated, 0), dtype=bool)
        max_shaft_power, max_thermo_power, gearbox_limit_power = self.max_power(
            atmosphere, mach, phase
        )

        # FN,FR =self.power_to_thrust_ML(atmosphere, mach,phase,max_shaft_power)
        T_prop, eta = Propeller().select(
            "power_to_thrust", Prop_fid, self, atmosphere, mach, phase, max_shaft_power
        )
        FR = self.compute_engine_point(atmosphere, mach, phase, T_prop=T_prop)

        # FR=0.
        max_thrust = T_prop + FR

        if not thrust_is_regulated:
            if thrust_rate == 1:
                power_rate = 1
                shaft_power = max_shaft_power
                thrust = thrust_rate * max_thrust
            else:
                # print(phase,altitude,mach)
                thrust = thrust_rate * max_thrust
                # shaft_power,FR = self.thrust_to_power_ML(atmosphere, mach,phase,thrust)
                shaft_power, eta = Propeller().select(
                    "thrust_to_power", Prop_fid, self, atmosphere, mach, phase, thrust
                )
                # T_prop=shaft_power*self.gearbox_eta*eta/V_TAS
                # FR=thrust-T_prop
                power_rate = shaft_power / max_shaft_power
        else:
            power_rate = thrust_rate
            # shaft_power,FR=self.thrust_to_power_ML(atmosphere, mach,phase,thrust)
            shaft_power, eta = Propeller().select(
                "thrust_to_power", Prop_fid, self, atmosphere, mach, phase, thrust
            )
            # T_prop=shaft_power*self.gearbox_eta*eta/V_TAS
            # FR=thrust-T_prop

        thrust_rate = np.asarray(thrust_rate)
        thrust = np.asarray(thrust)

        shaft_power = np.asarray(shaft_power)
        power_rate = np.asarray(power_rate)

        # We compute thrust values from thrust rates when needed
        idx = np.logical_not(thrust_is_regulated)
        if np.size(max_thrust) == 1:  # se scalare
            maximum_thrust = max_thrust
            maximum_shaft_power = max_shaft_power
            out_thrust_rate = thrust_rate
            out_thrust = thrust
            out_power_rate = power_rate
            out_power = shaft_power
        else:  # se vettore
            out_thrust_rate = (
                np.full(np.shape(max_thrust), thrust_rate.item())
                if np.size(thrust_rate) == 1
                else thrust_rate
            )
            out_power_rate = (
                np.full(np.shape(max_shaft_power), power_rate.item())
                if np.size(power_rate) == 1
                else power_rate
            )
            out_thrust = (
                np.full(np.shape(max_thrust), thrust.item())
                if np.size(thrust) == 1
                else thrust
            )
            out_power = (
                np.full(np.shape(max_shaft_power), shaft_power.item())
                if np.size(shaft_power) == 1
                else shaft_power
            )

            maximum_thrust = max_thrust[idx]
            maximum_shaft_power = max_shaft_power[idx]

        out_thrust[idx] = out_thrust_rate[idx] * maximum_thrust
        out_power[idx] = out_power_rate[idx] * maximum_shaft_power

        # thrust_rate is obtained from entire thrust vector (could be optimized if needed,
        # as some thrust rates that are computed may have been provided as input)
        out_thrust_rate = out_thrust / max_thrust
        out_power_rate = out_power / max_shaft_power

        # if phase.value==3:
        #     print(phase)

        # Now SFC can be computed
        # psfc_0 =self.sfc_at_max_power() #lb/hp/hr
        psfc = (
            self.psfc(atmosphere, mach, out_power_rate, phase) * self.k_psfc
        )  # kg/hp/hr
        ff = psfc / constants.hour * out_power / constants.hp  # Kg/s
        tsfc = ff / out_thrust

        # print(phase,out_power,out_thrust,out_power_rate)
        # if phase==1:
        #     RPS=1200/60
        # else:
        #     RPS=984/60

        # flight_points.CT = out_thrust/(atmosphere.density*0.5*V_TAS**2*61) #61=Sref
        # flight_points.CT = out_thrust/reference_force
        # print(out_thrust/(atmosphere.density*0.5*V_TAS**2*61),out_thrust/reference_force)
        flight_points.psfc = psfc / constants.hour / constants.hp
        flight_points.thrust_rate = out_thrust_rate
        flight_points.thrust = out_thrust
        flight_points.TPshaft_power = out_power
        flight_points.TP_power_rate = out_power_rate
        flight_points.thermo_power = max_thermo_power
        flight_points.TP_residual_thrust = FR
        flight_points.sfc = tsfc
        # return tsfc, out_thrust_rate, out_thrust,out_power,out_power_rate,max_thermo_power

    @staticmethod
    def _check_thrust_inputs(
        thrust_is_regulated: Optional[Union[float, Sequence]],
        thrust_rate: Optional[Union[float, Sequence]],
        thrust: Optional[Union[float, Sequence]],
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Checks that inputs are consistent and return them in proper shape.

        Some of the inputs can be None, but outputs will be proper numpy arrays.

        :param thrust_is_regulated:
        :param thrust_rate:
        :param thrust:
        :return: the inputs, but transformed in numpy arrays.
        """
        # Ensure they are numpy array
        if thrust_is_regulated is not None:
            # As OpenMDAO may provide floats that could be slightly different
            # from 0. or 1., a rounding operation is needed before converting
            # to booleans
            thrust_is_regulated = np.asarray(
                np.round(thrust_is_regulated, 0), dtype=bool
            )
        if thrust_rate is not None:
            thrust_rate = np.asarray(thrust_rate)
        if thrust is not None:
            thrust = np.asarray(thrust)

        # Check inputs: if use_thrust_rate is None, we will use the provided input between
        # thrust_rate and thrust
        if thrust_is_regulated is None:
            if thrust_rate is not None:
                thrust_is_regulated = False
                thrust = np.empty_like(thrust_rate)
            elif thrust is not None:
                thrust_is_regulated = True
                thrust_rate = np.empty_like(thrust)
            else:
                raise FastRubberEngineInconsistentInputParametersError(
                    "When use_thrust_rate is None, either thrust_rate or thrust should be provided."
                )

        elif np.size(thrust_is_regulated) == 1:
            # Check inputs: if use_thrust_rate is a scalar, the matching input(thrust_rate or
            # thrust) must be provided.
            if thrust_is_regulated:
                if thrust is None:
                    raise FastRubberEngineInconsistentInputParametersError(
                        "When thrust_is_regulated is True, thrust should be provided."
                    )
                thrust_rate = np.empty_like(thrust)
            else:
                if thrust_rate is None:
                    raise FastRubberEngineInconsistentInputParametersError(
                        "When thrust_is_regulated is False, thrust_rate should be provided."
                    )
                thrust = np.empty_like(thrust_rate)

        else:
            # Check inputs: if use_thrust_rate is not a scalar, both thrust_rate and thrust must be
            # provided and have the same shape as use_thrust_rate
            if thrust_rate is None or thrust is None:
                raise FastRubberEngineInconsistentInputParametersError(
                    "When thrust_is_regulated is a sequence, both thrust_rate and thrust should be "
                    "provided."
                )
            if np.shape(thrust_rate) != np.shape(thrust_is_regulated) or np.shape(
                thrust
            ) != np.shape(thrust_is_regulated):
                raise FastRubberEngineInconsistentInputParametersError(
                    "When use_thrust_rate is a sequence, both thrust_rate and thrust should have "
                    "same shape as use_thrust_rate"
                )

        return thrust_is_regulated, thrust_rate, thrust

    def psfc(
        self,
        atmosphere: Atmosphere,
        mach: Union[float, Sequence[float]],
        power_rate: Union[float, Sequence[float]],
        phase: Union[FlightPhase, Sequence],
    ) -> np.ndarray:

        """
        Computation of ratio :math:`\\frac{SFC(P)}{SFC(Pmax)}`, given altitude, mach
        and power_rate :math:`\\frac{F}{Fmax}`.



        :param altitude:
        :param thrust_rate:
        :return: SFC ratio
        """
        altitude = atmosphere.get_altitude(altitude_in_feet=True)
        filename = os.path.join(rhea_path, "resources/gasturbine/TP_L1/Metamodels")

        if phase == 1:
            filename = filename + "/TP_L1_NTO_PSFC.sav"
            poly = PolynomialFeatures(degree=3)
            x = poly.fit_transform(
                np.array([altitude, mach], dtype=object).reshape(1, -1)
            )

        elif phase == 2:
            filename = filename + "/TP_L1_MCL_PSFC.sav"
            poly = PolynomialFeatures(degree=3)
            x = poly.fit_transform(
                np.array([altitude, mach], dtype=object).reshape(1, -1)
            )

        elif phase == 3:
            filename = filename + "/TP_L1_CRZ_PSFC.sav"
            poly = PolynomialFeatures(degree=3)
            x = poly.fit_transform(
                np.array([altitude, mach, power_rate], dtype=object).reshape(1, -1)
            )

        elif phase == 5:
            filename = filename + "/TP_L1_CRZ_PSFC.sav"
            poly = PolynomialFeatures(degree=3)
            x = poly.fit_transform(
                np.array([altitude, mach, power_rate], dtype=object).reshape(1, -1)
            )

        elif phase == 8:
            filename = filename + "/TP_L1_RTO_PSFC.sav"
            poly = PolynomialFeatures(degree=3)
            x = poly.fit_transform(
                np.array([altitude, mach], dtype=object).reshape(1, -1)
            )

        elif phase == 9:
            filename = filename + "/TP_L1_MCT_PSFC.sav"
            poly = PolynomialFeatures(degree=3)
            x = poly.fit_transform(
                np.array([altitude, mach], dtype=object).reshape(1, -1)
            )

        else:
            filename = filename + "/TP_L1_MCL_PSFC.sav"
            poly = PolynomialFeatures(degree=3)
            x = poly.fit_transform(
                np.array([altitude, mach], dtype=object).reshape(1, -1)
            )

        loaded_model = joblib.load(open(filename, "rb"))
        psfc = loaded_model.predict(x)[0]

        return psfc

    def max_power(
        self,
        atmosphere: Atmosphere,
        mach: Union[float, Sequence[float]],
        phase: Union[FlightPhase, Sequence],
    ) -> np.ndarray:
        """
        Computation of maximum available power.

        Uses model described in :cite:`roux:2005`, p.57-58

        :param atmosphere: Atmosphere instance at intended altitude (should be <=20km)
        :param mach: Mach number(s) (should be between 0.05 and 1.0)
        :param phase: flight phase which influences engine rating (max mechanical power)
        :return: maximum power (in W)
        """
        """    TAXI_IN = 0
    TAKEOFF = 1
    CLIMB = 2
    CRUISE = 3
    DESCENT = 5
    LANDING = 6
    TAXI_OUT = 7"""
        altitude = atmosphere.get_altitude(altitude_in_feet=True)
        mach = np.asarray(mach)
        filename = os.path.join(rhea_path, "resources/gasturbine/TP_L1/Metamodels")

        if phase == 2:  #'MCL'
            max_power_rating = self.k_gb_MCL * self.RTO_power / constants.hp  # *1.05
            filename = filename + "/TP_L1_MCL_P_lapse.sav"
        elif phase == 4:  #'FID'
            max_power_rating = self.k_gb_FID * self.RTO_power / constants.hp
            filename = filename + "/TP_L1_CRZ_P_lapse.sav"

        elif phase == 1:  #'TO'
            max_power_rating = self.k_gb_NTO * self.RTO_power / constants.hp
            filename = filename + "/TP_L1_NTO_P_lapse.sav"

        elif phase == 9:  # MCT
            filename = filename + "/TP_L1_MCT_P_lapse.sav"
            max_power_rating = self.k_gb_MCT * self.RTO_power / constants.hp

        elif phase == 3:  #'CRZ'
            max_power_rating = self.k_gb_MCR * self.RTO_power / constants.hp
            filename = filename + "/TP_L1_CRZ_P_lapse.sav"
        elif phase == 8:  #'RTO'
            max_power_rating = (
                self.k_gb_RTO * self.RTO_power / constants.hp
            )  # Watt =2400. hp
            filename = filename + "/TP_L1_RTO_P_lapse.sav"

        else:
            max_power_rating = self.RTO_power / constants.hp
            filename = filename + "/TP_L1_MCL_P_lapse.sav"

        poly = PolynomialFeatures(degree=3)
        x = poly.fit_transform(np.array([altitude, mach], dtype=object).reshape(1, -1))
        loaded_model = joblib.load(open(filename, "rb"))
        K_powerlapse = loaded_model.predict(x)[0]

        # if altitude>=19000.:
        #     K_powerlapse=K_powerlapse*1.02

        max_thermo_power = max_power_rating * K_powerlapse

        if max_thermo_power > max_power_rating:
            max_shaft_power = max_power_rating  # hp #gearbox mechanical limit
        else:
            max_shaft_power = max_thermo_power

        # return max_shaft_power*constants.hp,max_thermo_power*constants.hp
        return (
            max_shaft_power * constants.hp,
            max_thermo_power * constants.hp,
            max_power_rating * constants.hp,
        )

    def compute_engine_point(
        self,
        atmosphere: Atmosphere,
        mach: Union[float, Sequence[float]],
        phase: Union[FlightPhase, Sequence],
        throttle=1,
        T_prop=1,
        shaft_power=1,
    ) -> np.ndarray:

        altitude = atmosphere.get_altitude(altitude_in_feet=True)
        filename = os.path.join(
            rhea_path, "resources/gasturbine/TP_L1/Metamodels/TP_L1_FR_Ratio.sav"
        )
        loaded_model = joblib.load(open(filename, "rb"))

        poly = PolynomialFeatures(degree=3)
        x = poly.fit_transform(
            np.array([altitude * constants.foot, mach, throttle], dtype=object).reshape(
                1, -1
            )
        )

        Ratio = loaded_model.predict(x)[0]

        FR = Ratio * T_prop
        return FR
