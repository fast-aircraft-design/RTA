"""
test module for modules in aerodynamics/components
"""

#  This file is part of FAST-OAD_CS25
#  Copyright (C) 2023 ONERA & ISAE-SUPAERO
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

import os.path as pth

import numpy as np
from fastoad._utils.testing import run_system
from fastoad.io import VariableIO
from numpy.testing import assert_allclose
from openmdao.core.group import Group
from openmdao.core.indepvarcomp import IndepVarComp
from pytest import approx
from stdatm import Atmosphere

from ..cd0_fuselage import Cd0Fuselage
from ..lg_effect import ComputeDeltaLg
from ..OEI_effect import ComputeDeltaOEI
from ..cd0_wing import Cd0Wing
from ..cd0_total import Cd0Total
from ..compute_polar import ComputePolar
from ..cd0_nacelle_pylons_TP import Cd0NacelleAndPylonsTP
from ..high_lift_aero import ComputeDeltaHighLift
# from ..oswald import OswaldCoefficient
from fastoad_cs25.models.aerodynamics.components.oswald import OswaldCoefficient


def get_indep_var_comp(var_names):
    """Reads required input data and returns an IndepVarcomp() instance"""
    reader = VariableIO(pth.join(pth.dirname(__file__), "data", "ref_aerodynamics.xml"))
    reader.path_separator = ":"
    ivc = reader.read(only=var_names).to_ivc()
    return ivc


def test_fuselage_cd0():

    input_list_lowspeed = [
        "data:aerodynamics:wing:low_speed:reynolds",
        "data:aerodynamics:aircraft:low_speed:CL",
        "data:aerodynamics:aircraft:takeoff:mach",
        "data:geometry:wing:area",
        "data:geometry:fuselage:length",
        "data:geometry:fuselage:maximum_width",
        "data:geometry:fuselage:maximum_height",
        "data:geometry:fuselage:wetted_area"
    ]

    input_list_cruise = [
        "data:aerodynamics:wing:cruise:reynolds",
        "data:aerodynamics:aircraft:cruise:CL",
        "data:TLAR:cruise_mach",
        "data:geometry:wing:area",
        "data:geometry:fuselage:length",
        "data:geometry:fuselage:maximum_width",
        "data:geometry:fuselage:maximum_height",
        "data:geometry:fuselage:wetted_area"
    ]

    inputs = get_indep_var_comp(input_list_lowspeed)

    prob = run_system(Cd0Fuselage(low_speed_aero = True), inputs)

    assert prob["data:aerodynamics:fuselage:low_speed:CD0"][0] == approx(0.01117, abs=1e-4)
    assert prob["data:aerodynamics:fuselage:low_speed:CD0"][35] == approx(0.01011, abs=1e-4)
    assert prob["data:aerodynamics:fuselage:low_speed:CD0"][120] == approx(0.01145, abs=1e-4)

    inputs = get_indep_var_comp(input_list_cruise)

    prob = run_system(Cd0Fuselage(low_speed_aero=False), inputs)

    assert prob["data:aerodynamics:fuselage:cruise:CD0"][0] == approx(0.01073, abs=1e-4)
    assert prob["data:aerodynamics:fuselage:cruise:CD0"][35] == approx(0.01009, abs=1e-4)
    assert prob["data:aerodynamics:fuselage:cruise:CD0"][120] == approx(0.0095, abs=1e-4)


def test_nacelle_cd0():

    input_list_low_speed = [
        "data:aerodynamics:wing:low_speed:reynolds",
        "data:aerodynamics:aircraft:takeoff:mach",
        "data:geometry:propulsion:pylon:length",
        "data:geometry:propulsion:nacelle:length",
        "data:geometry:propulsion:pylon:wetted_area",
        "data:geometry:propulsion:nacelle:wetted_area",
        "data:geometry:propulsion:engine:count",
        "data:geometry:wing:area",
    ]

    input_list_cruise = [
        "data:aerodynamics:wing:cruise:reynolds",
        "data:TLAR:cruise_mach",
        "data:geometry:propulsion:pylon:length",
        "data:geometry:propulsion:nacelle:length",
        "data:geometry:propulsion:pylon:wetted_area",
        "data:geometry:propulsion:nacelle:wetted_area",
        "data:geometry:propulsion:engine:count",
        "data:geometry:wing:area",
    ]

    inputs = get_indep_var_comp(input_list_low_speed)
    prob = run_system(Cd0NacelleAndPylonsTP(low_speed_aero=True), inputs)

    assert prob["data:aerodynamics:nacelles:low_speed:CD0"] == approx(0.00147, abs=1e-5)


    inputs = get_indep_var_comp(input_list_cruise)
    prob = run_system(Cd0NacelleAndPylonsTP(low_speed_aero=False), inputs)

    assert prob["data:aerodynamics:nacelles:cruise:CD0"] == approx(0.00145, abs=1e-5)

def test_oswald_coefficient():
    """Tests Oswald Coefficient, updated with RTA results"""
    input_list = [
        "data:geometry:wing:area",
        "data:geometry:wing:span",
        "data:geometry:fuselage:maximum_height",
        "data:geometry:fuselage:maximum_width",
        "data:geometry:wing:root:chord",
        "data:geometry:wing:tip:chord",
        "data:geometry:wing:sweep_25",
    ]

    def get_coeff(mach, low_speed_aero=False):
        ivc = get_indep_var_comp(input_list)
        if low_speed_aero:
            ivc.add_output("data:aerodynamics:aircraft:takeoff:mach", mach)
        else:
            ivc.add_output("data:TLAR:cruise_mach", mach)
        problem = run_system(OswaldCoefficient(low_speed_aero=low_speed_aero), ivc)
        if low_speed_aero:
            return problem["data:aerodynamics:aircraft:low_speed:oswald_coefficient"]
        else:
            return problem["data:aerodynamics:aircraft:cruise:oswald_coefficient"]

    assert get_coeff(0.2, True) == approx(0.8954, abs=1e-4)

    assert get_coeff(0.45, False) == approx(0.8954, abs=1e-4)


def test_cd0():
    """Tests CD0 (test of the group for comparison to legacy)"""
    input_list = [
        "data:geometry:fuselage:maximum_height",
        "data:geometry:fuselage:length",
        "data:geometry:fuselage:wetted_area",
        "data:geometry:fuselage:maximum_width",
        "data:geometry:horizontal_tail:MAC:length",
        "data:geometry:horizontal_tail:sweep_25",
        "data:geometry:horizontal_tail:thickness_ratio",
        "data:geometry:horizontal_tail:wetted_area",
        "data:geometry:wing:area",
        "data:geometry:propulsion:engine:count",
        "data:geometry:propulsion:fan:length",
        "data:geometry:propulsion:nacelle:length",
        "data:geometry:propulsion:nacelle:wetted_area",
        "data:geometry:propulsion:pylon:length",
        "data:geometry:propulsion:pylon:wetted_area",
        "data:geometry:aircraft:wetted_area",
        "data:geometry:vertical_tail:MAC:length",
        "data:geometry:vertical_tail:sweep_25",
        "data:geometry:vertical_tail:thickness_ratio",
        "data:geometry:vertical_tail:wetted_area",
        "data:geometry:wing:area",
        "data:geometry:wing:MAC:length",
        "data:geometry:wing:sweep_25",
        "data:geometry:wing:thickness_ratio",
        "data:geometry:wing:wetted_area",
    ]

    def get_cd0(alt, mach, cl, low_speed_aero):
        atm = Atmosphere(alt)
        atm.mach = mach
        reynolds = atm.unitary_reynolds

        ivc = get_indep_var_comp(input_list)
        if low_speed_aero:
            ivc.add_output("data:aerodynamics:aircraft:takeoff:mach", mach)
            ivc.add_output("data:aerodynamics:wing:low_speed:reynolds", reynolds)
            ivc.add_output(
                "data:aerodynamics:aircraft:low_speed:CL", 150 * [cl]
            )  # needed because size of input array is fixed
        else:
            ivc.add_output("data:TLAR:cruise_mach", mach)
            ivc.add_output("data:aerodynamics:wing:cruise:reynolds", reynolds)
            ivc.add_output(
                "data:aerodynamics:aircraft:cruise:CL", 150 * [cl]
            )  # needed because size of input array is fixed
        problem = run_system(Cd0Total(low_speed_aero=low_speed_aero), ivc)
        if low_speed_aero:
            return problem["data:aerodynamics:aircraft:low_speed:CD0"][0]
        else:
            return problem["data:aerodynamics:aircraft:cruise:CD0"][0]

    assert get_cd0(35000, 0.78, 0.5, False) == approx(0.01975, abs=1e-5)
    assert get_cd0(0, 0.2, 0.9, True) == approx(0.02727, abs=1e-5)


def test_polar_high_speed():
    """Tests ComputePolar"""

    input_list = [
        "tuning:aerodynamics:aircraft:cruise:CD:k",
        "tuning:aerodynamics:aircraft:cruise:CD:offset",
        "tuning:aerodynamics:aircraft:cruise:CD:winglet_effect:k",
        "tuning:aerodynamics:aircraft:cruise:CD:winglet_effect:offset",
        "data:aerodynamics:aircraft:cruise:CD0",
        "data:aerodynamics:aircraft:cruise:CD:trim",
        "data:aerodynamics:aircraft:cruise:CD:compressibility",
        "data:aerodynamics:aircraft:cruise:induced_drag_coefficient",
        "data:aerodynamics:aircraft:low_speed:CD0",
        "data:aerodynamics:aircraft:low_speed:CD:trim",
        "data:aerodynamics:aircraft:low_speed:induced_drag_coefficient",
        "tuning:aerodynamics:aircraft:low_speed:CD:winglet_effect:k",
        "tuning:aerodynamics:aircraft:low_speed:CD:winglet_effect:offset"
    ]


    ivc = get_indep_var_comp(input_list)
    ivc.add_output("data:aerodynamics:aircraft:cruise:CL", np.arange(0.0, 1.5, 0.01))
    ivc.add_output("data:aerodynamics:aircraft:low_speed:CL", np.arange(0.0, 1.5, 0.01))

    problem = run_system(ComputePolar(), ivc)

    cd = problem["data:aerodynamics:aircraft:cruise:CD"]
    cl = problem["data:aerodynamics:aircraft:cruise:CL"]

    assert cd[cl == 0.0] == approx(0.027555, abs=1e-5)
    assert cd[cl == 0.2] == approx(0.02281, abs=1e-5)
    assert cd[cl == 0.42] == approx(0.02977, abs=1e-5)
    assert cd[cl == 0.85] == approx(0.24041, abs=1e-5)

    assert problem["data:aerodynamics:aircraft:cruise:optimal_CL"] == approx(0.51, abs=1e-3)
    assert problem["data:aerodynamics:aircraft:cruise:optimal_CD"] == approx(0.03487, abs=1e-5)


def _test_polar_low_speed():
    """Tests ComputePolar"""

    # Need to plug Cd modules, Reynolds and Oswald

    input_list = [
        "data:aerodynamics:aircraft:takeoff:mach",
        "data:geometry:wing:area",
        "data:geometry:wing:span",
        "data:geometry:fuselage:maximum_height",
        "data:geometry:fuselage:maximum_width",
        "data:geometry:wing:root:chord",
        "data:geometry:wing:tip:chord",
        "data:geometry:wing:sweep_25",
        "data:geometry:wing:thickness_ratio",
        "data:geometry:wing:wetted_area",
        "data:geometry:wing:MAC:length",
        "data:geometry:fuselage:length",
        "data:geometry:fuselage:wetted_area",
        "data:geometry:horizontal_tail:MAC:length",
        "data:geometry:horizontal_tail:thickness_ratio",
        "data:geometry:horizontal_tail:sweep_25",
        "data:geometry:horizontal_tail:wetted_area",
        "data:geometry:vertical_tail:MAC:length",
        "data:geometry:vertical_tail:thickness_ratio",
        "data:geometry:vertical_tail:sweep_25",
        "data:geometry:vertical_tail:wetted_area",
        "data:geometry:propulsion:pylon:length",
        "data:geometry:propulsion:nacelle:length",
        "data:geometry:propulsion:pylon:wetted_area",
        "data:geometry:propulsion:nacelle:wetted_area",
        "data:geometry:propulsion:engine:count",
        "data:geometry:propulsion:fan:length",
        "data:geometry:aircraft:wetted_area",
        "tuning:aerodynamics:aircraft:cruise:CD:k",
        "tuning:aerodynamics:aircraft:cruise:CD:offset",
        "tuning:aerodynamics:aircraft:cruise:CD:winglet_effect:k",
        "tuning:aerodynamics:aircraft:cruise:CD:winglet_effect:offset",
    ]
    group = Group()
    group.add_subsystem("reynolds", ComputeReynolds(low_speed_aero=True), promotes=["*"])
    group.add_subsystem("oswald", OswaldCoefficient(low_speed_aero=True), promotes=["*"])
    group.add_subsystem(
        "induced_drag_coeff", InducedDragCoefficient(low_speed_aero=True), promotes=["*"]
    )
    group.add_subsystem("cd0", CD0(low_speed_aero=True), promotes=["*"])
    group.add_subsystem("cd_trim", CdTrim(low_speed_aero=True), promotes=["*"])
    group.add_subsystem("polar", ComputePolar(polar_type=PolarType.LOW_SPEED), promotes=["*"])

    ivc = get_indep_var_comp(input_list)
    ivc.add_output("data:aerodynamics:aircraft:low_speed:CL", np.arange(0.0, 1.5, 0.01))

    problem = run_system(group, ivc)

    cd = problem["data:aerodynamics:aircraft:low_speed:CD"]
    cl = problem["data:aerodynamics:aircraft:low_speed:CL"]

    assert cd[cl == 0.5] == approx(0.033441, abs=1e-5)
    assert cd[cl == 1.0] == approx(0.077523, abs=1e-5)


def _test_polar_high_lift():
    """Tests ComputePolar"""

    # Need to plug Cd modules, Reynolds and Oswald

    input_list = [
        "data:aerodynamics:aircraft:takeoff:mach",
        "data:geometry:wing:area",
        "data:geometry:wing:span",
        "data:geometry:fuselage:maximum_height",
        "data:geometry:fuselage:maximum_width",
        "data:geometry:wing:root:chord",
        "data:geometry:wing:tip:chord",
        "data:geometry:wing:sweep_25",
        "data:geometry:wing:thickness_ratio",
        "data:geometry:wing:wetted_area",
        "data:geometry:wing:MAC:length",
        "data:geometry:fuselage:length",
        "data:geometry:fuselage:wetted_area",
        "data:geometry:horizontal_tail:MAC:length",
        "data:geometry:horizontal_tail:thickness_ratio",
        "data:geometry:horizontal_tail:sweep_25",
        "data:geometry:horizontal_tail:wetted_area",
        "data:geometry:vertical_tail:MAC:length",
        "data:geometry:vertical_tail:thickness_ratio",
        "data:geometry:vertical_tail:sweep_25",
        "data:geometry:vertical_tail:wetted_area",
        "data:geometry:propulsion:pylon:length",
        "data:geometry:propulsion:nacelle:length",
        "data:geometry:propulsion:pylon:wetted_area",
        "data:geometry:propulsion:nacelle:wetted_area",
        "data:geometry:propulsion:engine:count",
        "data:geometry:propulsion:fan:length",
        "data:geometry:aircraft:wetted_area",
        "tuning:aerodynamics:aircraft:cruise:CD:k",
        "tuning:aerodynamics:aircraft:cruise:CD:offset",
        "tuning:aerodynamics:aircraft:cruise:CD:winglet_effect:k",
        "tuning:aerodynamics:aircraft:cruise:CD:winglet_effect:offset",
        "data:aerodynamics:high_lift_devices:takeoff:CL",
        "data:aerodynamics:high_lift_devices:takeoff:CD",
    ]
    group = Group()
    group.add_subsystem("reynolds", ComputeReynolds(low_speed_aero=True), promotes=["*"])
    group.add_subsystem("oswald", OswaldCoefficient(low_speed_aero=True), promotes=["*"])
    group.add_subsystem(
        "induced_drag_coeff", InducedDragCoefficient(low_speed_aero=True), promotes=["*"]
    )
    group.add_subsystem("cd0", CD0(low_speed_aero=True), promotes=["*"])
    group.add_subsystem("cd_trim", CdTrim(low_speed_aero=True), promotes=["*"])
    group.add_subsystem("polar", ComputePolar(polar_type=PolarType.TAKEOFF), promotes=["*"])

    ivc = get_indep_var_comp(input_list)
    ivc.add_output("data:aerodynamics:aircraft:low_speed:CL", np.arange(0.0, 1.5, 0.01))

    problem = run_system(group, ivc)

    cd = problem["data:aerodynamics:aircraft:takeoff:CD"]
    cl = problem["data:aerodynamics:aircraft:takeoff:CL"]

    assert cd[cl == 1.0] == approx(0.091497, abs=1e-5)
    assert cd[cl == 1.5] == approx(0.180787, abs=1e-5)

