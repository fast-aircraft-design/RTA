"""
Test module for geometry functions of cg components
"""
#  This file is part of RTA
#  Copyright (C) 2022 ONERA & ISAE-SUPAERO
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

# pylint: disable=redefined-outer-name  # needed for pytest fixtures

import os.path as pth

import openmdao.api as om
import pytest
from fastoad._utils.testing import run_system
from fastoad.io import VariableIO

# from ..cg import ComputeAircraftCG
# from ..cg_components.compute_cg_control_surfaces import ComputeControlSurfacesCG
# from ..cg_components.compute_cg_others import ComputeOthersCG
# from ..cg_components.compute_cg_ratio_aft import ComputeCGRatioAft
# from ..cg_components.compute_global_cg import ComputeGlobalCG
# from ..cg_components.compute_max_cg_ratio import ComputeMaxCGratio
# from ..cg_components.load_cases.compute_cg_loadcase1 import ComputeCGLoadCase1
# from ..cg_components.load_cases.compute_cg_loadcase2 import ComputeCGLoadCase2
# from ..cg_components.load_cases.compute_cg_loadcase3 import ComputeCGLoadCase3
# from ..cg_components.load_cases.compute_cg_loadcase4 import ComputeCGLoadCase4
# from ..cg_components.load_cases.compute_cg_loadcases import CGRatiosForLoadCases
# from ..cg_components.update_mlg import UpdateMLG
from ..cg_components.compute_cg_flight_controls import (
    ComputeFlightControlCG,
)


def test_ComputeFlightControlCG():
    """Tests computation of tanks center of gravity"""

    input_vars = om.IndepVarComp()
    input_vars.add_output("data:geometry:wing:MAC:length", 2.277, units="m")
    input_vars.add_output("data:geometry:wing:MAC:y", 6.197, units="m")
    input_vars.add_output("data:geometry:wing:MAC:at25percent:x", 12.628, units="m")
    input_vars.add_output(
        "data:geometry:wing:MAC:leading_edge:x:local", 0.302, units="m"
    )
    input_vars.add_output("data:geometry:wing:kink:chord", 2.4, units="m")
    input_vars.add_output("data:geometry:wing:kink:y", 5.189, units="m")
    input_vars.add_output(
        "data:geometry:wing:kink:leading_edge:x:local", 0.234, units="m"
    )
    input_vars.add_output("data:geometry:wing:root:chord", 2.633, units="m")
    input_vars.add_output("data:geometry:wing:root:y", 1.396, units="m")
    input_vars.add_output("data:geometry:wing:tip:chord", 1.685, units="m")
    input_vars.add_output(
        "data:geometry:wing:tip:leading_edge:x:local", 0.743, units="m"
    )
    input_vars.add_output("data:geometry:wing:tip:y", 13.421, units="m")

    problem = run_system(ComputeFlightControlCG(), input_vars)

    x_cg_flight_control = problem["data:weight:systems:flight_controls:CG:x"]
    assert x_cg_flight_control == pytest.approx(14.365, abs=1e-2)
