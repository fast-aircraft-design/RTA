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
from ..cg_components.compute_cg_tanks_RHEA import ComputeTanksCG_RHEA
# from ..cg_components.compute_cg_wing import ComputeWingCG
# from ..cg_components.compute_global_cg import ComputeGlobalCG
# from ..cg_components.compute_ht_cg import ComputeHTcg
# from ..cg_components.compute_max_cg_ratio import ComputeMaxCGratio
# from ..cg_components.compute_vt_cg import ComputeVTcg
# from ..cg_components.load_cases.compute_cg_loadcase1 import ComputeCGLoadCase1
# from ..cg_components.load_cases.compute_cg_loadcase2 import ComputeCGLoadCase2
# from ..cg_components.load_cases.compute_cg_loadcase3 import ComputeCGLoadCase3
# from ..cg_components.load_cases.compute_cg_loadcase4 import ComputeCGLoadCase4
# from ..cg_components.load_cases.compute_cg_loadcases import CGRatiosForLoadCases
# from ..cg_components.update_mlg import UpdateMLG

def test_compute_cg_tanks():
    """Tests computation of tanks center of gravity"""

    input_vars = om.IndepVarComp()
    input_vars.add_output("data:geometry:fuselage:maximum_width", 3.92, units="m")
    input_vars.add_output("data:geometry:wing:MAC:length", 4.457, units="m")
    input_vars.add_output("data:geometry:wing:MAC:at25percent:x", 16.457, units="m")
    input_vars.add_output("data:geometry:wing:MAC:leading_edge:x:local", 2.361, units="m")
    input_vars.add_output("data:geometry:wing:kink:chord", 3.985, units="m")
    input_vars.add_output("data:geometry:wing:kink:y", 6.321, units="m")
    input_vars.add_output("data:geometry:wing:kink:leading_edge:x:local", 2.275, units="m")
    input_vars.add_output("data:geometry:wing:root:chord", 6.26, units="m")
    input_vars.add_output("data:geometry:wing:root:y", 1.96, units="m")
    input_vars.add_output("data:geometry:wing:tip:chord", 1.882, units="m")
    input_vars.add_output("data:geometry:wing:tip:y", 15.801, units="m")
    input_vars.add_output("data:geometry:wing:tip:leading_edge:x:local", 7.222, units="m")
    input_vars.add_output("data:geometry:wing:spar_ratio:front:kink", 0.15, units=None)
    input_vars.add_output("data:geometry:wing:spar_ratio:front:root", 0.11, units=None)
    input_vars.add_output("data:geometry:wing:spar_ratio:front:tip", 0.27, units=None)
    input_vars.add_output("data:geometry:wing:spar_ratio:rear:kink", 0.66, units=None)
    input_vars.add_output("data:geometry:wing:spar_ratio:rear:root", 0.57, units=None)
    input_vars.add_output("data:geometry:wing:spar_ratio:rear:tip", 0.56, units=None)

    problem = run_system(ComputeTanksCG_RHEA(), input_vars)

    x_cg_tank = problem["data:weight:fuel_tank:CG:x"]
    assert x_cg_tank == pytest.approx(16.12, abs=1e-2)

    # With no kink
    input_vars = om.IndepVarComp()
    input_vars.add_output("data:geometry:fuselage:maximum_width", 3.92, units="m")
    input_vars.add_output("data:geometry:wing:MAC:length", 4.457, units="m")
    input_vars.add_output("data:geometry:wing:MAC:at25percent:x", 16.457, units="m")
    input_vars.add_output("data:geometry:wing:MAC:leading_edge:x:local", 2.361, units="m")
    input_vars.add_output("data:geometry:wing:kink:chord", 6.26, units="m")
    input_vars.add_output("data:geometry:wing:kink:y", 1.96, units="m")
    input_vars.add_output("data:geometry:wing:kink:leading_edge:x:local", 0.0, units="m")
    input_vars.add_output("data:geometry:wing:root:chord", 6.26, units="m")
    input_vars.add_output("data:geometry:wing:root:y", 1.96, units="m")
    input_vars.add_output("data:geometry:wing:tip:chord", 1.882, units="m")
    input_vars.add_output("data:geometry:wing:tip:y", 15.801, units="m")
    input_vars.add_output("data:geometry:wing:tip:leading_edge:x:local", 7.222, units="m")
    input_vars.add_output("data:geometry:wing:spar_ratio:front:kink", 0.15, units=None)
    input_vars.add_output("data:geometry:wing:spar_ratio:front:root", 0.11, units=None)
    input_vars.add_output("data:geometry:wing:spar_ratio:front:tip", 0.27, units=None)
    input_vars.add_output("data:geometry:wing:spar_ratio:rear:kink", 0.66, units=None)
    input_vars.add_output("data:geometry:wing:spar_ratio:rear:root", 0.57, units=None)
    input_vars.add_output("data:geometry:wing:spar_ratio:rear:tip", 0.56, units=None)

    problem = run_system(ComputeTanksCG_RHEA(), input_vars)

    x_cg_tank = problem["data:weight:fuel_tank:CG:x"]
    assert x_cg_tank == pytest.approx(16.52, abs=1e-2)