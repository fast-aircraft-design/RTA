#  This file is part of FAST-OAD_CS25
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
# from fastoad.io import VariableIO
from ..a_airframe import NacellesWeight


def test_compute_payload():
    ivc = om.IndepVarComp()
    ivc.add_output("data:propulsion:RTO_power", val=2.05e6, units='W')
    ivc.add_output("data:geometry:propulsion:engine:count", val=2)
    ivc.add_output("tuning:weight:airframe:nacelle:mass:k", val=1.0)
    ivc.add_output("tuning:weight:airframe:nacelle:mass:offset", val=0.0)

    problem = run_system(NacellesWeight(), ivc)

    assert problem["data:weight:airframe:nacelle:mass"] == pytest.approx(349.15, abs=0.1)