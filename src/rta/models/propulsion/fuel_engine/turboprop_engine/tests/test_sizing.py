"""Test for propeller and engine sizing"""

from pytest import approx
from fastoad.testing import run_system
from openmdao.core.indepvarcomp import IndepVarComp

from ..sizing.Prop_sizing import Prop_sizing
from ..sizing.TP_sizing import TP_sizing


def test_propeller_sizing():
    ivc = IndepVarComp()

    ivc.add_output("data:propulsion:propeller:disk_loading", 185, units="kW/m**2")
    ivc.add_output("data:propulsion:propeller:max_power", 2200, units="kW")

    problem = run_system(Prop_sizing(), ivc)

    assert problem["data:geometry:propulsion:propeller:diameter"] == approx(3.89, rel=1e-3)


def test_tp_sizing():
    ivc = IndepVarComp()

    ivc.add_output("data:propulsion:RTO_power", 2e6, units="W")
    ivc.add_output("data:propulsion:Design_Thermo_Power", 2.5e6, units="W")
    ivc.add_output("data:propulsion:Power_Offtake", 7500, units="W")
    ivc.add_output("data:propulsion:gearbox_eta", 0.98)
    ivc.add_output("data:propulsion:L1_engine:fuel", 0.0)
    ivc.add_output("data:propulsion:L1_engine:turbine_inlet_temperature", 1500, units="K")
    ivc.add_output("data:propulsion:L1_engine:inlet:inlet_eta_pol", 0.98)
    ivc.add_output("data:propulsion:L1_engine:inlet:inlet_pressure_ratio", 1.0)
    ivc.add_output("data:propulsion:L1_engine:lpc:lpc_eta_pol", 0.85)
    ivc.add_output("data:propulsion:L1_engine:lpc:lpc_pressure_ratio", 5.65)
    ivc.add_output("data:propulsion:L1_engine:hpc:hpc_eta_pol", 0.84)
    ivc.add_output("data:propulsion:L1_engine:hpc:hpc_pressure_ratio", 2.6)
    ivc.add_output("data:propulsion:L1_engine:combustor:combustor_eta", 0.995)
    ivc.add_output("data:propulsion:L1_engine:combustor:combustor_pressure_ratio", 0.93)
    ivc.add_output("data:propulsion:L1_engine:hpt:hpt_eta_pol", 0.86)
    ivc.add_output("data:propulsion:L1_engine:hpt:hpt_eta_mech", 0.96)
    ivc.add_output("data:propulsion:L1_engine:lpt:lpt_eta_pol", 0.86)
    ivc.add_output("data:propulsion:L1_engine:lpt:lpt_eta_mech", 0.96)
    ivc.add_output("data:propulsion:L1_engine:pt:pt_eta_pol", 0.88)
    ivc.add_output("data:propulsion:L1_engine:pt:pt_eta_mech", 0.98)
    ivc.add_output("data:propulsion:L1_engine:nozzle:nozzle_eta_pol", 0.98)
    ivc.add_output("data:propulsion:L1_engine:nozzle:nozzle_pressure_ratio", 1.0)
    ivc.add_output("data:propulsion:L1_engine:nozzle:nozzle_area_ratio", 1.33)

    problem = run_system(TP_sizing(), ivc)

    assert problem["data:propulsion:L1_engine:sizing:k0"] == approx(2.44e-4, rel=1e-3)
    assert problem["data:propulsion:L1_engine:sizing:k1"] == approx(0.132, rel=1e-3)
    assert problem["data:propulsion:L1_engine:sizing:k2"] == approx(0.1517, rel=1e-3)
    assert problem["data:propulsion:L1_engine:sizing:tau_t_sizing"] == approx(0.589, rel=1e-3)
    assert problem["data:propulsion:L1_engine:sizing:pi_t_sizing"] == approx(0.0817, rel=1e-3)
    assert problem["data:propulsion:L1_engine:sizing:M_out_sizing"] == approx(0.41, rel=1e-3)
