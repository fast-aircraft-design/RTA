from pytest import approx
from fastoad.testing import run_system
from openmdao.core.indepvarcomp import IndepVarComp

from ..nacelle.compute_nacelle import ComputeNacelleGeometry
from ..wing.components.compute_toc_wing_rta import ComputeToCWingRTA
from ..wing.components.compute_wet_area_wing_rta import ComputeWetAreaWingRTA


def test_nacelle():
    ivc = IndepVarComp()

    ivc.add_output("data:geometry:propulsion:engine:y_ratio", val=0.3)
    ivc.add_output("data:geometry:wing:span", val=26.84, units="m")
    ivc.add_output("data:propulsion:Design_Thermo_Power", val=2.497e6, units="W")
    ivc.add_output("data:propulsion:electric_systems:P_nom", val=0.0, units="W")

    problem = run_system(ComputeNacelleGeometry(), ivc)

    assert problem["data:geometry:propulsion:nacelle:length"] == approx(2.17, rel=1e-3)
    assert problem["data:geometry:propulsion:nacelle:diameter"] == approx(0.638, rel=1e-3)
    assert problem["data:geometry:propulsion:nacelle:y"] == approx(4.026, rel=1e-3)
    assert problem["data:geometry:propulsion:nacelle:wetted_area"] == approx(4.349, rel=1e-3)


def test_wing_ToC():
    ivc = IndepVarComp()

    ivc.add_output("data:TLAR:cruise_mach", val=0.45)
    ivc.add_output("data:geometry:wing:sweep_25", val=2.3, units="deg")

    problem = run_system(ComputeToCWingRTA(), ivc)

    assert problem["data:geometry:wing:thickness_ratio"] == approx(0.1407, rel=1e-3)
    assert problem["data:geometry:wing:root:thickness_ratio"] == approx(0.1875, rel=1e-3)
    assert problem["data:geometry:wing:kink:thickness_ratio"] == approx(0.1407, rel=1e-3)
    assert problem["data:geometry:wing:tip:thickness_ratio"] == approx(0.125, rel=1e-3)


def test_wing_wet_area():
    ivc = IndepVarComp()

    ivc.add_output("data:geometry:wing:root:chord", val=2.634, units="m")
    ivc.add_output("data:geometry:wing:root:y", val=1.396, units="m")
    ivc.add_output("data:geometry:wing:area", val=60.07, units="m**2")
    ivc.add_output("data:geometry:wing:thickness_ratio", val=0.1407)

    problem = run_system(ComputeWetAreaWingRTA(), ivc)

    assert problem["data:geometry:wing:outer_area"] == approx(52.715, rel=1e-3)
    assert problem["data:geometry:wing:wetted_area"] == approx(108.075, rel=1e-3)
