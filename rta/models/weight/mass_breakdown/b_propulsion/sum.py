import openmdao.api as om
from fastoad.module_management.service_registry import RegisterSubmodel
from fastoad_cs25.models.weight.mass_breakdown.b_propulsion.constants import SERVICE_FUEL_LINES_MASS
from fastoad_cs25.models.weight.mass_breakdown.constants import SERVICE_PROPULSION_MASS

from rta.models.weight.mass_breakdown.b_propulsion.constants import SERVICE_TURBOPROP_MASS

RegisterSubmodel.active_models[SERVICE_PROPULSION_MASS] = ("rta.submodel.weight.mass.propulsion.legacy")

@RegisterSubmodel(SERVICE_PROPULSION_MASS, "rta.submodel.weight.mass.propulsion.legacy")
class PropulsionWeight(om.Group):
    """
    Computes mass of propulsion.
    """

    def setup(self):
        # Engine RTOpower has to be computed before nacelles
        self.add_subsystem("ATA61_72_73", RegisterSubmodel.get_submodel(SERVICE_TURBOPROP_MASS), promotes=["*"])
        self.add_subsystem("ATA28", RegisterSubmodel.get_submodel(SERVICE_FUEL_LINES_MASS), promotes=["*"])

        weight_sum = om.AddSubtractComp()
        weight_sum.add_equation(
            "data:weight:propulsion:mass",
            [
                "data:weight:propulsion:engine:mass",
                "data:weight:propulsion:propeller:mass",
                "data:weight:propulsion:engine_controls_instrumentation:mass",
                "data:weight:propulsion:fuel_lines:mass",
            ],
            units="kg",
            desc="Mass of the propulsion system",
        )

        self.add_subsystem(
            "propulsion_weight_sum",
            weight_sum,
            promotes=["*"],
        )