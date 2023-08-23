# """
# This module is for registering all internal OpenMDAO modules that we want
# available through OpenMDAOSystemRegistry
# """
# #  This file is part of FAST : A framework for rapid Overall Aircraft Design
# #  Copyright (C) 2020  ONERA & ISAE-SUPAERO
# #  FAST is free software: you can redistribute it and/or modify
# #  it under the terms of the GNU General Public License as published by
# #  the Free Software Foundation, either version 3 of the License, or
# #  (at your option) any later version.
# #  This program is distributed in the hope that it will be useful,
# #  but WITHOUT ANY WARRANTY; without even the implied warranty of
# #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# #  GNU General Public License for more details.
# #  You should have received a copy of the GNU General Public License
# #  along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
#
# from aerodynamics.aerodynamics_high_speed_RHEA import AerodynamicsHighSpeed_RHEA
# from aerodynamics.aerodynamics_low_speed_RHEA import AerodynamicsLowSpeed_RHEA
# from aerodynamics.aerodynamics_takeoff_RHEA import AerodynamicsTakeoffRHEA
#
# from geometry.geometry_RHEA_sizing import Geometry_sizing
# from geometry.geometry_RHEA_IN import Geometry_IN
# from geometry.geometry_RHEA_wing_IN import Geometry_sizing_no_wing
# from loops.compute_wing_area_RHEA import ComputeWingArea_RHEA
# from loops.compute_engine_size import ComputeEngineSize
# from loops.compute_wing_position import ComputeWingPosition
# #from geometry.geometry_RHEA2 import Geometry_RHEA2
# from weight.mass_breakdown.mass_breakdown_RHEA_OAD import MTOWComputation_RHEA
#
# from weight.weight_RHEA_IN import Weight_RHEA_IN
# from weight.weight_RHEA_sizing import Weight_RHEA_sizing
# from weight.weight_RHEA_sizing_OAD import Weight_RHEA_sizing_OAD
#
# from performances.mission.openmdao.regional_flight import SizingFlight_RHEA
# from performances.mission.openmdao.OEI_ceiling import OEI_ceiling
# from performances.mission.openmdao.TOFL import TakeOffFlight
# from performances.mission.openmdao.DOC_regional_flight import DOCFlight_RHEA
# from performances.mission.openmdao.Payload_range import Payload_range
# from performances.mission.complete_mission_outputs import CompleteMissionOutputs
#
# from propulsion.fuel_engine.turboprop_engine.sizing import TP_sizing
# from propulsion.fuel_engine.turboprop_engine.sizing import Prop_sizing
#
# from propulsion.fuel_engine.hybrid_engine.sizing import FC_sizing
# from propulsion.fuel_engine.hybrid_engine.sizing import BoP_sizing
#
# from fastoad.module_management import OpenMDAOSystemRegistry
# from fastoad.module_management.constants import ModelDomain
# #from fastoad.models.performances import BreguetFromMTOW
# #from performances.breguet_RHEA import BreguetFromOWE_RHEA, BreguetFromMTOW_RHEA
# from propulsion.fuel_engine.turboprop_engine import  OMTPEngineL0Component,OMTPEngineL1Component
# from propulsion.fuel_engine.complete_propulsion_inputs import CompletePropulsionInputs
# """
# The place where to register RHEA internal models.
#
# Warning: this function is effective only if called from a Python module that
# is a started bundle for iPOPO
# """
# # Aerodynamics ################################################################
#
# OpenMDAOSystemRegistry.register_system(
#     AerodynamicsHighSpeed_RHEA,
#     "rhea.aerodynamics.highspeed",
#     domain=ModelDomain.AERODYNAMICS,
# )
# '''OpenMDAOSystemRegistry.register_system(
#     AerodynamicsHighSpeed_ATR,
#     "rhea.aerodynamics.highspeedATR",
#     domain=ModelDomain.AERODYNAMICS,
# )'''
#
# OpenMDAOSystemRegistry.register_system(
#     AerodynamicsLowSpeed_RHEA,
#     "rhea.aerodynamics.lowspeed",
#     domain=ModelDomain.AERODYNAMICS,
# )
#
#
# OpenMDAOSystemRegistry.register_system(
#     AerodynamicsTakeoffRHEA,
#     "rhea.aerodynamics.takeoff",
#     domain=ModelDomain.AERODYNAMICS,
# )
#
# # Geometry ####################################################################
#
# OpenMDAOSystemRegistry.register_system(
#     Geometry_IN, "rhea.geometry.IN", domain=ModelDomain.GEOMETRY
# )
# OpenMDAOSystemRegistry.register_system(
#     Geometry_sizing, "rhea.geometry.sizing", domain=ModelDomain.GEOMETRY
# )
# OpenMDAOSystemRegistry.register_system(
#     Geometry_sizing_no_wing, "rhea.geometry.sizing_wing_IN", domain=ModelDomain.GEOMETRY
# )
#
# # handling qualities ##########################################################
#
# # Loops #######################################################################
# OpenMDAOSystemRegistry.register_system(
#     ComputeWingArea_RHEA, "rhea.loop.wing_area", domain=ModelDomain.OTHER
# )
# OpenMDAOSystemRegistry.register_system(
#     ComputeEngineSize, "rhea.loop.engine_size", domain=ModelDomain.PROPULSION
# )
# OpenMDAOSystemRegistry.register_system(
#     ComputeWingPosition,"rhea.loop.wing_position", domain=ModelDomain.GEOMETRY
# )
#
# OpenMDAOSystemRegistry.register_system(
#     MTOWComputation_RHEA, "rhea.loop.mtow", domain=ModelDomain.WEIGHT
# )
#
# # Weight ######################################################################
#
# OpenMDAOSystemRegistry.register_system(
#     Weight_RHEA_IN, "rhea.weight.IN", domain=ModelDomain.WEIGHT
# )
# OpenMDAOSystemRegistry.register_system(
#     Weight_RHEA_sizing, "rhea.weight.sizing", domain=ModelDomain.WEIGHT
# )
# OpenMDAOSystemRegistry.register_system(
#     Weight_RHEA_sizing_OAD, "rhea.weight.hybrid_sizing", domain=ModelDomain.WEIGHT
# )
# # Performance #################################################################
# OpenMDAOSystemRegistry.register_system(
#     SizingFlight_RHEA, "rhea.performances.sizing_flight", domain=ModelDomain.PERFORMANCE
# )
#
# OpenMDAOSystemRegistry.register_system(
#     DOCFlight_RHEA, "rhea.performances.DOC_flight", domain=ModelDomain.PERFORMANCE
# )
#
# OpenMDAOSystemRegistry.register_system(
#     Payload_range, "rhea.performances.payload_range", domain=ModelDomain.PERFORMANCE
# )
#
# OpenMDAOSystemRegistry.register_system(
#     CompleteMissionOutputs, "rhea.performace.mission_outputs", domain=ModelDomain.PERFORMANCE
# )
# OpenMDAOSystemRegistry.register_system(
#     OEI_ceiling, "rhea.performances.OEI", domain=ModelDomain.PERFORMANCE
# )
#
# OpenMDAOSystemRegistry.register_system(
#     TakeOffFlight, "rhea.performances.TOFL", domain=ModelDomain.PERFORMANCE
# )
#
#
# # Propulsion ##################################################################
#
#
#
# rubber_TP_engine_description = """
# Parametric engine model as OpenMDAO component.
#
# """
#
# OpenMDAOSystemRegistry.register_system(
#     CompletePropulsionInputs, "rhea.propulsion.propulsion_inputs", domain=ModelDomain.PROPULSION
# )
#
# OpenMDAOSystemRegistry.register_system(
#     TP_sizing, "rhea.propulsion.turboprop_sizing", domain=ModelDomain.PROPULSION
# )
# OpenMDAOSystemRegistry.register_system(
#     Prop_sizing, "rhea.propulsion.propeller_sizing", domain=ModelDomain.PROPULSION
# )
# OpenMDAOSystemRegistry.register_system(
#     FC_sizing, "rhea.propulsion.fuel_cell_sizing", domain=ModelDomain.PROPULSION
# )
#
# OpenMDAOSystemRegistry.register_system(
#     BoP_sizing, "rhea.propulsion.BoP_sizing", domain=ModelDomain.PROPULSION
# )
#
#
#
# ''' #not necessary
# OpenMDAOSystemRegistry.register_system(
#     OMTPEngineL0Component,
#     "rhea.propulsion.TP_engine_L0",
#     domain=ModelDomain.PROPULSION,
#     desc=rubber_TP_engine_description,
# )
#
#
# OpenMDAOSystemRegistry.register_system(
#     OMTPEngineL1Component,
#     "rhea.propulsion.TP_engine_L1",
#     domain=ModelDomain.PROPULSION,
#     desc="L1",
# )
# '''