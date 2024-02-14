"""
Defines the analysis and plotting functions for postprocessing
"""
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
import numpy as np
import plotly
import plotly.graph_objects as go
from fastoad.io import VariableIO
from plotly.subplots import make_subplots
import matplotlib.pyplot as plt

COLS = plotly.colors.DEFAULT_PLOTLY_COLORS


def plot_double(x, y1, y2, x_label, y1_label, y2_label):

    fig, ax1 = plt.subplots()
    color = "tab:red"
    ax1.set_xlabel(x_label)
    ax1.set_ylabel(y1_label, color=color)
    ax1.plot(x, y1, color=color)

    ax1.tick_params(axis="y", labelcolor=color)
    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

    color = "tab:blue"
    ax2.set_ylabel(y2_label, color=color)  # we already handled the x-label with ax1
    ax2.tick_params(axis="y", labelcolor=color)
    ax2.plot(x, y2, color=color)
    plt.show()


def loading_diagram_plot(
    aircraft_file_path: str,
    name=None,
    fig=None,
    file_formatter=None,
    dist_ref=None,
    pl_ref=None,
) -> go.FigureWidget:
    results = VariableIO(aircraft_file_path, file_formatter).read()

    # Certified ATR72 limits
    index_in_flight = [-27.6, -41.5, -24.7839648, 49.00784, 25.794]
    weight_in_flight = [12000, 18000, 22800, 22800, 12000]

    index_TO_LD = [-29.478, -25.333, -30.5, -15.4724752, 42.00672, 22.109]
    weight_TO_LD = [12800, 15000, 18000, 22800, 22800, 12000]

    # Operational standard limits
    index_op = [-16.665, -14.545, -22.136, -3.8, 32, 13.003, 10.746]
    weight_op = [13500, 14688, 18000, 23000, 23000, 14688, 13500]

    OWE_index = np.asarray(
        results["data:weight:aircraft:operating_empty:CG:index"].value
    )[0]
    Fuel_index = np.asarray(results["data:weight:aircraft:load_case_1:CG:index"].value)[
        0
    ]
    Cargo_index = np.asarray(
        results["data:weight:aircraft:load_case_2:CG:index"].value
    )[0]
    Pax_index = np.asarray(results["data:weight:aircraft:load_case_3:CG:index"].value)[
        0
    ]

    OWE_weight = np.asarray(results["data:weight:aircraft:operating_empty:mass"].value)[
        0
    ]
    Fuel_weight = np.asarray(results["data:weight:aircraft:load_case_1:mass"].value)[0]
    Cargo_weight = np.asarray(results["data:weight:aircraft:load_case_2:mass"].value)[0]
    Pax_weight = np.asarray(results["data:weight:aircraft:load_case_3:mass"].value)[0]

    fig = go.Figure()

    scatter = go.Scatter(
        x=np.array(index_in_flight),
        y=np.array(weight_in_flight),
        line=dict(color="darkblue"),
        name="In flight",
    )
    scatter2 = go.Scatter(
        x=np.array(index_TO_LD),
        y=np.array(weight_TO_LD),
        line=dict(color="red"),
        name="TO/LD",
    )
    scatter3 = go.Scatter(
        x=np.array(index_op),
        y=np.array(weight_op),
        line=dict(color="green"),
        name="Operational",
    )

    scatter4 = go.Scatter(
        x=np.array([OWE_index, Fuel_index, Cargo_index, Pax_index]),
        y=np.array([OWE_weight, Fuel_weight, Cargo_weight, Pax_weight]),
        line=dict(color="Crimson", dash="dash"),
        marker=dict(symbol="diamond"),
        name="Result",
    )

    fig.add_trace(scatter)
    fig.add_trace(scatter2)
    fig.add_trace(scatter3)
    fig.add_trace(scatter4)
    fig = go.FigureWidget(fig)

    fig.update_layout(
        title_text="Loading diagram",
        title_x=0.5,
        xaxis_title="index",
        yaxis_title="Weight",
    )
    return fig


def payload_range_plot(
    aircraft_file_path: str,
    name=None,
    fig=None,
    file_formatter=None,
    dist_ref=None,
    pl_ref=None,
    color=None,
) -> go.FigureWidget:
    variables = VariableIO(aircraft_file_path, file_formatter).read()
    ranges = np.asarray(variables["data:mission:payload_range:range"].value)
    payloads = np.asarray(variables["data:mission:payload_range:payload"].value)

    if fig is None:
        fig = go.Figure()

    if pl_ref is None:

        scatter = go.Scatter(
            x=np.delete(ranges[1:], 1),
            y=np.delete(payloads[1:], 1),
            line=dict(color=color),
            name=name,
        )
        scatter2 = go.Scatter(
            x=np.array([ranges[0], ranges[2], ranges[3]]),
            y=np.array([payloads[0], payloads[2], payloads[3]]),
            line=dict(color=color),
            name=name,
            showlegend=False,
        )

    else:
        scatter = go.Scatter(x=dist_ref, y=pl_ref, line=dict(color=color), name=name)
        scatter2 = go.Scatter(
            x=np.array([0, dist_ref[2]]),
            y=np.array([pl_ref[2], pl_ref[2]]),
            line=dict(color=color, dash="dash"),
            name=name,
            showlegend=False,
        )

    fig.add_trace(scatter)
    fig.add_trace(scatter2)

    fig = go.FigureWidget(fig)

    fig.update_layout(
        title_text="Payload range",
        title_x=0.5,
        xaxis_title="range",
        yaxis_title="Payload",
    )

    return fig


def payload_range_plot2(
    aircraft_file_path: str,
    name=None,
    fig=None,
    file_formatter=None,
    dist_ref=None,
    pl_ref=None,
    color=None,
) -> go.FigureWidget:
    variables = VariableIO(aircraft_file_path, file_formatter).read()
    ranges = np.asarray(variables["data:mission:payload_range:range"].value)
    payloads = np.asarray(variables["data:mission:payload_range:payload"].value)

    if fig is None:
        fig = go.Figure()

    if pl_ref is None:

        scatter = go.Scatter(
            x=np.delete(ranges, 1),
            y=np.delete(payloads, 1),
            line=dict(color=color),
            name=name,
        )

    else:
        scatter = go.Scatter(x=dist_ref, y=pl_ref, line=dict(color=color), name=name)

    fig.add_trace(scatter)

    fig = go.FigureWidget(fig)

    fig.update_layout(
        title_text="Payload range",
        title_x=0.5,
        xaxis_title="range",
        yaxis_title="Payload",
    )

    return fig


def wing_geometry_plot(
    aircraft_file_path: str, name=None, fig=None, file_formatter=None
) -> go.FigureWidget:
    """
    Returns a figure plot of the top view of the wing.
    Different designs can be superposed by providing an existing fig.
    Each design can be provided a name.

    :param aircraft_file_path: path of data file
    :param name: name to give to the trace added to the figure
    :param fig: existing figure to which add the plot
    :param file_formatter: the formatter that defines the format of data file. If not provided, default format will
                           be assumed.
    :return: wing plot figure
    """
    variables = VariableIO(aircraft_file_path, file_formatter).read()

    wing_kink_leading_edge_x = variables[
        "data:geometry:wing:kink:leading_edge:x:local"
    ].value[0]
    wing_tip_leading_edge_x = variables[
        "data:geometry:wing:tip:leading_edge:x:local"
    ].value[0]
    wing_root_y = variables["data:geometry:wing:root:y"].value[0]
    wing_kink_y = variables["data:geometry:wing:kink:y"].value[0]
    wing_tip_y = variables["data:geometry:wing:tip:y"].value[0]
    wing_root_chord = variables["data:geometry:wing:root:chord"].value[0]
    wing_kink_chord = variables["data:geometry:wing:kink:chord"].value[0]
    wing_tip_chord = variables["data:geometry:wing:tip:chord"].value[0]

    # pylint: disable=invalid-name # that's a common naming
    y = np.array(
        [
            0,
            wing_root_y,
            wing_kink_y,
            wing_tip_y,
            wing_tip_y,
            wing_kink_y,
            wing_root_y,
            0,
            0,
        ]
    )
    # pylint: disable=invalid-name # that's a common naming
    y = np.concatenate((-y, y))

    # pylint: disable=invalid-name # that's a common naming
    x = np.array(
        [
            0,
            0,
            wing_kink_leading_edge_x,
            wing_tip_leading_edge_x,
            wing_tip_leading_edge_x + wing_tip_chord,
            wing_kink_leading_edge_x + wing_kink_chord,
            wing_root_chord,
            wing_root_chord,
            0,
        ]
    )
    # pylint: disable=invalid-name # that's a common naming
    x = np.concatenate((x, x))

    if fig is None:
        fig = go.Figure()

    scatter = go.Scatter(x=y, y=x, mode="lines+markers", name=name)

    fig.add_trace(scatter)

    fig.layout = go.Layout(yaxis=dict(scaleanchor="x", scaleratio=1))

    fig = go.FigureWidget(fig)

    fig.update_layout(
        title_text="Wing Geometry",
        title_x=0.5,
        xaxis_title="y",
        yaxis_title="x",
    )

    return fig


# pylint: disable-msg=too-many-locals
def aircraft_geometry_plot(
    aircraft_file_path: str, name=None, fig=None, file_formatter=None
) -> go.FigureWidget:
    """
    Returns a figure plot of the top view of the wing.
    Different designs can be superposed by providing an existing fig.
    Each design can be provided a name.

    :param aircraft_file_path: path of data file
    :param name: name to give to the trace added to the figure
    :param fig: existing figure to which add the plot
    :param file_formatter: the formatter that defines the format of data file. If not provided, default format will
                           be assumed.
    :return: wing plot figure
    """
    variables = VariableIO(aircraft_file_path, file_formatter).read()

    # Wing parameters
    wing_kink_leading_edge_x = variables[
        "data:geometry:wing:kink:leading_edge:x:local"
    ].value[0]
    wing_tip_leading_edge_x = variables[
        "data:geometry:wing:tip:leading_edge:x:local"
    ].value[0]
    wing_root_y = variables["data:geometry:wing:root:y"].value[0]
    wing_kink_y = variables["data:geometry:wing:kink:y"].value[0]
    wing_tip_y = variables["data:geometry:wing:tip:y"].value[0]
    wing_root_chord = variables["data:geometry:wing:root:chord"].value[0]
    wing_kink_chord = variables["data:geometry:wing:kink:chord"].value[0]
    wing_tip_chord = variables["data:geometry:wing:tip:chord"].value[0]

    y_wing = np.array(
        [
            0,
            wing_root_y,
            wing_kink_y,
            wing_tip_y,
            wing_tip_y,
            wing_kink_y,
            wing_root_y,
            0,
            0,
        ]
    )

    x_wing = np.array(
        [
            0,
            0,
            wing_kink_leading_edge_x,
            wing_tip_leading_edge_x,
            wing_tip_leading_edge_x + wing_tip_chord,
            wing_kink_leading_edge_x + wing_kink_chord,
            wing_root_chord,
            wing_root_chord,
            0,
        ]
    )

    # Horizontal Tail parameters
    ht_root_chord = variables["data:geometry:horizontal_tail:root:chord"].value[0]
    ht_tip_chord = variables["data:geometry:horizontal_tail:tip:chord"].value[0]
    ht_span = variables["data:geometry:horizontal_tail:span"].value[0]
    ht_sweep_0 = variables["data:geometry:horizontal_tail:sweep_0"].value[0]

    ht_tip_leading_edge_x = ht_span / 2.0 * np.tan(ht_sweep_0 * np.pi / 180.0)

    y_ht = np.array([0, ht_span / 2.0, ht_span / 2.0, 0.0, 0.0])

    x_ht = np.array(
        [
            0,
            ht_tip_leading_edge_x,
            ht_tip_leading_edge_x + ht_tip_chord,
            ht_root_chord,
            0,
        ]
    )

    # Fuselage parameters
    fuselage_max_width = variables["data:geometry:fuselage:maximum_width"].value[0]
    fuselage_length = variables["data:geometry:fuselage:length"].value[0]
    fuselage_front_length = variables["data:geometry:fuselage:front_length"].value[0]
    fuselage_rear_length = variables["data:geometry:fuselage:rear_length"].value[0]

    x_fuselage = np.array(
        [
            0.0,
            0.0,
            fuselage_front_length,
            fuselage_length - fuselage_rear_length,
            fuselage_length,
            fuselage_length,
        ]
    )

    y_fuselage = np.array(
        [
            0.0,
            fuselage_max_width / 4.0,
            fuselage_max_width / 2.0,
            fuselage_max_width / 2.0,
            fuselage_max_width / 4.0,
            0.0,
        ]
    )

    # CGs
    wing_25mac_x = variables["data:geometry:wing:MAC:at25percent:x"].value[0]
    wing_mac_length = variables["data:geometry:wing:MAC:length"].value[0]
    local_wing_mac_le_x = variables[
        "data:geometry:wing:MAC:leading_edge:x:local"
    ].value[0]
    local_ht_25mac_x = variables[
        "data:geometry:horizontal_tail:MAC:at25percent:x:local"
    ].value[0]
    ht_distance_from_wing = variables[
        "data:geometry:horizontal_tail:MAC:at25percent:x:from_wingMAC25"
    ].value[0]

    x_wing = x_wing + wing_25mac_x - 0.25 * wing_mac_length - local_wing_mac_le_x
    x_ht = x_ht + wing_25mac_x + ht_distance_from_wing - local_ht_25mac_x

    # pylint: disable=invalid-name # that's a common naming
    x = np.concatenate((x_fuselage, x_wing, x_ht))
    # pylint: disable=invalid-name # that's a common naming
    y = np.concatenate((y_fuselage, y_wing, y_ht))

    # pylint: disable=invalid-name # that's a common naming
    y = np.concatenate((-y, y))
    # pylint: disable=invalid-name # that's a common naming
    x = np.concatenate((x, x))

    if fig is None:
        fig = go.Figure()

    scatter = go.Scatter(x=y, y=x, mode="lines+markers", name=name)

    fig.add_trace(scatter)

    fig.layout = go.Layout(yaxis=dict(scaleanchor="x", scaleratio=1))

    fig = go.FigureWidget(fig)

    fig.update_layout(
        title_text="Aircraft Geometry",
        title_x=0.5,
        xaxis_title="y",
        yaxis_title="x",
    )

    return fig


def drag_polar_plot(
    aircraft_file_path: str,
    name=None,
    fig=None,
    file_formatter=None,
    Cl_list=None,
    Cd_list=None,
) -> go.FigureWidget:
    """
    Returns a figure plot of the aircraft drag polar.
    Different designs can be superposed by providing an existing fig.
    Each design can be provided a name.

    :param aircraft_file_path: path of data file
    :param name: name to give to the trace added to the figure
    :param fig: existing figure to which add the plot
    :param file_formatter: the formatter that defines the format of data file. If not provided, default format will
                           be assumed.
    :return: wing plot figure
    """
    if Cl_list is None:
        variables = VariableIO(aircraft_file_path, file_formatter).read()

        # pylint: disable=invalid-name # that's a common naming
        cd = np.asarray(variables["data:aerodynamics:aircraft:cruise:CD"].value)
        # pylint: disable=invalid-name # that's a common naming
        cl = np.asarray(variables["data:aerodynamics:aircraft:cruise:CL"].value)

        # TODO: remove filtering one models provide proper bounds
        cd_short = cd[cd <= 0.3]
        cl_short = cl[cd <= 2.0]
    else:
        cd_short = Cd_list
        cl_short = Cl_list

    if fig is None:
        fig = go.Figure()

    scatter = go.Scatter(x=cd_short, y=cl_short, mode="lines+markers", name=name)

    fig.add_trace(scatter)

    fig = go.FigureWidget(fig)

    fig.update_layout(
        title_text="Drag Polar",
        title_x=0.5,
        xaxis_title="Cd",
        yaxis_title="Cl",
    )

    return fig


def mass_breakdown_bar_plot(
    aircraft_file_path: str, name=None, fig=None, file_formatter=None
) -> go.FigureWidget:
    """
    Returns a figure plot of the aircraft mass breakdown using bar plots.
    Different designs can be superposed by providing an existing fig.
    Each design can be provided a name.

    :param aircraft_file_path: path of data file
    :param name: name to give to the trace added to the figure
    :param fig: existing figure to which add the plot
    :param file_formatter: the formatter that defines the format of data file. If not provided, default format will
                           be assumed.
    :return: bar plot figure
    """
    variables = VariableIO(aircraft_file_path, file_formatter).read()

    systems = variables["data:weight:systems:mass"].value[0]

    furniture = variables["data:weight:furniture:mass"].value[0]

    operational = variables["data:weight:operational:mass"].value[0]

    airframe = variables["data:weight:airframe:mass"].value[0]

    propulsion = variables["data:weight:propulsion:mass"].value[0]

    # pylint: disable=invalid-name # that's a common naming
    MTOW = variables["data:weight:aircraft:MTOW"].value[0]
    # pylint: disable=invalid-name # that's a common naming
    OWE = variables["data:weight:aircraft:OWE"].value[0]
    payload = variables["data:weight:aircraft:payload"].value[0]
    fuel_mission = variables["data:mission:sizing:fuel"].value[0]
    H2_mission = variables["data:mission:sizing:H2"].value[0]

    if fig is None:
        fig = make_subplots(
            rows=1,
            cols=2,
            subplot_titles=(
                "Maximum Take-Off Weight Breakdown",
                "Overall Weight Empty Breakdown",
            ),
        )

    # Same color for each aircraft configuration
    i = len(fig.data)

    # weight_labels = ["MTOW", "OWE", "Fuel - Mission", "Payload"]
    # weight_values = [MTOW, OWE, fuel_mission, payload]
    weight_labels = ["MTOW", "OWE", "Fuel - Mission", "H2 - Mission", "Payload"]
    weight_values = [MTOW, OWE, fuel_mission, H2_mission, payload]
    fig.add_trace(
        go.Bar(
            name="",
            x=weight_labels,
            y=weight_values,
            marker_color=COLS[i],
            showlegend=False,
        ),
        row=1,
        col=1,
    )

    weight_labels = ["Airframe", "Propulsion", "Systems", "Furniture", "Operational"]
    weight_values = [airframe, propulsion, systems, furniture, operational]
    fig.add_trace(
        go.Bar(name=name, x=weight_labels, y=weight_values, marker_color=COLS[i]),
        row=1,
        col=2,
    )

    fig.update_layout(yaxis_title="[kg]")

    return fig


def mass_breakdown_sun_plot(aircraft_file_path: str, file_formatter=None):
    """
    Returns a figure sunburst plot of the mass breakdown.
    On the left a MTOW sunburst and on the right a OWE sunburst.
    Different designs can be superposed by providing an existing fig.
    Each design can be provided a name.

    :param aircraft_file_path: path of data file
    :param file_formatter: the formatter that defines the format of data file. If not provided, default format will
                           be assumed.
    :return: sunburst plot figure
    """
    variables = VariableIO(aircraft_file_path, file_formatter).read()

    systems = variables["data:weight:systems:mass"].value[0]
    C11 = variables["data:weight:systems:auxiliary_power_unit:mass"].value[0]
    C12 = variables[
        "data:weight:systems:electric_systems:electric_generation:mass"
    ].value[0]
    C13 = variables[
        "data:weight:systems:electric_systems:electric_common_installation:mass"
    ].value[0]
    C14 = variables["data:weight:systems:hydraulic_systems:mass"].value[0]
    C15 = variables["data:weight:systems:fire_protection:mass"].value[0]
    C16 = variables["data:weight:systems:flight_furnishing:mass"].value[0]
    C17 = variables["data:weight:systems:automatic_flight_system:mass"].value[0]
    C18 = variables["data:weight:systems:communications:mass"].value[0]
    C19 = variables["data:weight:systems:ECS:mass"].value[0]
    C20 = variables["data:weight:systems:de-icing:mass"].value[0]
    C21 = variables["data:weight:systems:navigation:mass"].value[0]
    C22 = variables["data:weight:systems:flight_controls:mass"].value[0]

    furniture = variables["data:weight:furniture:mass"].value[0]
    D2 = variables["data:weight:furniture:furnishing:mass"].value[0]
    D3 = variables["data:weight:furniture:water:mass"].value[0]
    D4 = variables["data:weight:furniture:interior_integration:mass"].value[0]
    D5 = variables["data:weight:furniture:insulation:mass"].value[0]
    D6 = variables["data:weight:furniture:cabin_lighting:mass"].value[0]
    D7 = variables["data:weight:furniture:seats_crew_accommodation:mass"].value[0]
    D8 = variables["data:weight:furniture:oxygen:mass"].value[0]

    operational = variables["data:weight:operational:mass"].value[0]
    O2 = variables["data:weight:operational:items:passenger_seats:mass"].value[0]
    O3 = variables["data:weight:operational:items:unusable_fuel:mass"].value[0]
    O4 = variables["data:weight:operational:items:documents_toolkit:mass"].value[0]
    O5 = variables["data:weight:operational:items:galley_structure:mass"].value[0]
    O6 = variables["data:weight:operational:equipment:crew:mass"].value[0]
    O7 = variables["data:weight:operational:equipment:others:mass"].value[0]

    airframe = variables["data:weight:airframe:mass"].value[0]
    wing = variables["data:weight:airframe:wing:mass"].value[0]
    fuselage = variables["data:weight:airframe:fuselage:mass"].value[0]
    h_tail = variables["data:weight:airframe:horizontal_tail:mass"].value[0]
    v_tail = variables["data:weight:airframe:vertical_tail:mass"].value[0]
    landing_gear_1 = variables["data:weight:airframe:landing_gear:main:mass"].value[0]
    landing_gear_2 = variables["data:weight:airframe:landing_gear:front:mass"].value[0]
    engine_nacelle = variables["data:weight:airframe:pylon:mass"].value[0]

    propulsion = variables["data:weight:propulsion:mass"].value[0]
    B1 = variables["data:weight:propulsion:engine:mass"].value[0]
    B2 = variables["data:weight:propulsion:propeller:mass"].value[0]
    B3 = variables["data:weight:propulsion:engine_controls_instrumentation:mass"].value[
        0
    ]
    B4 = variables["data:weight:propulsion:fuel_system:mass"].value[0]

    try:
        B5 = variables["data:weight:propulsion:electric_systems:fuel_cell:mass"].value[
            0
        ]
        B6 = variables["data:weight:propulsion:electric_systems:motor:mass"].value[0]
        B12 = variables[
            "data:weight:propulsion:electric_systems:power_electronics:mass"
        ].value[0]
        B7 = variables["data:weight:propulsion:electric_systems:cooling:mass"].value[0]
        B10 = variables[
            "data:weight:propulsion:electric_systems:H2_distribution:mass"
        ].value[0]
        B11 = variables["data:weight:propulsion:electric_systems:cables:mass"].value[0]
        B8 = variables["data:weight:propulsion:electric_systems:battery:mass"].value[0]
        B9 = variables["data:weight:propulsion:electric_systems:H2_storage:mass"].value[
            0
        ]
    except ValueError:
        B5 = 0
        B6 = 0
        B7 = 0
        B10 = 0
        B11 = 0
        B8 = 0
        B9 = 0
        B12 = 0

    margin_policy = variables["data:weight:aircraft_empty:contingency"].value[0]

    MTOW = variables["data:weight:aircraft:MTOW"].value[0]
    OWE = variables["data:weight:aircraft:OWE"].value[0]
    payload = variables["data:weight:aircraft:payload"].value[0]
    fuel_mission = variables["data:mission:sizing:fuel"].value[0]
    H2_mission = variables["data:mission:sizing:H2"].value[0]

    # TODO: Deal with this in a more generic manner ?
    if round(MTOW, 6) == round(OWE + payload + fuel_mission, 6):
        MTOW = OWE + payload + fuel_mission

    fig = make_subplots(
        1,
        2,
        specs=[[{"type": "domain"}, {"type": "domain"}]],
    )

    # fig.add_trace(
    #     go.Sunburst(
    #         labels=[
    #             "MTOW" + "<br>" + str(int(MTOW)) + " [kg]",
    #             "payload"
    #             + "<br>"
    #             + str(int(payload))
    #             + " [kg] ("
    #             + str(round(payload / MTOW * 100, 1))
    #             + "%)",
    #             "fuel_mission"
    #             + "<br>"
    #             + str(int(fuel_mission))
    #             + " [kg] ("
    #             + str(round(fuel_mission / MTOW * 100, 1))
    #             + "%)",
    #             "OWE" + "<br>" + str(int(OWE)) + " [kg] (" + str(round(OWE / MTOW * 100, 1)) + "%)",
    #         ],
    #         parents=[
    #             "",
    #             "MTOW" + "<br>" + str(int(MTOW)) + " [kg]",
    #             "MTOW" + "<br>" + str(int(MTOW)) + " [kg]",
    #             "MTOW" + "<br>" + str(int(MTOW)) + " [kg]",
    #         ],
    #         values=[MTOW, payload, fuel_mission, OWE],
    #         branchvalues="total",
    #     ),
    #     1,
    #     1,
    # )
    fig.add_trace(
        go.Sunburst(
            labels=[
                "MTOW" + "<br>" + str(int(MTOW)) + " [kg]",
                "payload"
                + "<br>"
                + str(int(payload))
                + " [kg] ("
                + str(round(payload / MTOW * 100, 1))
                + "%)",
                "fuel_mission"
                + "<br>"
                + str(int(fuel_mission))
                + " [kg] ("
                + str(round(fuel_mission / MTOW * 100, 1))
                + "%)",
                "H2_mission"
                + "<br>"
                + str(int(H2_mission))
                + " [kg] ("
                + str(round(H2_mission / MTOW * 100, 1))
                + "%)",
                "OWE"
                + "<br>"
                + str(int(OWE))
                + " [kg] ("
                + str(round(OWE / MTOW * 100, 1))
                + "%)",
            ],
            parents=[
                "",
                "MTOW" + "<br>" + str(int(MTOW)) + " [kg]",
                "MTOW" + "<br>" + str(int(MTOW)) + " [kg]",
                "MTOW" + "<br>" + str(int(MTOW)) + " [kg]",
                "MTOW" + "<br>" + str(int(MTOW)) + " [kg]",
            ],
            values=[MTOW, payload, fuel_mission, H2_mission, OWE],
            branchvalues="total",
        ),
        1,
        1,
    )

    airframe_str = (
        "airframe"
        + "<br>"
        + str(int(airframe))
        + " [kg] ("
        + str(round(airframe / OWE * 100, 1))
        + "%)"
    )
    propulsion_str = (
        "propulsion"
        + "<br>"
        + str(int(propulsion))
        + " [kg] ("
        + str(round(propulsion / OWE * 100, 1))
        + "%)"
    )
    systems_str = (
        "systems"
        + "<br>"
        + str(int(systems))
        + " [kg] ("
        + str(round(systems / OWE * 100, 1))
        + "%)"
    )
    furniture_str = (
        "furniture"
        + "<br>"
        + str(int(furniture))
        + " [kg] ("
        + str(round(furniture / OWE * 100, 1))
        + "%)"
    )
    operational_str = (
        "operational"
        + "<br>"
        + str(int(operational))
        + " [kg] ("
        + str(round(operational / OWE * 100, 1))
        + "%)"
    )

    contingency_str = (
        "margin policy"
        + "<br>"
        + str(int(margin_policy))
        + " [kg] ("
        + str(round(margin_policy / OWE * 100, 1))
        + "%)"
    )

    fig.add_trace(
        go.Sunburst(
            labels=[
                "OWE" + "<br>" + str(int(OWE)) + " [kg]",
                airframe_str,
                propulsion_str,
                systems_str,
                furniture_str,
                operational_str,
                # contingency_str,
                "wing",
                "fuselage",
                "horizontal_tail",
                "vertical_tail",
                "landing_gear_main",
                "landing_gear_front",
                "nacelle",
                "engine",
                "propeller",
                "engine_controls",
                "fuel_system",
                "fuel_cell",
                "motor",
                "power_electronics",
                "cooling",
                "H2_distribution",
                "cables",
                "battery",
                "H2_storage",
                "APU",
                "electric_generation",
                "electric_installation",
                "hydraulic_systems",
                "fire_protection",
                "flight_furnishing",
                "automatic_flight_system",
                "communications",
                "ECS",
                "de-icing",
                "navigation",
                "flight_controls",
                "furnishing",
                "water",
                "interior_integration",
                "insulation",
                "cabin_lighting",
                "seats_crew",
                "oxygen",
                "passenger_seats",
                "unusable_fuel",
                "documents_toolkit",
                "galley_structure",
                "crew",
                "equipments",
            ],
            parents=[
                "",
                "OWE" + "<br>" + str(int(OWE)) + " [kg]",
                "OWE" + "<br>" + str(int(OWE)) + " [kg]",
                "OWE" + "<br>" + str(int(OWE)) + " [kg]",
                "OWE" + "<br>" + str(int(OWE)) + " [kg]",
                "OWE" + "<br>" + str(int(OWE)) + " [kg]",
                # "OWE" + "<br>" + str(int(OWE)) + " [kg]",
                airframe_str,
                airframe_str,
                airframe_str,
                airframe_str,
                airframe_str,
                airframe_str,
                airframe_str,
                propulsion_str,
                propulsion_str,
                propulsion_str,
                propulsion_str,
                propulsion_str,
                propulsion_str,
                propulsion_str,
                propulsion_str,
                propulsion_str,
                propulsion_str,
                propulsion_str,
                propulsion_str,
                systems_str,
                systems_str,
                systems_str,
                systems_str,
                systems_str,
                systems_str,
                systems_str,
                systems_str,
                systems_str,
                systems_str,
                systems_str,
                systems_str,
                # "furniture", "furniture", "furniture", "furniture", "furniture",
                furniture_str,
                furniture_str,
                furniture_str,
                furniture_str,
                furniture_str,
                furniture_str,
                furniture_str,
                operational_str,
                operational_str,
                operational_str,
                operational_str,
                operational_str,
                operational_str,
            ],
            values=[
                OWE,
                airframe,
                propulsion,
                systems,
                furniture,
                operational,
                # margin_policy,
                wing,
                fuselage,
                h_tail,
                v_tail,
                landing_gear_1,
                landing_gear_2,
                engine_nacelle,
                B1,
                B2,
                B3,
                B4,
                B5,
                B6,
                B12,
                B7,
                B10,
                B11,
                B8,
                B9,
                C11,
                C12,
                C13,
                C14,
                C15,
                C16,
                C17,
                C18,
                C19,
                C20,
                C21,
                C22,
                D2,
                D3,
                D4,
                D5,
                D6,
                D7,
                D8,
                O2,
                O3,
                O4,
                O5,
                O6,
                O7,
            ],
            branchvalues="total",
        ),
        1,
        2,
    )

    fig.update_layout(title_text="Mass Breakdown", title_x=0.5)

    return fig


def drag_breakdown_sun_plot(aircraft_file_path: str, Cl_cruise, file_formatter=None):
    """
    Returns a figure sunburst plot of the mass breakdown.
    On the left a MTOW sunburst and on the right a OWE sunburst.
    Different designs can be superposed by providing an existing fig.
    Each design can be provided a name.

    :param aircraft_file_path: path of data file
    :param file_formatter: the formatter that defines the format of data file. If not provided, default format will
                           be assumed.
    :return: sunburst plot figure
    """
    variables = VariableIO(aircraft_file_path, file_formatter).read()
    Cl_rhea = np.asarray(variables["data:aerodynamics:aircraft:cruise:CL"].value)
    # Cd_rhea =  np.asarray(variables["data:aerodynamics:aircraft:cruise:CD"].value)

    # Cl_cruise=0.45
    idx = list(Cl_rhea).index(Cl_cruise)
    Cd_cruise = (
        np.asarray(variables["data:aerodynamics:aircraft:cruise:CD"].value[idx])
        * 10**4
    )

    k_e = np.asarray(
        variables["data:aerodynamics:aircraft:cruise:induced_drag_coefficient"].value
    )[0]
    k_winglet = np.asarray(
        variables["tuning:aerodynamics:aircraft:cruise:CD:winglet_effect:k"].value
    )[0]
    Cdi_rhea = k_e * Cl_cruise**2 * 10**4 * k_winglet

    Cd0_rhea_w = (
        np.asarray(variables["data:aerodynamics:wing:cruise:CD0"].value[idx]) * 10**4
    )
    Cd0_rhea_f = (
        np.asarray(variables["data:aerodynamics:fuselage:cruise:CD0"].value[idx])
        * 10**4
    )
    Cd0_rhea_ht = (
        np.asarray(variables["data:aerodynamics:horizontal_tail:cruise:CD0"].value)[0]
        * 10**4
    )
    Cd0_rhea_vt = (
        np.asarray(variables["data:aerodynamics:vertical_tail:cruise:CD0"].value)[0]
        * 10**4
    )
    Cd0_rhea_nac = (
        np.asarray(variables["data:aerodynamics:nacelles:cruise:CD0"].value)[0]
        * 10**4
    )

    Cd_0 = (
        np.asarray(variables["data:aerodynamics:aircraft:cruise:CD0"].value[idx])
        * 10**4
    )
    Cd0_parasitic = Cd_0 - (
        Cd0_rhea_nac + Cd0_rhea_vt + Cd0_rhea_ht + Cd0_rhea_f + Cd0_rhea_w
    )

    Cd_c = (
        np.asarray(
            variables["data:aerodynamics:aircraft:cruise:CD:compressibility"].value[idx]
        )
        * 10**4
    )
    Cd_trim = (
        np.asarray(variables["data:aerodynamics:aircraft:cruise:CD:trim"].value[idx])
        * 10**4
    )

    # cd0_tot=Cd0_rhea_w+Cd0_rhea_f+Cd0_rhea_ht+Cd0_rhea_vt+Cd0_rhea_nac

    fig = go.Figure(
        go.Sunburst(
            labels=[
                "Cd",
                "Cd0",
                "Cdi",
                "Cdc",
                "Cdtrim",
                "Cd0_w",
                "Cd0_f",
                "Cd0_nac",
                "Cd0_ht",
                "Cd0_vt",
                "Cd0_paras",
            ],
            parents=[
                "",
                "Cd",
                "Cd",
                "Cd",
                "Cd",
                "Cd0",
                "Cd0",
                "Cd0",
                "Cd0",
                "Cd0",
                "Cd0",
            ],
            values=[
                Cd_cruise,
                Cd_0,
                Cdi_rhea,
                Cd_c,
                Cd_trim,
                Cd0_rhea_w,
                Cd0_rhea_f,
                Cd0_rhea_nac,
                Cd0_rhea_ht,
                Cd0_rhea_vt,
                Cd0_parasitic,
            ],
            branchvalues="total",
        )
    )

    fig.update_layout(
        title_text="Cruise Drag Breakdown", title_x=0.5
    )  # ,margin = dict(t=0, l=0, r=0, b=0))

    return fig


# def drag_breakdown_bar_plot(aircraft_file_path: str, file_formatter=None):
#     """
#     Returns a figure sunburst plot of the mass breakdown.
#     On the left a MTOW sunburst and on the right a OWE sunburst.
#     Different designs can be superposed by providing an existing fig.
#     Each design can be provided a name.

#     :param aircraft_file_path: path of data file
#     :param file_formatter: the formatter that defines the format of data file. If not provided, default format will
#                            be assumed.
#     :return: sunburst plot figure
#     """
#     variables = VariableIO(aircraft_file_path, file_formatter).read()
#     Cl_rhea =np.asarray(variables["data:aerodynamics:aircraft:cruise:CL"].value)
#     # Cd_rhea =  np.asarray(variables["data:aerodynamics:aircraft:cruise:CD"].value)

#     Cl_cruise=0.6
#     idx= list(Cl_rhea).index(Cl_cruise)
#     Cd_cruise =  np.asarray(variables["data:aerodynamics:aircraft:cruise:CD"].value[idx])*10**4

#     k_e =  np.asarray(variables["data:aerodynamics:aircraft:cruise:induced_drag_coefficient"].value)[0]
#     Cdi_rhea =k_e* Cl_cruise**2*10**4

#     Cd0_rhea_w =  np.asarray(variables["data:aerodynamics:wing:cruise:CD0"].value[idx])*10**4
#     Cd0_rhea_f =  np.asarray(variables["data:aerodynamics:fuselage:cruise:CD0"].value[idx])*10**4
#     Cd0_rhea_ht =  np.asarray(variables["data:aerodynamics:horizontal_tail:cruise:CD0"].value)[0]*10**4
#     Cd0_rhea_vt =  np.asarray(variables["data:aerodynamics:vertical_tail:cruise:CD0"].value)[0]*10**4
#     Cd0_rhea_nac =  np.asarray(variables["data:aerodynamics:nacelles:cruise:CD0"].value)[0]*10**4

#     Cd_0 =  np.asarray(variables["data:aerodynamics:aircraft:cruise:CD0"].value[idx])*10**4

#     Cd_c =  np.asarray(variables["data:aerodynamics:aircraft:cruise:CD:compressibility"].value[idx])*10**4
#     Cd_trim =  np.asarray(variables["data:aerodynamics:aircraft:cruise:CD:trim"].value[idx])*10**4


#     components=['Cdtot','Cdo', 'Cdi', 'Cdc','Cdtrim']

#     fig = go.Figure(data=[
#         go.Bar(name='wing', x=components, y=[0,Cd0_rhea_w, 0, 0,0 ]),
#         go.Bar(name='fuselage', x=components, y=[0,Cd0_rhea_f, 0, 0,0]),
#         go.Bar(name='nacelles', x=components, y=[0,Cd0_rhea_nac, 0, 0,0]),
#         go.Bar(name='HT', x=components, y=[0,Cd0_rhea_ht, 0, 0,0]),
#         go.Bar(name='VT', x=components, y=[0,Cd0_rhea_vt, 0, 0,0]),
#         go.Bar(name='aircraft', x=components, y=[Cd_cruise,0, Cdi_rhea, Cd_c,Cd_trim])


#     ])
#     # Change the bar mode
#     fig.update_layout(barmode='stack', title='Cruise drag breakdown',
#         yaxis=dict(
#             title='CD 10**4',
#             titlefont_size=16,
#             tickfont_size=14,
#         ),title_x=0.5)

#     return fig
def drag_breakdown_bar_plot(
    aircraft_file_path: str, Cl_cruise, name=None, fig=None, file_formatter=None
) -> go.FigureWidget:
    """
    Returns a figure sunburst plot of the mass breakdown.
    On the left a MTOW sunburst and on the right a OWE sunburst.
    Different designs can be superposed by providing an existing fig.
    Each design can be provided a name.

    :param aircraft_file_path: path of data file
    :param file_formatter: the formatter that defines the format of data file. If not provided, default format will
                           be assumed.
    :return: sunburst plot figure
    """

    if aircraft_file_path == "ATR72_ref_data" and Cl_cruise == 0.45:
        Cd_cruise = 378
        Cd0_rhea_w = 93
        Cd0_rhea_f = 100.6
        Cd0_rhea_nac = 29
        Cd0_rhea_ht = 15.5
        Cd0_rhea_vt = 18.6
        Cd0_parasitic = 61.3
        Cdi_rhea = 60
        Cd_c = 0
        Cd_trim = 0
        Cd_0 = 256.7 + Cd0_parasitic
    else:
        variables = VariableIO(aircraft_file_path, file_formatter).read()
        Cl_rhea = np.asarray(variables["data:aerodynamics:aircraft:cruise:CL"].value)
        # Cd_rhea =  np.asarray(variables["data:aerodynamics:aircraft:cruise:CD"].value)

        # Cl_cruise=0.55
        idx = list(Cl_rhea).index(Cl_cruise)
        Cd_cruise = (
            np.asarray(variables["data:aerodynamics:aircraft:cruise:CD"].value[idx])
            * 10**4
        )

        k_e = np.asarray(
            variables[
                "data:aerodynamics:aircraft:cruise:induced_drag_coefficient"
            ].value
        )[0]
        k_winglet = np.asarray(
            variables["tuning:aerodynamics:aircraft:cruise:CD:winglet_effect:k"].value
        )[0]

        Cdi_rhea = k_e * Cl_cruise**2 * 10**4 * k_winglet

        Cd0_rhea_w = (
            np.asarray(variables["data:aerodynamics:wing:cruise:CD0"].value[idx])
            * 10**4
        )
        Cd0_rhea_f = (
            np.asarray(variables["data:aerodynamics:fuselage:cruise:CD0"].value[idx])
            * 10**4
        )
        Cd0_rhea_ht = (
            np.asarray(variables["data:aerodynamics:horizontal_tail:cruise:CD0"].value)[
                0
            ]
            * 10**4
        )
        Cd0_rhea_vt = (
            np.asarray(variables["data:aerodynamics:vertical_tail:cruise:CD0"].value)[0]
            * 10**4
        )
        Cd0_rhea_nac = (
            np.asarray(variables["data:aerodynamics:nacelles:cruise:CD0"].value)[0]
            * 10**4
        )

        Cd_0 = (
            np.asarray(variables["data:aerodynamics:aircraft:cruise:CD0"].value[idx])
            * 10**4
        )
        Cd0_parasitic = Cd_0 - (
            Cd0_rhea_nac + Cd0_rhea_vt + Cd0_rhea_ht + Cd0_rhea_f + Cd0_rhea_w
        )
        Cd_c = (
            np.asarray(
                variables["data:aerodynamics:aircraft:cruise:CD:compressibility"].value[
                    idx
                ]
            )
            * 10**4
        )
        Cd_trim = (
            np.asarray(
                variables["data:aerodynamics:aircraft:cruise:CD:trim"].value[idx]
            )
            * 10**4
        )

    components = [
        "Cdtot",
        "Cd0 - Cd0_para",
        "Cd0_w",
        "Cd0_f",
        "Cd0_nac",
        "Cd0_ht",
        "Cd0_vt",
        "Cd0_para",
        "Cdi",
        "Cdc",
        "Cdtrim",
    ]
    if fig is None:
        fig = go.Figure()

    fig.add_trace(
        go.Bar(
            x=components,
            y=[
                Cd_cruise,
                Cd_0 - Cd0_parasitic,
                Cd0_rhea_w,
                Cd0_rhea_f,
                Cd0_rhea_nac,
                Cd0_rhea_ht,
                Cd0_rhea_vt,
                Cd0_parasitic,
                Cdi_rhea,
                Cd_c,
                Cd_trim,
            ],
            name=name,
        )
    )
    # Change the bar mode
    fig = go.FigureWidget(fig)

    fig.update_layout(
        barmode="group",
        title="Cruise drag breakdown",
        title_x=0.5,
        yaxis=dict(
            title="CD 10**4",
            titlefont_size=16,
            tickfont_size=14,
        ),
    )

    return fig


def wetted_surface_bar_plot(
    aircraft_file_path: str, name=None, fig=None, file_formatter=None
) -> go.FigureWidget:
    """
    Returns a figure plot of the top view of the wing.
    Different designs can be superposed by providing an existing fig.
    Each design can be provided a name.

    :param aircraft_file_path: path of data file
    :param name: name to give to the trace added to the figure
    :param fig: existing figure to which add the plot
    :param file_formatter: the formatter that defines the format of data file. If not provided, default format will
                           be assumed.
    :return: wing plot figure
    """
    variables = VariableIO(aircraft_file_path, file_formatter).read()

    S_wet_w = np.asarray(variables["data:geometry:wing:wetted_area"].value)[0]
    S_wet_f = np.asarray(variables["data:geometry:fuselage:wetted_area"].value)[0]
    S_wet_ht = np.asarray(variables["data:geometry:horizontal_tail:wetted_area"].value)[
        0
    ]
    S_wet_vt = np.asarray(variables["data:geometry:vertical_tail:wetted_area"].value)[0]
    S_wet_nac = np.asarray(
        variables["data:geometry:propulsion:nacelle:wetted_area"].value
    )[0]
    S_wet_tot = np.asarray(variables["data:geometry:aircraft:wetted_area"].value)[0]

    if fig is None:
        fig = go.Figure()

    areas = ["wing", "fuselage", "ht", "vt", "nacelle", "aircraft"]

    fig.add_trace(
        go.Bar(
            x=areas,
            y=[S_wet_w, S_wet_f, S_wet_ht, S_wet_vt, S_wet_nac, S_wet_tot],
            name=name,
        )
    )

    fig = go.FigureWidget(fig)

    fig.update_layout(
        title_text="Wet Areas", title_x=0.5, yaxis_title="m**2", barmode="group"
    )
    return fig
