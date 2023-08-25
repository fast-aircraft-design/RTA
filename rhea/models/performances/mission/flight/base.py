"""Base classes for flight computation."""
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

from typing import List, Union

import pandas as pd
from fastoad.base.flight_point import FlightPoint
from scipy.optimize import root_scalar

from fastoad.models.performances.mission.flight.base import RangedFlight as RF


class RangedFlight(RF):
    """
    Computes a flight so that it covers the specified distance.
    """

    def compute_from(self, start: FlightPoint) -> pd.DataFrame:
        def compute_flight(cruise_distance):
            old_cruise_distance = cruise_distance

            if cruise_distance < 0:  # fix for convergence when GT power too low
                cruise_distance = 100.0

            self.flight.cruise_distance = cruise_distance

            self.flight_points = self.flight.compute_from(start)
            obtained_distance = (
                self.flight_points.iloc[-1].ground_distance
                - self.flight_points.iloc[0].ground_distance
            )

            target = self.flight_distance - obtained_distance
            if old_cruise_distance < 0:  # fix for convergence when GT power too low
                print("WARNING: Cruise distance did not converge")

            return target

        root_scalar(
            compute_flight,
            x0=self.flight_distance * 0.5,
            x1=self.flight_distance * 0.25,
            xtol=0.5e3,
            method="secant",
        )
        return self.flight_points
