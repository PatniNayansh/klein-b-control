"""A* for point-to-point and boustrophedon (lawnmower) for full coverage.

Both planners consume the OccupancyGrid. A* treats `is_occupied` cells
as blocked; free and unknown cells are both walkable (I want the boat
to be willing to explore).
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

from klein_b.core.types import GridCell, Waypoint
from klein_b.navigation.mapping import OccupancyGrid

SQRT2 = math.sqrt(2.0)

# 8-connected neighbourhood: (row_delta, col_delta, step_cost)
NEIGHBOR_OFFSETS: tuple[tuple[int, int, float], ...] = (
    (-1,  0, 1.0), (1,  0, 1.0), (0, -1, 1.0), (0,  1, 1.0),
    (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2),
)


@dataclass(frozen=True, slots=True)
class Rect:
    x_min: float
    y_min: float
    x_max: float
    y_max: float


class PathPlanner:
    def plan_point_to_point(
        self,
        start: Waypoint,
        goal: Waypoint,
        grid: OccupancyGrid,
    ) -> list[Waypoint] | None:
        """Classic A* on the grid. Returns waypoints or None if unreachable."""
        start_cell = grid.world_to_grid(start.x, start.y)
        goal_cell = grid.world_to_grid(goal.x, goal.y)
        if grid.is_occupied(goal_cell):
            return None

        # heapq can't compare GridCells directly, so add a counter as a tiebreaker.
        tiebreaker = 0
        frontier: list[tuple[float, int, GridCell]] = [(0.0, tiebreaker, start_cell)]
        came_from: dict[GridCell, GridCell] = {}
        cost_to_reach: dict[GridCell, float] = {start_cell: 0.0}

        while frontier:
            _, _, current_cell = heapq.heappop(frontier)
            if current_cell == goal_cell:
                return self._reconstruct_path(came_from, current_cell, grid, goal)

            for row_delta, col_delta, step_cost in NEIGHBOR_OFFSETS:
                neighbor = GridCell(
                    current_cell.row + row_delta,
                    current_cell.col + col_delta,
                )
                if not grid.in_bounds(neighbor) or grid.is_occupied(neighbor):
                    continue
                new_cost = cost_to_reach[current_cell] + step_cost
                if new_cost < cost_to_reach.get(neighbor, math.inf):
                    came_from[neighbor] = current_cell
                    cost_to_reach[neighbor] = new_cost
                    priority = new_cost + _octile_distance(neighbor, goal_cell)
                    tiebreaker += 1
                    heapq.heappush(frontier, (priority, tiebreaker, neighbor))
        return None

    def plan_coverage(
        self,
        bounds: Rect,
        grid: OccupancyGrid,
        swath_width: float,
    ) -> list[Waypoint]:
        """Lawnmower pattern across `bounds` with `swath_width`-meter spacing.

        Alternates east-west passes. If the start or end of a pass is inside
        an obstacle, we try sliding it inward a bit; if that still doesn't
        clear, the pass is dropped.
        """
        if swath_width <= 0:
            raise ValueError("swath_width must be positive")

        waypoints: list[Waypoint] = []
        sweep_y = bounds.y_min + swath_width / 2.0
        going_east = True
        while sweep_y <= bounds.y_max - swath_width / 2.0 + 1e-9:
            if going_east:
                sweep_start_x, sweep_end_x = bounds.x_min, bounds.x_max
            else:
                sweep_start_x, sweep_end_x = bounds.x_max, bounds.x_min

            start_wp = self._slide_into_free(
                sweep_start_x, sweep_y,
                toward=(sweep_end_x, sweep_y),
                grid=grid,
            )
            end_wp = self._slide_into_free(
                sweep_end_x, sweep_y,
                toward=(sweep_start_x, sweep_y),
                grid=grid,
            )
            if start_wp is not None and end_wp is not None:
                waypoints.append(start_wp)
                waypoints.append(end_wp)

            sweep_y += swath_width
            going_east = not going_east
        return waypoints

    def _reconstruct_path(
        self,
        came_from: dict[GridCell, GridCell],
        end_cell: GridCell,
        grid: OccupancyGrid,
        goal_waypoint: Waypoint,
    ) -> list[Waypoint]:
        cells_backwards = [end_cell]
        while cells_backwards[-1] in came_from:
            cells_backwards.append(came_from[cells_backwards[-1]])
        cells_backwards.reverse()
        waypoints = [Waypoint(*grid.grid_to_world(cell)) for cell in cells_backwards]
        # Replace the last waypoint with the actual goal so we land exactly on it.
        if waypoints:
            waypoints[-1] = goal_waypoint
        return waypoints

    @staticmethod
    def _slide_into_free(
        x: float, y: float,
        *,
        toward: tuple[float, float],
        grid: OccupancyGrid,
    ) -> Waypoint | None:
        """If (x,y) is in an obstacle, slide toward `toward` until it's clear."""
        toward_x, toward_y = toward
        for shrink_factor in (0.0, 0.25, 0.5, 0.75):
            probe_x = x + (toward_x - x) * shrink_factor
            probe_y = y + (toward_y - y) * shrink_factor
            if not grid.is_occupied(grid.world_to_grid(probe_x, probe_y)):
                return Waypoint(probe_x, probe_y)
        return None


def _octile_distance(cell: GridCell, goal: GridCell) -> float:
    """Octile distance - admissible heuristic for 8-connected grids."""
    row_distance = abs(cell.row - goal.row)
    col_distance = abs(cell.col - goal.col)
    return (row_distance + col_distance) + (SQRT2 - 2.0) * min(row_distance, col_distance)
