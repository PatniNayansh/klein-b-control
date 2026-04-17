"""Tests for the path planner - A* and boustrophedon coverage."""

from __future__ import annotations

import pytest

from klein_b.core.types import GridCell, Waypoint
from klein_b.navigation.mapping import GridSpec, OccupancyGrid
from klein_b.navigation.path_planner import PathPlanner, Rect


def _grid(size: int = 10) -> OccupancyGrid:
    return OccupancyGrid(
        GridSpec(width_cells=size, height_cells=size, resolution_m=1.0,
                 origin_x=0.0, origin_y=0.0)
    )


def test_astar_finds_path_on_empty_grid() -> None:
    planner = PathPlanner()
    grid = _grid()
    path = planner.plan_point_to_point(
        Waypoint(0.5, 0.5), Waypoint(7.5, 7.5), grid,
    )
    assert path is not None
    assert path[-1].x == 7.5 and path[-1].y == 7.5
    assert len(path) >= 2


def test_astar_blocked_goal_returns_none() -> None:
    planner = PathPlanner()
    grid = _grid()
    goal_cell = grid.world_to_grid(7.5, 7.5)
    for _ in range(10):
        grid.update_cell(goal_cell, hit=True)
    assert grid.is_occupied(goal_cell)
    path = planner.plan_point_to_point(
        Waypoint(0.5, 0.5), Waypoint(7.5, 7.5), grid,
    )
    assert path is None


def test_astar_detours_around_wall() -> None:
    planner = PathPlanner()
    grid = _grid()
    # Place a vertical wall at col=5, rows 0..7 (leaves row 8-9 as the gap).
    for r in range(8):
        for _ in range(10):
            grid.update_cell(GridCell(r, 5), hit=True)

    open_path = planner.plan_point_to_point(
        Waypoint(0.5, 0.5), Waypoint(9.5, 0.5), grid,
    )
    assert open_path is not None
    # The path must use at least one cell in rows 8-9 to bypass the wall.
    assert any(wp.y >= 8.0 for wp in open_path)


def test_boustrophedon_generates_alternating_sweeps() -> None:
    planner = PathPlanner()
    grid = _grid(20)
    bounds = Rect(x_min=0.0, y_min=0.0, x_max=10.0, y_max=10.0)
    waypoints = planner.plan_coverage(bounds, grid, swath_width=2.0)

    # 5 swaths × 2 endpoints each = 10 waypoints on a clear grid.
    assert len(waypoints) == 10
    # Alternation check: swath 0 goes east, swath 1 goes west.
    assert waypoints[0].x < waypoints[1].x
    assert waypoints[2].x > waypoints[3].x


def test_boustrophedon_rejects_nonpositive_swath() -> None:
    planner = PathPlanner()
    grid = _grid()
    bounds = Rect(0.0, 0.0, 5.0, 5.0)
    with pytest.raises(ValueError):
        planner.plan_coverage(bounds, grid, swath_width=0.0)
