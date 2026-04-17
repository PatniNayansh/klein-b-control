"""Tests for OccupancyGrid - log-odds updates and lidar integration."""

from __future__ import annotations

import math

from klein_b.core.types import GridCell, LiDARPoint, Pose2D
from klein_b.navigation.mapping import LOG_ODDS_CLAMP, GridSpec, OccupancyGrid


def _grid() -> OccupancyGrid:
    return OccupancyGrid(
        GridSpec(width_cells=20, height_cells=20, resolution_m=0.5,
                 origin_x=-5.0, origin_y=-5.0)
    )


def test_world_to_grid_roundtrip_within_resolution() -> None:
    grid = _grid()
    x, y = 1.25, -2.75
    cell = grid.world_to_grid(x, y)
    wx, wy = grid.grid_to_world(cell)
    assert abs(wx - x) < grid.spec.resolution_m
    assert abs(wy - y) < grid.spec.resolution_m


def test_log_odds_start_at_zero() -> None:
    grid = _grid()
    assert grid.log_odds.sum() == 0


def test_hit_raises_occupancy_miss_lowers_it() -> None:
    grid = _grid()
    cell = GridCell(5, 5)
    grid.update_cell(cell, hit=True)
    assert grid.is_free(cell) is False
    for _ in range(10):
        grid.update_cell(cell, hit=True)
    assert grid.is_occupied(cell)
    assert grid.log_odds[cell.row, cell.col] <= LOG_ODDS_CLAMP

    # Now drown it with misses — should swing back to free.
    for _ in range(40):
        grid.update_cell(cell, hit=False)
    assert grid.is_free(cell)
    assert grid.log_odds[cell.row, cell.col] >= -LOG_ODDS_CLAMP


def test_out_of_bounds_update_is_silent() -> None:
    grid = _grid()
    # Should not raise and should not modify anything.
    grid.update_cell(GridCell(-1, -1), hit=True)
    grid.update_cell(GridCell(9999, 9999), hit=True)
    assert grid.log_odds.sum() == 0


def test_integrate_scan_marks_hit_and_free_cells() -> None:
    grid = _grid()
    pose = Pose2D(x=0.0, y=0.0, heading=0.0, timestamp=0.0)
    # Single ray straight down +x, 2 meters.
    scan = [LiDARPoint(range=2.0, angle=0.0, intensity=1.0)]

    # Apply many times so the log-odds crosses the occupied/free thresholds.
    for _ in range(5):
        grid.integrate_scan(pose, scan, max_range=10.0)

    endpoint_cell = grid.world_to_grid(2.0, 0.0)
    origin_cell = grid.world_to_grid(0.0, 0.0)
    assert grid.is_occupied(endpoint_cell)
    assert grid.is_free(origin_cell)


def test_integrate_scan_with_max_range_only_misses() -> None:
    grid = _grid()
    pose = Pose2D(x=0.0, y=0.0, heading=0.0, timestamp=0.0)
    # Range beyond max_range → clipped; endpoint is NOT a hit.
    scan = [LiDARPoint(range=100.0, angle=0.0, intensity=0.0)]
    for _ in range(5):
        grid.integrate_scan(pose, scan, max_range=2.0)

    endpoint_cell = grid.world_to_grid(2.0, 0.0)
    assert not grid.is_occupied(endpoint_cell)


def test_infinite_and_zero_ranges_ignored() -> None:
    grid = _grid()
    pose = Pose2D(x=0.0, y=0.0, heading=0.0, timestamp=0.0)
    scan = [
        LiDARPoint(range=math.inf, angle=0.0, intensity=0.0),
        LiDARPoint(range=0.0, angle=0.0, intensity=0.0),
        LiDARPoint(range=-1.0, angle=0.0, intensity=0.0),
    ]
    grid.integrate_scan(pose, scan)
    assert grid.log_odds.sum() == 0
