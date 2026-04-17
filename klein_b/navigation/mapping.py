"""Log-odds occupancy grid with Bresenham raycasting.

Log-odds is the standard trick for occupancy maps: instead of storing a
probability [0,1] and multiplying every update, you store log(p/(1-p))
and just ADD. Saturates nicely too. (I learned this from the Probabilistic
Robotics book.)
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from klein_b.core.types import GridCell, LiDARPoint, Pose2D

# Standard values - p(hit)=0.7, p(miss)=0.4. Clamp so a cell can recover.
LOG_ODDS_HIT = math.log(0.70 / 0.30)   # ~+0.85
LOG_ODDS_MISS = math.log(0.40 / 0.60)  # ~-0.41
LOG_ODDS_CLAMP = 10.0
OCCUPIED_THRESHOLD = 2.0
FREE_THRESHOLD = -2.0


@dataclass(frozen=True, slots=True)
class GridSpec:
    width_cells: int
    height_cells: int
    resolution_m: float
    origin_x: float  # world (x,y) of the (row=0, col=0) corner
    origin_y: float


class OccupancyGrid:
    def __init__(self, spec: GridSpec) -> None:
        self.spec = spec
        self._log_odds = np.zeros((spec.height_cells, spec.width_cells), dtype=np.float32)

    @property
    def log_odds(self) -> np.ndarray:
        return self._log_odds

    def world_to_grid(self, x: float, y: float) -> GridCell:
        """World (x, y) -> grid indices. Clamps to bounds."""
        col = int((x - self.spec.origin_x) / self.spec.resolution_m)
        row = int((y - self.spec.origin_y) / self.spec.resolution_m)
        col = max(0, min(self.spec.width_cells - 1, col))
        row = max(0, min(self.spec.height_cells - 1, row))
        return GridCell(row=row, col=col)

    def grid_to_world(self, cell: GridCell) -> tuple[float, float]:
        """Center of the given cell in world coords."""
        x = self.spec.origin_x + (cell.col + 0.5) * self.spec.resolution_m
        y = self.spec.origin_y + (cell.row + 0.5) * self.spec.resolution_m
        return x, y

    def in_bounds(self, cell: GridCell) -> bool:
        return 0 <= cell.row < self.spec.height_cells and 0 <= cell.col < self.spec.width_cells

    def update_cell(self, cell: GridCell, *, hit: bool) -> None:
        if not self.in_bounds(cell):
            return
        update = LOG_ODDS_HIT if hit else LOG_ODDS_MISS
        current_value = float(self._log_odds[cell.row, cell.col])
        new_value = current_value + update
        # Clamp so we don't get stuck - lets cells recover if the world changes.
        self._log_odds[cell.row, cell.col] = max(-LOG_ODDS_CLAMP, min(LOG_ODDS_CLAMP, new_value))

    def is_occupied(self, cell: GridCell) -> bool:
        if not self.in_bounds(cell):
            return False
        return bool(self._log_odds[cell.row, cell.col] > OCCUPIED_THRESHOLD)

    def is_free(self, cell: GridCell) -> bool:
        if not self.in_bounds(cell):
            return False
        return bool(self._log_odds[cell.row, cell.col] < FREE_THRESHOLD)

    def integrate_scan(
        self,
        pose: Pose2D,
        scan: list[LiDARPoint],
        *,
        max_range: float = 50.0,
    ) -> None:
        """Raycast every lidar return into the grid.

        For each return: the cells between the boat and the hit are marked
        "miss" (free), the endpoint is marked "hit" (occupied). Returns past
        max_range are treated as "no hit" so the whole ray is just free.
        """
        origin_cell = self.world_to_grid(pose.x, pose.y)
        for point in scan:
            if not math.isfinite(point.range) or point.range <= 0.0:
                continue
            ray_length = min(point.range, max_range)
            world_angle = pose.heading + point.angle
            endpoint_x = pose.x + ray_length * math.cos(world_angle)
            endpoint_y = pose.y + ray_length * math.sin(world_angle)
            endpoint_cell = self.world_to_grid(endpoint_x, endpoint_y)

            ray_cells = _bresenham(
                origin_cell.row, origin_cell.col,
                endpoint_cell.row, endpoint_cell.col,
            )
            # Everything up to the endpoint is free space.
            for (row, col) in ray_cells[:-1]:
                self.update_cell(GridCell(row, col), hit=False)
            # Endpoint is only a hit if it wasn't clipped by max_range.
            if ray_cells:
                hit_the_thing = point.range < max_range
                last_row, last_col = ray_cells[-1]
                self.update_cell(GridCell(last_row, last_col), hit=hit_the_thing)


def _bresenham(
    start_row: int, start_col: int,
    end_row: int, end_col: int,
) -> list[tuple[int, int]]:
    """Bresenham's line algorithm. Returns every integer cell between the endpoints."""
    cells: list[tuple[int, int]] = []
    row_distance = abs(end_row - start_row)
    col_distance = abs(end_col - start_col)
    row_step = 1 if start_row < end_row else -1
    col_step = 1 if start_col < end_col else -1
    error = row_distance - col_distance

    row, col = start_row, start_col
    while True:
        cells.append((row, col))
        if row == end_row and col == end_col:
            break
        error_doubled = 2 * error
        if error_doubled > -col_distance:
            error -= col_distance
            row += row_step
        if error_doubled < row_distance:
            error += row_distance
            col += col_step
    return cells
