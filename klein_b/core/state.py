"""WorldState = everything the boat knows at one instant. BoatState = running totals."""

from __future__ import annotations

from dataclasses import dataclass, field

from klein_b.core.types import IMUReading, LiDARPoint, Pose2D


@dataclass(frozen=True, slots=True)
class WorldState:
    """Fused snapshot of all sensors. Any field can be None if that sensor failed."""

    timestamp: float
    lidar_scan: list[LiDARPoint] = field(default_factory=list)
    gps: tuple[float, float] | None = None  # (lat, lon) in WGS84 degrees
    imu: IMUReading | None = None
    battery_pct: float | None = None
    water_turbidity: float | None = None  # NTU
    water_ph: float | None = None


@dataclass(slots=True)
class BoatState:
    """Stuff we track across ticks (mutable, unlike WorldState)."""

    pose: Pose2D
    mission_elapsed_s: float = 0.0
    total_area_covered_m2: float = 0.0
