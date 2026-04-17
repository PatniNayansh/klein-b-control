"""Shared types (pose, waypoint, grid cell, lidar point, etc.)."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


@dataclass(frozen=True, slots=True)
class Pose2D:
    """Boat pose in the lake frame. x/y in meters, heading in radians (CCW from +x)."""

    x: float
    y: float
    heading: float
    timestamp: float


@dataclass(frozen=True, slots=True)
class Waypoint:
    x: float
    y: float
    tolerance: float = 1.0  # meters; within this distance counts as "reached"


@dataclass(frozen=True, slots=True)
class GridCell:
    row: int
    col: int


@dataclass(frozen=True, slots=True)
class LiDARPoint:
    """One return from the lidar in the sensor frame. inf range = no hit."""

    range: float
    angle: float
    intensity: float = 0.0


@dataclass(frozen=True, slots=True)
class IMUReading:
    ax: float
    ay: float
    az: float
    wx: float
    wy: float
    wz: float
    heading: float
    timestamp: float


@dataclass(frozen=True, slots=True)
class ControlOutput:
    """Thrust + actuator command produced by Brain.tick().

    Thrust is [-1, 1]; the hardware layer maps that to PWM.
    """

    left_thrust: float
    right_thrust: float
    conveyor_on: bool
    pump_on: bool

    def __post_init__(self) -> None:
        # Sanity check - helps catch controller bugs before they reach motors.
        for name, val in (("left_thrust", self.left_thrust), ("right_thrust", self.right_thrust)):
            if not -1.0 <= val <= 1.0:
                raise ValueError(f"{name}={val} outside [-1, 1]")


class InterruptLevel(IntEnum):
    INFO = 0
    WARN = 1
    CRITICAL = 2
    EMERGENCY = 3


NEUTRAL_OUTPUT = ControlOutput(left_thrust=0.0, right_thrust=0.0, conveyor_on=False, pump_on=False)
