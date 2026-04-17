"""Safety rules. Each rule is a simple function of (world, pose).

Kept these as independent rules rather than a single state machine - if
one rule gets out of sync with reality that's way harder to debug than
a missed detection.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

from klein_b.core.state import WorldState
from klein_b.core.types import InterruptLevel, Pose2D


@dataclass(frozen=True, slots=True)
class Interrupt:
    level: InterruptLevel
    reason: str
    recommended_action: str


class SafetyRule(ABC):
    @abstractmethod
    def check(self, world_state: WorldState, pose: Pose2D) -> Interrupt | None:
        """Return an Interrupt if this rule trips, else None."""


class SafetyMonitor:
    """Runs every rule each tick and returns the most severe interrupt (if any)."""

    def __init__(self, rules: list[SafetyRule] | None = None) -> None:
        self._rules: list[SafetyRule] = list(rules) if rules else []

    def register(self, rule: SafetyRule) -> None:
        self._rules.append(rule)

    def check(self, world_state: WorldState, pose: Pose2D) -> Interrupt | None:
        worst_interrupt: Interrupt | None = None
        for rule in self._rules:
            result = rule.check(world_state, pose)
            if result is None:
                continue
            if worst_interrupt is None or result.level > worst_interrupt.level:
                worst_interrupt = result
        return worst_interrupt


class LowBatteryRule(SafetyRule):
    """Trips at 30% (warn), 20% (critical), 5% (emergency)."""

    def __init__(
        self,
        warn_pct: float = 30.0,
        critical_pct: float = 20.0,
        emergency_pct: float = 5.0,
    ) -> None:
        self.warn_pct = warn_pct
        self.critical_pct = critical_pct
        self.emergency_pct = emergency_pct

    def check(self, world_state: WorldState, pose: Pose2D) -> Interrupt | None:
        battery_pct = world_state.battery_pct
        if battery_pct is None:
            # If we can't read the battery, flag it but don't panic.
            return Interrupt(InterruptLevel.WARN, "battery SOC unavailable",
                             "continue but flag for operator")
        if battery_pct <= self.emergency_pct:
            return Interrupt(InterruptLevel.EMERGENCY, f"battery {battery_pct:.1f}%",
                             "halt propulsion; send distress beacon")
        if battery_pct <= self.critical_pct:
            return Interrupt(InterruptLevel.CRITICAL, f"battery {battery_pct:.1f}%",
                             "abort mission; return to dock")
        if battery_pct <= self.warn_pct:
            return Interrupt(InterruptLevel.WARN, f"battery {battery_pct:.1f}%",
                             "finish current task then return")
        return None


class ObstacleProximityRule(SafetyRule):
    """Looks at the closest lidar return and trips if we're too close."""

    def __init__(self, warn_range_m: float = 3.0, critical_range_m: float = 1.2) -> None:
        self.warn_range_m = warn_range_m
        self.critical_range_m = critical_range_m

    def check(self, world_state: WorldState, pose: Pose2D) -> Interrupt | None:
        nearest_range = math.inf
        for point in world_state.lidar_scan:
            if math.isfinite(point.range) and 0.0 < point.range < nearest_range:
                nearest_range = point.range
        if nearest_range == math.inf:
            return None
        if nearest_range <= self.critical_range_m:
            return Interrupt(InterruptLevel.CRITICAL,
                             f"obstacle at {nearest_range:.2f}m",
                             "full stop; plan avoidance")
        if nearest_range <= self.warn_range_m:
            return Interrupt(InterruptLevel.WARN,
                             f"obstacle at {nearest_range:.2f}m",
                             "reduce speed")
        return None


class CapsizeRule(SafetyRule):
    """IMU-based capsize detection.

    Uses the gravity vector in the body frame: if the boat is level, gravity
    is pointing along body -z (so az is negative with magnitude ~9.8). Tilt
    is the angle between that vector and body +z. cos(tilt) = -az / |g|.
    """

    def __init__(self, tilt_limit_rad: float = math.radians(30.0)) -> None:
        self.tilt_limit_rad = tilt_limit_rad

    def check(self, world_state: WorldState, pose: Pose2D) -> Interrupt | None:
        imu = world_state.imu
        if imu is None:
            return None
        gravity_magnitude = math.sqrt(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az)
        if gravity_magnitude < 1e-3:
            return None  # weird reading, skip this tick
        cos_tilt = max(-1.0, min(1.0, -imu.az / gravity_magnitude))
        tilt_rad = math.acos(cos_tilt)
        if tilt_rad > self.tilt_limit_rad:
            return Interrupt(InterruptLevel.EMERGENCY,
                             f"tilt {math.degrees(tilt_rad):.1f}deg",
                             "kill thrusters; broadcast MAYDAY")
        return None


class GeofenceRule(SafetyRule):
    """Keep the boat inside a rectangular operations area."""

    def __init__(
        self,
        x_min: float, y_min: float, x_max: float, y_max: float,
        buffer_m: float = 2.0,
    ) -> None:
        self.x_min = x_min
        self.y_min = y_min
        self.x_max = x_max
        self.y_max = y_max
        self.buffer_m = buffer_m

    def check(self, world_state: WorldState, pose: Pose2D) -> Interrupt | None:
        if (
            pose.x < self.x_min - self.buffer_m
            or pose.x > self.x_max + self.buffer_m
            or pose.y < self.y_min - self.buffer_m
            or pose.y > self.y_max + self.buffer_m
        ):
            return Interrupt(InterruptLevel.CRITICAL,
                             f"outside geofence at ({pose.x:.1f}, {pose.y:.1f})",
                             "turn around; re-enter surveyed bounds")
        return None
