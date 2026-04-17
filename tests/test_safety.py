"""Tests for the safety rules."""

from __future__ import annotations

import math

from klein_b.core.safety import (
    CapsizeRule,
    GeofenceRule,
    LowBatteryRule,
    ObstacleProximityRule,
    SafetyMonitor,
)
from klein_b.core.state import WorldState
from klein_b.core.types import IMUReading, InterruptLevel, LiDARPoint, Pose2D

POSE = Pose2D(x=0.0, y=0.0, heading=0.0, timestamp=0.0)


def _world(**kwargs: object) -> WorldState:
    defaults: dict[str, object] = {"timestamp": 0.0}
    defaults.update(kwargs)
    return WorldState(**defaults)  # type: ignore[arg-type]


def test_low_battery_bucket_thresholds() -> None:
    rule = LowBatteryRule()
    assert rule.check(_world(battery_pct=80.0), POSE) is None

    warn = rule.check(_world(battery_pct=25.0), POSE)
    assert warn is not None and warn.level is InterruptLevel.WARN

    crit = rule.check(_world(battery_pct=15.0), POSE)
    assert crit is not None and crit.level is InterruptLevel.CRITICAL

    em = rule.check(_world(battery_pct=4.0), POSE)
    assert em is not None and em.level is InterruptLevel.EMERGENCY


def test_low_battery_unknown_is_warn() -> None:
    rule = LowBatteryRule()
    result = rule.check(_world(battery_pct=None), POSE)
    assert result is not None and result.level is InterruptLevel.WARN


def test_obstacle_proximity_scales_with_range() -> None:
    rule = ObstacleProximityRule()
    far = _world(lidar_scan=[LiDARPoint(range=10.0, angle=0.0)])
    warn = _world(lidar_scan=[LiDARPoint(range=2.5, angle=0.0)])
    crit = _world(lidar_scan=[LiDARPoint(range=0.5, angle=0.0)])

    assert rule.check(far, POSE) is None
    w = rule.check(warn, POSE)
    c = rule.check(crit, POSE)
    assert w is not None and w.level is InterruptLevel.WARN
    assert c is not None and c.level is InterruptLevel.CRITICAL


def test_obstacle_proximity_empty_scan_passes() -> None:
    assert ObstacleProximityRule().check(_world(), POSE) is None


def test_capsize_fires_past_tilt_limit() -> None:
    rule = CapsizeRule(tilt_limit_rad=math.radians(30.0))
    level = IMUReading(ax=0.0, ay=0.0, az=-9.81, wx=0, wy=0, wz=0, heading=0, timestamp=0)
    tilted = IMUReading(ax=6.93, ay=0.0, az=-6.93, wx=0, wy=0, wz=0, heading=0, timestamp=0)
    flipped = IMUReading(ax=0.0, ay=0.0, az=9.81, wx=0, wy=0, wz=0, heading=0, timestamp=0)

    assert rule.check(_world(imu=level), POSE) is None
    tilt_result = rule.check(_world(imu=tilted), POSE)
    flip_result = rule.check(_world(imu=flipped), POSE)
    assert tilt_result is not None and tilt_result.level is InterruptLevel.EMERGENCY
    assert flip_result is not None and flip_result.level is InterruptLevel.EMERGENCY


def test_geofence_inside_vs_outside() -> None:
    rule = GeofenceRule(x_min=-10.0, y_min=-10.0, x_max=10.0, y_max=10.0, buffer_m=0.0)
    inside = Pose2D(x=0.0, y=0.0, heading=0.0, timestamp=0.0)
    outside = Pose2D(x=15.0, y=0.0, heading=0.0, timestamp=0.0)

    assert rule.check(_world(), inside) is None
    out = rule.check(_world(), outside)
    assert out is not None and out.level is InterruptLevel.CRITICAL


def test_monitor_returns_worst_across_rules() -> None:
    monitor = SafetyMonitor([
        LowBatteryRule(),
        ObstacleProximityRule(),
    ])
    # Both rules fire — return the higher-severity (obstacle CRITICAL).
    world = _world(
        battery_pct=25.0,  # WARN
        lidar_scan=[LiDARPoint(range=0.5, angle=0.0)],  # CRITICAL
    )
    result = monitor.check(world, POSE)
    assert result is not None and result.level is InterruptLevel.CRITICAL
