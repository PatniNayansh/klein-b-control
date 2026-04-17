"""Shows how the Brain gets wired up.

This DOESN'T actually run the control loop - the sensor classes below
raise NotImplementedError on read(). It's just here so you can see how
all the pieces fit together. Drop in real drivers for the "Unconnected*"
classes and brain.tick(dt, now) will actually produce thrust commands.

Run:
    python examples/brain_wiring.py
"""

from __future__ import annotations

import logging

from klein_b.brain import Brain
from klein_b.core.safety import (
    CapsizeRule,
    GeofenceRule,
    LowBatteryRule,
    ObstacleProximityRule,
    SafetyMonitor,
)
from klein_b.core.types import IMUReading, LiDARPoint, Pose2D
from klein_b.navigation.controller import WaypointController
from klein_b.navigation.localization import Localizer
from klein_b.navigation.mapping import GridSpec, OccupancyGrid
from klein_b.navigation.path_planner import PathPlanner
from klein_b.scheduler.scheduler import TaskScheduler
from klein_b.sensors.fusion import SensorFusion
from klein_b.sensors.interfaces import (
    BatterySensor,
    GPSSensor,
    IMUSensor,
    LiDARSensor,
    WaterQualitySensor,
)


# Placeholder sensors - real drivers go here later.
class UnconnectedLiDAR(LiDARSensor):
    def read(self) -> list[LiDARPoint]:
        raise NotImplementedError


class UnconnectedGPS(GPSSensor):
    def read(self) -> tuple[float, float] | None:
        raise NotImplementedError


class UnconnectedIMU(IMUSensor):
    def read(self) -> IMUReading | None:
        raise NotImplementedError


class UnconnectedBattery(BatterySensor):
    def read(self) -> float | None:
        raise NotImplementedError


class UnconnectedWaterProbe(WaterQualitySensor):
    def read(self) -> tuple[float, float] | None:
        raise NotImplementedError


def build_brain() -> Brain:
    fusion = SensorFusion(
        lidar=UnconnectedLiDAR(),
        gps=UnconnectedGPS(),
        imu=UnconnectedIMU(),
        battery=UnconnectedBattery(),
        water=UnconnectedWaterProbe(),
    )

    localizer = Localizer(Pose2D(x=0.0, y=0.0, heading=0.0, timestamp=0.0))

    # 100m x 100m at 0.5m resolution, centered on origin.
    grid = OccupancyGrid(GridSpec(
        width_cells=200, height_cells=200,
        resolution_m=0.5,
        origin_x=-50.0, origin_y=-50.0,
    ))

    safety = SafetyMonitor([
        LowBatteryRule(),
        ObstacleProximityRule(),
        CapsizeRule(),
        GeofenceRule(x_min=-40.0, y_min=-40.0, x_max=40.0, y_max=40.0),
    ])

    return Brain(
        fusion=fusion,
        localizer=localizer,
        grid=grid,
        planner=PathPlanner(),
        controller=WaypointController(),
        scheduler=TaskScheduler(),
        safety=safety,
    )


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    brain = build_brain()
    logging.getLogger(__name__).info("Brain wired up OK.")
    # DON'T call brain.tick() - sensor stubs will raise NotImplementedError.
    _ = brain


if __name__ == "__main__":
    main()
