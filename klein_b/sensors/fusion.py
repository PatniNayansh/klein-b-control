"""Combine sensor readings into a single WorldState snapshot.

Not the Kalman kind of fusion - that's in localization.py. Here we just
poll each sensor, catch failures, and build a WorldState. If a sensor
raises, its field becomes None and a warning gets logged.
"""

from __future__ import annotations

import logging

from klein_b.core.state import WorldState
from klein_b.sensors.interfaces import (
    BatterySensor,
    GPSSensor,
    IMUSensor,
    LiDARSensor,
    WaterQualitySensor,
)

log = logging.getLogger(__name__)


class SensorFusion:
    def __init__(
        self,
        lidar: LiDARSensor,
        gps: GPSSensor,
        imu: IMUSensor,
        battery: BatterySensor,
        water: WaterQualitySensor,
    ) -> None:
        self._lidar = lidar
        self._gps = gps
        self._imu = imu
        self._battery = battery
        self._water = water

    def sample(self, now: float) -> WorldState:
        """Poll every sensor and return a WorldState stamped at `now`."""
        try:
            scan = self._lidar.read()
        except Exception as error:
            log.warning("lidar read failed: %s", error)
            scan = []

        try:
            gps = self._gps.read()
        except Exception as error:
            log.warning("gps read failed: %s", error)
            gps = None

        try:
            imu = self._imu.read()
        except Exception as error:
            log.warning("imu read failed: %s", error)
            imu = None

        try:
            battery = self._battery.read()
        except Exception as error:
            log.warning("battery read failed: %s", error)
            battery = None

        try:
            water = self._water.read()
        except Exception as error:
            log.warning("water read failed: %s", error)
            water = None

        turbidity, ph = (water if water is not None else (None, None))

        return WorldState(
            timestamp=now,
            lidar_scan=scan,
            gps=gps,
            imu=imu,
            battery_pct=battery,
            water_turbidity=turbidity,
            water_ph=ph,
        )
