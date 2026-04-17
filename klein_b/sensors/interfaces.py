"""Abstract sensor interfaces — one per physical sensor on the boat.

Each `read()` is a stub that raises NotImplementedError. The real hardware
driver lives elsewhere (Velodyne SDK, uBlox serial reader, etc.) and
subclasses the ABC here.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from klein_b.core.types import IMUReading, LiDARPoint


class LiDARSensor(ABC):
    """2D/3D lidar. We use the Velodyne HDL-64E spec as reference (5-20 Hz)."""

    @abstractmethod
    def read(self) -> list[LiDARPoint]:
        """Return the latest scan as a list of LiDARPoint in the sensor frame."""
        raise NotImplementedError  # TODO: plug in Velodyne driver


class GPSSensor(ABC):
    """GNSS receiver (uBlox ZED-F9P). Returns WGS84 lat/lon in degrees."""

    @abstractmethod
    def read(self) -> tuple[float, float] | None:
        """Return (lat, lon) or None if there's no fix."""
        raise NotImplementedError


class IMUSensor(ABC):
    """IMU with magnetometer-aided heading (planning on BNO085)."""

    @abstractmethod
    def read(self) -> IMUReading | None:
        raise NotImplementedError


class BatterySensor(ABC):
    """Talks to the BMS over CAN and reports state-of-charge (0-100%)."""

    @abstractmethod
    def read(self) -> float | None:
        raise NotImplementedError


class WaterQualitySensor(ABC):
    """Turbidity + pH probe (Atlas Scientific EZO). ~1 Hz is plenty."""

    @abstractmethod
    def read(self) -> tuple[float, float] | None:
        """Return (turbidity_NTU, pH). None if the probe isn't calibrated."""
        raise NotImplementedError
