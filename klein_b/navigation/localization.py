"""State estimator. Dead-reckoning today, full EKF later.

State vector is [x, y, heading, velocity, omega] (meters, meters, rad, m/s, rad/s).
Motion model is a standard unicycle. Predict integrates the state; update
currently absorbs IMU heading + yaw rate directly (which works well enough
when the magnetometer is calibrated).

The GPS measurement update - the part that actually corrects position
drift - is the TODO block below. Writing the Kalman gain without real
covariance numbers from the hardware would just be fake - I'll do it
once I have GPS data to characterize against.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np

from klein_b.core.types import Pose2D

if TYPE_CHECKING:
    from klein_b.core.state import WorldState

# State vector layout - makes the code readable without magic numbers.
IDX_X = 0
IDX_Y = 1
IDX_HEADING = 2
IDX_VELOCITY = 3
IDX_OMEGA = 4

# Process noise (diagonal of Q). Ballpark values.
PROCESS_NOISE_DIAG = np.array([0.05, 0.05, 0.02, 0.10, 0.05], dtype=np.float64)


class Localizer:
    def __init__(self, initial_pose: Pose2D) -> None:
        self._state = np.array(
            [initial_pose.x, initial_pose.y, initial_pose.heading, 0.0, 0.0],
            dtype=np.float64,
        )
        self._covariance = np.diag([1.0, 1.0, 0.2, 0.5, 0.2]).astype(np.float64)
        self._last_timestamp = initial_pose.timestamp

    def predict(self, dt: float) -> None:
        """Integrate motion forward by dt seconds (unicycle model)."""
        if dt <= 0.0:
            return
        velocity = self._state[IDX_VELOCITY]
        heading = self._state[IDX_HEADING]
        omega = self._state[IDX_OMEGA]

        self._state[IDX_X] += velocity * math.cos(heading) * dt
        self._state[IDX_Y] += velocity * math.sin(heading) * dt
        self._state[IDX_HEADING] = _wrap_to_pi(heading + omega * dt)
        # velocity and omega are treated as a random walk between updates.

        # TODO: proper covariance propagation through the motion Jacobian.
        # For now just inflate with process noise - good enough at 20+ Hz.
        self._covariance += np.diag(PROCESS_NOISE_DIAG * dt)

    def update(self, world_state: WorldState) -> None:
        """Pull in the latest sensor data."""
        imu = world_state.imu
        if imu is not None:
            # Magnetometer-aided IMU heading is accurate enough to use directly.
            self._state[IDX_HEADING] = _wrap_to_pi(imu.heading)
            self._state[IDX_OMEGA] = imu.wz
            # Tighten heading/omega covariance after a fresh IMU reading.
            self._covariance[IDX_HEADING, IDX_HEADING] = min(
                self._covariance[IDX_HEADING, IDX_HEADING], 0.02,
            )
            self._covariance[IDX_OMEGA, IDX_OMEGA] = min(
                self._covariance[IDX_OMEGA, IDX_OMEGA], 0.05,
            )

        if world_state.gps is not None:
            # TODO: EKF measurement update on (x, y).
            #   1. lat/lon -> local ENU via a configured origin
            #   2. innovation = measurement - H @ state   (H picks x,y out of state)
            #   3. S = H @ P @ H^T + R_gps
            #   4. K = P @ H^T @ inv(S)
            #   5. state += K @ innovation;  P = (I - K @ H) @ P
            # Need real R_gps from the receiver's HDOP. Not worth faking.
            pass

        self._last_timestamp = world_state.timestamp

    def get_pose(self) -> Pose2D:
        return Pose2D(
            x=float(self._state[IDX_X]),
            y=float(self._state[IDX_Y]),
            heading=float(self._state[IDX_HEADING]),
            timestamp=float(self._last_timestamp),
        )

    @property
    def covariance(self) -> np.ndarray:
        return self._covariance


def _wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
