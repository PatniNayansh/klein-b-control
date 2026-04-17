"""Pure-pursuit waypoint controller. Turns (pose, target) into L/R thrust.

Basic idea:
    linear  = k_lin * distance_to_target   (capped)
    angular = k_ang * heading_error
    L = linear - angular
    R = linear + angular

When the heading error is big, we damp linear so the boat pivots instead
of swinging out. Gains below are ballpark - final values come from field
tuning (which obviously we haven't done yet).
"""

from __future__ import annotations

import math

from klein_b.core.types import ControlOutput, Pose2D, Waypoint

# TODO: tune these on the real hull once we have one
LINEAR_GAIN = 0.6
ANGULAR_GAIN = 1.1
MAX_LINEAR_THRUST = 0.7
HEADING_DAMP_THRESHOLD_RAD = math.radians(30.0)


class WaypointController:
    def compute(
        self,
        pose: Pose2D,
        target: Waypoint,
        *,
        conveyor_on: bool = False,
        pump_on: bool = False,
    ) -> ControlOutput:
        delta_x = target.x - pose.x
        delta_y = target.y - pose.y
        distance_to_target = math.hypot(delta_x, delta_y)
        desired_heading = math.atan2(delta_y, delta_x)
        heading_error = _wrap_to_pi(desired_heading - pose.heading)

        linear_command = min(LINEAR_GAIN * distance_to_target, MAX_LINEAR_THRUST)
        if abs(heading_error) > HEADING_DAMP_THRESHOLD_RAD:
            # Big heading error - cut linear so angular dominates (pivot in place).
            linear_command *= max(0.0, 1.0 - abs(heading_error) / math.pi)

        angular_command = ANGULAR_GAIN * heading_error
        left_thrust = _clip_to_unit(linear_command - angular_command)
        right_thrust = _clip_to_unit(linear_command + angular_command)
        return ControlOutput(
            left_thrust=left_thrust,
            right_thrust=right_thrust,
            conveyor_on=conveyor_on,
            pump_on=pump_on,
        )


def _wrap_to_pi(angle: float) -> float:
    """Wrap an angle in radians to (-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _clip_to_unit(value: float) -> float:
    return max(-1.0, min(1.0, value))
