"""The main control loop.

Each tick does:
    1. sense     - poll all sensors (fusion.sample)
    2. localize  - predict + update from IMU
    3. map       - integrate lidar scan into the occupancy grid
    4. safety    - run every safety rule; if CRITICAL+ we preempt and stop
    5. schedule  - run the current task's tick, get a TaskResult
    6. plan      - A* from current pose to the task's nav_goal
    7. control   - pure pursuit to the next waypoint, emit thrust

Everything it needs is injected in the constructor (DI keeps testing
sane - swap in fake sensors and you can drive the Brain without hardware).
"""

from __future__ import annotations

import logging
import math

from klein_b.core.safety import SafetyMonitor
from klein_b.core.types import NEUTRAL_OUTPUT, ControlOutput, InterruptLevel, Pose2D, Waypoint
from klein_b.navigation.controller import WaypointController
from klein_b.navigation.localization import Localizer
from klein_b.navigation.mapping import OccupancyGrid
from klein_b.navigation.path_planner import PathPlanner
from klein_b.scheduler.scheduler import TaskScheduler
from klein_b.sensors.fusion import SensorFusion

log = logging.getLogger(__name__)


class Brain:
    def __init__(
        self,
        *,
        fusion: SensorFusion,
        localizer: Localizer,
        grid: OccupancyGrid,
        planner: PathPlanner,
        controller: WaypointController,
        scheduler: TaskScheduler,
        safety: SafetyMonitor,
        lidar_max_range_m: float = 50.0,
    ) -> None:
        self._fusion = fusion
        self._localizer = localizer
        self._grid = grid
        self._planner = planner
        self._controller = controller
        self._scheduler = scheduler
        self._safety = safety
        self._lidar_max_range_m = lidar_max_range_m
        # Cache the path so we don't re-run A* every single tick.
        self._cached_goal: Waypoint | None = None
        self._cached_path: list[Waypoint] = []

    def tick(self, dt: float, now: float) -> ControlOutput:
        # 1. sense
        world_state = self._fusion.sample(now)

        # 2. localize
        self._localizer.predict(dt)
        self._localizer.update(world_state)
        current_pose = self._localizer.get_pose()

        # 3. map
        self._grid.integrate_scan(
            current_pose,
            world_state.lidar_scan,
            max_range=self._lidar_max_range_m,
        )

        # 4. safety - anything CRITICAL or worse stops thrust
        interrupt = self._safety.check(world_state, current_pose)
        if interrupt is not None and interrupt.level >= InterruptLevel.CRITICAL:
            log.error(
                "SAFETY %s: %s -> %s",
                interrupt.level.name, interrupt.reason, interrupt.recommended_action,
            )
            self._scheduler.preempt(interrupt.reason)
            self._cached_path = []
            return NEUTRAL_OUTPUT

        # 5. schedule
        task_result = self._scheduler.tick(world_state, current_pose, self._grid)
        if task_result is None or task_result.nav_goal is None:
            return NEUTRAL_OUTPUT

        # 6. plan (cached)
        next_waypoint = self._get_next_waypoint(current_pose, task_result.nav_goal)
        if next_waypoint is None:
            log.warning("no path to %s; idling", task_result.nav_goal)
            return NEUTRAL_OUTPUT

        # 7. control
        return self._controller.compute(
            current_pose,
            next_waypoint,
            conveyor_on=bool(task_result.actuator_cmds.get("conveyor_on", False)),
            pump_on=bool(task_result.actuator_cmds.get("pump_on", False)),
        )

    def _get_next_waypoint(self, pose: Pose2D, goal: Waypoint) -> Waypoint | None:
        """Next waypoint from the cached path. Replans if the goal changed."""
        if self._cached_goal != goal or not self._cached_path:
            self._cached_goal = goal
            new_path = self._planner.plan_point_to_point(
                Waypoint(pose.x, pose.y, tolerance=goal.tolerance),
                goal,
                self._grid,
            )
            self._cached_path = new_path or []

        # Pop any waypoints we've already reached.
        while self._cached_path and _already_reached(pose, self._cached_path[0]):
            self._cached_path.pop(0)
        return self._cached_path[0] if self._cached_path else None


def _already_reached(pose: Pose2D, waypoint: Waypoint) -> bool:
    distance_to_wp = math.hypot(waypoint.x - pose.x, waypoint.y - pose.y)
    return distance_to_wp <= waypoint.tolerance
