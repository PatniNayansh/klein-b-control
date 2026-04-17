"""Task = one unit of mission work (e.g. "go dock", "sweep this region").

Tasks here are generic on purpose - the actual phase-specific tasks
(Azolla planting, Daphnia release, surface skim) go in a separate
mission-runner project. This file is just the abstract contract.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Any

from klein_b.core.types import Waypoint

if TYPE_CHECKING:
    from klein_b.core.state import WorldState
    from klein_b.core.types import Pose2D
    from klein_b.navigation.mapping import OccupancyGrid


class TaskStatus(Enum):
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    PREEMPTED = "preempted"


@dataclass(frozen=True, slots=True)
class TaskResult:
    """What a task emits on each tick.

    actuator_cmds is free-form: put whatever keys your actuators expect
    (e.g. {"conveyor_on": True, "pump_rate": 0.4}).
    """

    status: TaskStatus
    nav_goal: Waypoint | None = None
    actuator_cmds: dict[str, Any] = field(default_factory=dict)


class Task(ABC):
    """Subclass this to make a concrete mission task."""

    @property
    @abstractmethod
    def name(self) -> str:
        ...

    @property
    @abstractmethod
    def priority(self) -> int:
        """Higher = runs first. Safety-critical stuff uses >= 900."""

    @abstractmethod
    def preconditions(self, world_state: WorldState, pose: Pose2D) -> bool:
        """Can this task run right now?"""

    @abstractmethod
    def on_tick(
        self, world_state: WorldState, pose: Pose2D, grid: OccupancyGrid,
    ) -> TaskResult:
        ...

    @abstractmethod
    def is_complete(self) -> bool:
        ...

    # Default hooks - override if you need them.
    def on_start(self) -> None:  # noqa: B027
        pass

    def on_preempt(self, reason: str) -> None:  # noqa: B027
        pass
