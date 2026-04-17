"""Priority-queue task scheduler with preemption.

Rules:
    - Submit tasks once; they live in the queue until they finish or fail.
    - Each tick, pick the highest-priority task whose preconditions hold.
    - If a higher-priority task becomes available mid-run, preempt.
    - SafetyMonitor can call preempt(reason) to force a suspension.
    - One tick of "no output" follows any preemption so the safety handler
      can do its thing without the old task fighting back.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from klein_b.scheduler.task import Task, TaskResult, TaskStatus

if TYPE_CHECKING:
    from klein_b.core.state import WorldState
    from klein_b.core.types import Pose2D
    from klein_b.navigation.mapping import OccupancyGrid

log = logging.getLogger(__name__)


class TaskScheduler:
    def __init__(self) -> None:
        self._pending_tasks: list[Task] = []
        self._running_task: Task | None = None
        self._preemption_reason: str | None = None

    def submit(self, task: Task) -> None:
        self._pending_tasks.append(task)
        self._pending_tasks.sort(key=lambda t: t.priority, reverse=True)
        log.info("submitted task %s (priority=%d)", task.name, task.priority)

    def current_task(self) -> Task | None:
        return self._running_task

    def preempt(self, reason: str) -> None:
        if self._running_task is not None:
            log.warning("preempting %s: %s", self._running_task.name, reason)
            self._running_task.on_preempt(reason)
            self._pending_tasks.append(self._running_task)
            self._pending_tasks.sort(key=lambda t: t.priority, reverse=True)
            self._running_task = None
        self._preemption_reason = reason

    def tick(
        self,
        world_state: WorldState,
        pose: Pose2D,
        grid: OccupancyGrid,
    ) -> TaskResult | None:
        # Right after a preemption, skip a tick so the safety response runs clean.
        if self._preemption_reason is not None:
            self._preemption_reason = None
            return None

        # Idle? pick something to do.
        if self._running_task is None:
            self._running_task = self._pick_eligible_task(world_state, pose)
            if self._running_task is None:
                return None
            self._pending_tasks.remove(self._running_task)
            self._running_task.on_start()
            log.info("starting task %s", self._running_task.name)

        # Anything higher-priority newly eligible? if so preempt.
        higher_task = self._pick_eligible_task(
            world_state, pose, minimum_priority=self._running_task.priority,
        )
        if higher_task is not None:
            self.preempt(f"higher-priority task available: {higher_task.name}")
            return None

        result = self._running_task.on_tick(world_state, pose, grid)

        if self._running_task.is_complete():
            log.info("task %s completed", self._running_task.name)
            self._running_task = None
        elif result.status is TaskStatus.FAILED:
            log.warning("task %s failed", self._running_task.name)
            self._running_task = None

        return result

    def _pick_eligible_task(
        self,
        world_state: WorldState,
        pose: Pose2D,
        *,
        minimum_priority: int | None = None,
    ) -> Task | None:
        for task in self._pending_tasks:
            if minimum_priority is not None and task.priority <= minimum_priority:
                continue
            if task.preconditions(world_state, pose):
                return task
        return None
