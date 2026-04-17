"""Tests for the task scheduler."""

from __future__ import annotations

from klein_b.core.state import WorldState
from klein_b.core.types import Pose2D, Waypoint
from klein_b.navigation.mapping import GridSpec, OccupancyGrid
from klein_b.scheduler.scheduler import TaskScheduler
from klein_b.scheduler.task import Task, TaskResult, TaskStatus

POSE = Pose2D(x=0.0, y=0.0, heading=0.0, timestamp=0.0)
WORLD = WorldState(timestamp=0.0)
GRID = OccupancyGrid(GridSpec(width_cells=10, height_cells=10, resolution_m=1.0,
                              origin_x=0.0, origin_y=0.0))


class FakeTask(Task):
    def __init__(
        self, name: str, priority: int, ticks: int = 3, *, eligible: bool = True,
    ) -> None:
        self._name = name
        self._priority = priority
        self._remaining = ticks
        self._eligible = eligible
        self.tick_count = 0
        self.started = False
        self.preempted_with: str | None = None

    @property
    def name(self) -> str:
        return self._name

    @property
    def priority(self) -> int:
        return self._priority

    def preconditions(self, world_state: WorldState, pose: Pose2D) -> bool:
        return self._eligible

    def on_start(self) -> None:
        self.started = True

    def on_preempt(self, reason: str) -> None:
        self.preempted_with = reason

    def on_tick(
        self, world_state: WorldState, pose: Pose2D, grid: OccupancyGrid,
    ) -> TaskResult:
        self.tick_count += 1
        self._remaining -= 1
        return TaskResult(
            status=TaskStatus.RUNNING,
            nav_goal=Waypoint(x=1.0, y=1.0),
        )

    def is_complete(self) -> bool:
        return self._remaining <= 0


def test_empty_scheduler_returns_none() -> None:
    sched = TaskScheduler()
    assert sched.tick(WORLD, POSE, GRID) is None


def test_highest_priority_runs_first() -> None:
    sched = TaskScheduler()
    low = FakeTask("low", priority=10)
    high = FakeTask("high", priority=50)
    sched.submit(low)
    sched.submit(high)

    sched.tick(WORLD, POSE, GRID)
    assert sched.current_task() is high
    assert high.started
    assert not low.started


def test_completion_advances_to_next_task() -> None:
    sched = TaskScheduler()
    first = FakeTask("first", priority=10, ticks=1)
    second = FakeTask("second", priority=5, ticks=5)
    sched.submit(first)
    sched.submit(second)

    sched.tick(WORLD, POSE, GRID)  # runs first, completes in this same tick
    assert sched.current_task() is None
    sched.tick(WORLD, POSE, GRID)  # picks up second; still running (ticks=5)
    assert sched.current_task() is second
    assert second.started
    assert second.tick_count == 1


def test_higher_priority_preempts_running_task() -> None:
    sched = TaskScheduler()
    running = FakeTask("running", priority=10, ticks=100)
    sched.submit(running)
    sched.tick(WORLD, POSE, GRID)
    assert sched.current_task() is running

    urgent = FakeTask("urgent", priority=500, ticks=100)
    sched.submit(urgent)

    sched.tick(WORLD, POSE, GRID)  # detects higher-priority, preempts
    assert sched.current_task() is None
    assert running.preempted_with is not None

    sched.tick(WORLD, POSE, GRID)  # skip tick after preemption
    assert sched.current_task() is None

    sched.tick(WORLD, POSE, GRID)  # urgent takes over
    assert sched.current_task() is urgent


def test_external_preempt_skips_next_tick() -> None:
    sched = TaskScheduler()
    t = FakeTask("t", priority=10, ticks=100)
    sched.submit(t)
    sched.tick(WORLD, POSE, GRID)
    assert sched.current_task() is t

    sched.preempt("low battery")
    assert sched.current_task() is None
    assert t.preempted_with == "low battery"

    # The tick immediately after a preemption must yield None.
    assert sched.tick(WORLD, POSE, GRID) is None


def test_ineligible_task_is_skipped() -> None:
    sched = TaskScheduler()
    gated = FakeTask("gated", priority=100, eligible=False)
    ready = FakeTask("ready", priority=10)
    sched.submit(gated)
    sched.submit(ready)

    sched.tick(WORLD, POSE, GRID)
    assert sched.current_task() is ready
