"""Mission cycle timer for tracking elapsed time and worst-case cycle duration."""

from __future__ import annotations

import time


class MissionTimer:
    """
    Track mission elapsed time and per-cycle durations.

    All timing uses ``time.monotonic()`` to avoid wall-clock jumps.
    The 1200-second budget guard in ``canStartCycle`` prevents starting a new
    cycle when the worst-case cycle time would push the mission past the limit.
    """

    def __init__(self) -> None:
        """Initialise all timing state at construction time."""
        self.missionStartTime: float = time.monotonic()
        self.cycleStartTime: float = time.monotonic()
        self.worstCycleTime: float = 0.0
        self.completedCycles: int = 0

    def beginCycle(self) -> None:
        """Reset the per-cycle start time to now."""
        self.cycleStartTime = time.monotonic()

    def recordCycleTime(self) -> float:
        """
        Record the current cycle duration and return it.

        Computes elapsed time since the last ``beginCycle`` call, updates
        ``worstCycleTime`` if this cycle is the slowest seen so far, increments
        ``completedCycles``, and returns the recorded duration.
        """
        duration: float = time.monotonic() - self.cycleStartTime
        if duration > self.worstCycleTime:
            self.worstCycleTime = duration
        self.completedCycles += 1
        return duration

    def canStartCycle(self) -> bool:
        """
        Return whether there is enough time budget to start another cycle.

        The first cycle (``completedCycles == 0``) is always allowed.
        Subsequent cycles are allowed only when the elapsed mission time plus
        the worst recorded cycle time stays below 1200 seconds.
        """
        if self.completedCycles == 0:
            return True
        elapsed_mission_time: float = time.monotonic() - self.missionStartTime
        return elapsed_mission_time + self.worstCycleTime < 1200
