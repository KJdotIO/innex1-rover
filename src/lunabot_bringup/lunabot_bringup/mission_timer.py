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

    def canStartCycle(self, pre_hoc_time_s: float | None = None) -> bool:
        """
        Return whether there is enough time budget to start another cycle.

        For the first cycle (``completedCycles == 0``), a ``pre_hoc_time_s``
        estimate **must** be supplied.  The cycle is allowed when
        ``elapsed_mission_time + pre_hoc_time_s < 1200``.  Omitting the
        estimate on the first cycle raises ``ValueError``; the method never
        silently permits an unconstrained first cycle.

        For subsequent cycles (``completedCycles > 0``), the worst recorded
        cycle time is used as the budget estimate and ``pre_hoc_time_s`` is
        ignored.
        """
        elapsed_mission_time: float = time.monotonic() - self.missionStartTime

        if self.completedCycles == 0:
            if pre_hoc_time_s is None:
                raise ValueError(
                    "pre_hoc_time_s must be supplied for the first cycle "
                    "(completedCycles == 0); refusing to allow cycle without "
                    "a time estimate"
                )
            return elapsed_mission_time + pre_hoc_time_s < 1200

        return elapsed_mission_time + self.worstCycleTime < 1200
