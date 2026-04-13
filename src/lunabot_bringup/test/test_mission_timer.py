"""Unit tests for the MissionTimer class."""

from __future__ import annotations

import time
from unittest.mock import patch

import pytest

from lunabot_bringup.mission_timer import MissionTimer


def test_fresh_timer_has_expected_initial_state():
    """A newly constructed MissionTimer has zeroed counters and monotonic timestamps."""
    before = time.monotonic()
    timer = MissionTimer()
    after = time.monotonic()

    assert before <= timer.missionStartTime <= after
    assert before <= timer.cycleStartTime <= after
    assert timer.worstCycleTime == 0.0
    assert timer.completedCycles == 0


def test_begin_cycle_resets_cycle_start_time():
    """Verify beginCycle() resets cycleStartTime to a value at or after the previous one."""
    timer = MissionTimer()
    old_start = timer.cycleStartTime
    timer.beginCycle()
    assert timer.cycleStartTime >= old_start


def test_record_cycle_time_returns_positive_float_and_increments_counter():
    """Verify recordCycleTime() returns a non-negative duration and increments completedCycles."""
    timer = MissionTimer()
    timer.beginCycle()
    duration = timer.recordCycleTime()

    assert isinstance(duration, float)
    assert duration >= 0.0
    assert timer.completedCycles == 1


def test_record_cycle_time_updates_worst_cycle_time_for_slower_cycle(monkeypatch):
    """Verify worstCycleTime is updated when a slower cycle is recorded."""
    timer = MissionTimer()

    # First cycle: fake a duration of 10 seconds
    fake_now = [1000.0]

    def fake_monotonic():
        return fake_now[0]

    monkeypatch.setattr(time, "monotonic", fake_monotonic)
    timer.cycleStartTime = 990.0  # 10 s ago relative to fake_now[0]
    timer.recordCycleTime()
    assert timer.worstCycleTime == pytest.approx(10.0)

    # Second cycle: fake a duration of 20 seconds (slower)
    timer.cycleStartTime = 980.0  # 20 s ago
    timer.recordCycleTime()
    assert timer.worstCycleTime == pytest.approx(20.0)


def test_record_cycle_time_does_not_update_worst_cycle_time_for_faster_cycle(
    monkeypatch,
):
    """Verify worstCycleTime is not updated when a faster cycle is recorded."""
    timer = MissionTimer()

    fake_now = [1000.0]

    def fake_monotonic():
        return fake_now[0]

    monkeypatch.setattr(time, "monotonic", fake_monotonic)

    # First cycle: 20 s
    timer.cycleStartTime = 980.0
    timer.recordCycleTime()
    assert timer.worstCycleTime == pytest.approx(20.0)

    # Second cycle: 5 s (faster) — worstCycleTime must stay at 20
    timer.cycleStartTime = 995.0
    timer.recordCycleTime()
    assert timer.worstCycleTime == pytest.approx(20.0)


def test_first_cycle_allowed_when_prehoc_fits_budget():
    """First cycle is allowed when elapsed + pre_hoc estimate is under 1200 s."""
    timer = MissionTimer()
    timer.missionStartTime = 0.0
    # elapsed = 0, pre_hoc = 100 → 100 < 1200
    with patch("time.monotonic", return_value=0.0):
        assert timer.canStartCycle(pre_hoc_time_s=100.0) is True


def test_first_cycle_halted_when_prehoc_would_overrun():
    """First cycle is denied when elapsed + pre_hoc estimate would exceed 1200 s."""
    timer = MissionTimer()
    timer.missionStartTime = 0.0
    # elapsed = 1100, pre_hoc = 200 → 1300 >= 1200
    with patch("time.monotonic", return_value=1100.0):
        assert timer.canStartCycle(pre_hoc_time_s=200.0) is False


def test_first_cycle_raises_when_no_prehoc_supplied():
    """canStartCycle raises ValueError when completedCycles==0 and no estimate given."""
    timer = MissionTimer()
    with patch("time.monotonic", return_value=0.0), pytest.raises(
        ValueError, match="pre_hoc_time_s"
    ):
        timer.canStartCycle()


def test_later_cycle_allowed_when_worst_cycle_fits(monkeypatch):
    """Verify canStartCycle() returns True when elapsed + worstCycleTime < 1200."""
    timer = MissionTimer()

    timer.missionStartTime = 0.0
    timer.worstCycleTime = 100.0
    timer.completedCycles = 1

    # monotonic() returns 1000 — elapsed 1000 + worst 100 = 1100 < 1200 → True
    monkeypatch.setattr(time, "monotonic", lambda: 1000.0)
    assert timer.canStartCycle() is True


def test_later_cycle_halted_when_worst_cycle_would_overrun(monkeypatch):
    """Verify canStartCycle() returns False when elapsed + worstCycleTime >= 1200."""
    timer = MissionTimer()

    timer.missionStartTime = 0.0
    timer.worstCycleTime = 100.0
    timer.completedCycles = 1

    # monotonic() returns 1150 — elapsed 1150 + worst 100 = 1250 >= 1200 → False
    monkeypatch.setattr(time, "monotonic", lambda: 1150.0)
    assert timer.canStartCycle() is False


def test_later_cycle_ignores_prehoc_time_s(monkeypatch):
    """pre_hoc_time_s is ignored when completedCycles > 0; worst cycle time is used."""
    timer = MissionTimer()
    timer.missionStartTime = 0.0
    timer.worstCycleTime = 50.0
    timer.completedCycles = 1

    # Even with a huge pre_hoc value, the post-hoc path uses worstCycleTime
    # elapsed=100, worstCycle=50 → 150 < 1200 → allowed despite large pre_hoc arg
    monkeypatch.setattr(time, "monotonic", lambda: 100.0)
    assert timer.canStartCycle(pre_hoc_time_s=9999.0) is True
