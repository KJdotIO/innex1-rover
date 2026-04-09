"""Unit tests for the MissionTimer class."""

from __future__ import annotations

import time

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
    """beginCycle() updates cycleStartTime to a value at least as large as before."""
    timer = MissionTimer()
    old_start = timer.cycleStartTime
    timer.beginCycle()
    assert timer.cycleStartTime >= old_start


def test_record_cycle_time_returns_positive_float_and_increments_counter():
    """recordCycleTime() returns a positive duration and increments completedCycles."""
    timer = MissionTimer()
    timer.beginCycle()
    duration = timer.recordCycleTime()

    assert isinstance(duration, float)
    assert duration >= 0.0
    assert timer.completedCycles == 1


def test_record_cycle_time_updates_worst_cycle_time_for_slower_cycle(monkeypatch):
    """worstCycleTime is updated when a slower cycle is recorded."""
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


def test_record_cycle_time_does_not_update_worst_cycle_time_for_faster_cycle(monkeypatch):
    """worstCycleTime is NOT updated when a faster cycle is recorded."""
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


def test_can_start_cycle_returns_true_when_no_cycles_completed():
    """canStartCycle() always returns True before any cycle has been recorded."""
    timer = MissionTimer()
    assert timer.canStartCycle() is True


def test_can_start_cycle_returns_false_when_budget_would_be_exceeded(monkeypatch):
    """canStartCycle() returns False when elapsed + worstCycleTime >= 1200."""
    timer = MissionTimer()

    # Simulate missionStartTime at t=0
    timer.missionStartTime = 0.0
    # Worst cycle observed: 100 s
    timer.worstCycleTime = 100.0
    # At least one cycle has been completed
    timer.completedCycles = 1

    # monotonic() returns 1150 — elapsed 1150 + worst 100 = 1250 >= 1200 → False
    monkeypatch.setattr(time, "monotonic", lambda: 1150.0)
    assert timer.canStartCycle() is False


def test_can_start_cycle_returns_true_when_budget_is_sufficient(monkeypatch):
    """canStartCycle() returns True when elapsed + worstCycleTime < 1200."""
    timer = MissionTimer()

    timer.missionStartTime = 0.0
    timer.worstCycleTime = 100.0
    timer.completedCycles = 1

    # monotonic() returns 1000 — elapsed 1000 + worst 100 = 1100 < 1200 → True
    monkeypatch.setattr(time, "monotonic", lambda: 1000.0)
    assert timer.canStartCycle() is True
