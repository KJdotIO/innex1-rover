"""Unit tests for MissionManager FSM transition logic."""

from types import SimpleNamespace
from unittest.mock import MagicMock

from lunabot_bringup.mission_manager import MissionManager, MissionState
from lunabot_bringup.mission_timer import MissionTimer


def _fake_logger():
    """Return a lightweight logger stub."""
    return SimpleNamespace(
        info=lambda _msg: None,
        error=lambda _msg: None,
        warn=lambda _msg: None,
    )


def _make_manager(monkeypatch):
    """Construct a MissionManager without spinning a real ROS node."""
    monkeypatch.setattr(
        "rclpy.node.Node.__init__",
        lambda _self, *_args, **_kwargs: None,
    )
    manager = object.__new__(MissionManager)
    manager.get_logger = _fake_logger
    manager._state = MissionState.INITIALIZE_MISSION
    manager._timer = None
    manager._navigate_client = MagicMock()
    manager._excavate_client = MagicMock()
    manager._deposit_client = MagicMock()
    return manager


# ------------------------------------------------------------------
# Initial state
# ------------------------------------------------------------------


def test_initial_state_is_initialize_mission(monkeypatch):
    """Initial FSM state must be INITIALIZE_MISSION."""
    manager = _make_manager(monkeypatch)
    assert manager._state == MissionState.INITIALIZE_MISSION


# ------------------------------------------------------------------
# INITIALIZE_MISSION
# ------------------------------------------------------------------


def test_initialize_mission_transitions_to_acquire_tag(monkeypatch):
    """INITIALIZE_MISSION must transition to ACQUIRE_TAG and create a timer."""
    manager = _make_manager(monkeypatch)
    next_state = manager._handle_initialize_mission()
    assert next_state == MissionState.ACQUIRE_TAG
    assert isinstance(manager._timer, MissionTimer)


# ------------------------------------------------------------------
# Happy-path cycle
# ------------------------------------------------------------------


def test_acquire_tag_transitions_to_turn_to_excavation(monkeypatch):
    """ACQUIRE_TAG must call beginCycle and advance to TURN_TO_EXCAVATION."""
    manager = _make_manager(monkeypatch)
    manager._timer = MagicMock(spec=MissionTimer)
    next_state = manager._handle_acquire_tag()
    manager._timer.beginCycle.assert_called_once()
    assert next_state == MissionState.TURN_TO_EXCAVATION


def test_turn_to_excavation_transitions_to_nav_to_excavation(monkeypatch):
    """TURN_TO_EXCAVATION must transition to NAV_TO_EXCAVATION."""
    manager = _make_manager(monkeypatch)
    next_state = manager._handle_turn_to_excavation()
    assert next_state == MissionState.NAV_TO_EXCAVATION


def test_nav_to_excavation_success_transitions_to_excavate(monkeypatch):
    """NAV_TO_EXCAVATION must advance to EXCAVATE on success."""
    manager = _make_manager(monkeypatch)
    monkeypatch.setattr(
        manager, "_send_navigate_to_excavation", lambda: (True, "ok")
    )
    next_state = manager._handle_nav_to_excavation()
    assert next_state == MissionState.EXCAVATE


def test_excavate_success_transitions_to_turn_to_deposition(monkeypatch):
    """EXCAVATE must advance to TURN_TO_DEPOSITION on success."""
    manager = _make_manager(monkeypatch)
    monkeypatch.setattr(manager, "_send_excavate", lambda: (True, "ok"))
    next_state = manager._handle_excavate()
    assert next_state == MissionState.TURN_TO_DEPOSITION


def test_turn_to_deposition_transitions_to_nav_to_deposition(monkeypatch):
    """TURN_TO_DEPOSITION must transition to NAV_TO_DEPOSITION."""
    manager = _make_manager(monkeypatch)
    next_state = manager._handle_turn_to_deposition()
    assert next_state == MissionState.NAV_TO_DEPOSITION


def test_nav_to_deposition_success_transitions_to_deposit(monkeypatch):
    """NAV_TO_DEPOSITION must advance to DEPOSIT on success."""
    manager = _make_manager(monkeypatch)
    monkeypatch.setattr(
        manager, "_send_navigate_to_deposition", lambda: (True, "ok")
    )
    next_state = manager._handle_nav_to_deposition()
    assert next_state == MissionState.DEPOSIT


def test_deposit_success_transitions_to_check_next_cycle_time(monkeypatch):
    """DEPOSIT must advance to CHECK_NEXT_CYCLE_TIME on success."""
    manager = _make_manager(monkeypatch)
    monkeypatch.setattr(manager, "_send_deposit", lambda: (True, "ok"))
    next_state = manager._handle_deposit()
    assert next_state == MissionState.CHECK_NEXT_CYCLE_TIME


# ------------------------------------------------------------------
# CHECK_NEXT_CYCLE_TIME branching
# ------------------------------------------------------------------


def test_check_next_cycle_time_loops_when_can_start_cycle(monkeypatch):
    """CHECK_NEXT_CYCLE_TIME must return ACQUIRE_TAG when canStartCycle is True."""
    manager = _make_manager(monkeypatch)
    mock_timer = MagicMock(spec=MissionTimer)
    mock_timer.canStartCycle.return_value = True
    manager._timer = mock_timer
    next_state = manager._handle_check_next_cycle_time()
    mock_timer.recordCycleTime.assert_called_once()
    assert next_state == MissionState.ACQUIRE_TAG


def test_check_next_cycle_time_halts_when_cannot_start_cycle(monkeypatch):
    """CHECK_NEXT_CYCLE_TIME must return HALT_MISSION when canStartCycle is False."""
    manager = _make_manager(monkeypatch)
    mock_timer = MagicMock(spec=MissionTimer)
    mock_timer.canStartCycle.return_value = False
    manager._timer = mock_timer
    next_state = manager._handle_check_next_cycle_time()
    assert next_state == MissionState.HALT_MISSION


# ------------------------------------------------------------------
# Failure paths → SAFE_FAIL
# ------------------------------------------------------------------


def test_nav_to_excavation_failure_transitions_to_safe_fail(monkeypatch):
    """NAV_TO_EXCAVATION failure must transition to SAFE_FAIL."""
    manager = _make_manager(monkeypatch)
    monkeypatch.setattr(
        manager, "_send_navigate_to_excavation", lambda: (False, "nav failed")
    )
    next_state = manager._handle_nav_to_excavation()
    assert next_state == MissionState.SAFE_FAIL


def test_excavate_failure_transitions_to_safe_fail(monkeypatch):
    """EXCAVATE failure must transition to SAFE_FAIL."""
    manager = _make_manager(monkeypatch)
    monkeypatch.setattr(manager, "_send_excavate", lambda: (False, "excavate failed"))
    next_state = manager._handle_excavate()
    assert next_state == MissionState.SAFE_FAIL


def test_nav_to_deposition_failure_transitions_to_safe_fail(monkeypatch):
    """NAV_TO_DEPOSITION failure must transition to SAFE_FAIL."""
    manager = _make_manager(monkeypatch)
    monkeypatch.setattr(
        manager, "_send_navigate_to_deposition", lambda: (False, "nav failed")
    )
    next_state = manager._handle_nav_to_deposition()
    assert next_state == MissionState.SAFE_FAIL


def test_deposit_failure_transitions_to_safe_fail(monkeypatch):
    """DEPOSIT failure must transition to SAFE_FAIL."""
    manager = _make_manager(monkeypatch)
    monkeypatch.setattr(manager, "_send_deposit", lambda: (False, "deposit failed"))
    next_state = manager._handle_deposit()
    assert next_state == MissionState.SAFE_FAIL


# ------------------------------------------------------------------
# SAFE_FAIL → HALT_MISSION
# ------------------------------------------------------------------


def test_safe_fail_transitions_to_halt_mission(monkeypatch):
    """SAFE_FAIL must transition to HALT_MISSION."""
    manager = _make_manager(monkeypatch)
    next_state = manager._handle_safe_fail()
    assert next_state == MissionState.HALT_MISSION
