"""Unit tests for MissionManager FSM transition logic."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

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
    manager._max_nav_retries = 2
    manager._max_exc_retries = 1
    manager._max_dep_retries = 1
    manager._max_cycles = 10
    manager._cycle_count = 0
    manager._estop_active = False
    manager._motion_inhibited = False
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
# ACQUIRE_TAG branching (first cycle vs. later cycles)
# ------------------------------------------------------------------


def test_acquire_tag_transitions_to_prehoc_on_first_cycle(monkeypatch):
    """ACQUIRE_TAG must transition to PREHOC_TRAVERSAL when completedCycles == 0."""
    manager = _make_manager(monkeypatch)
    mock_timer = MagicMock(spec=MissionTimer)
    mock_timer.completedCycles = 0
    manager._timer = mock_timer
    next_state = manager._handle_acquire_tag()
    mock_timer.beginCycle.assert_called_once()
    assert next_state == MissionState.PREHOC_TRAVERSAL


def test_acquire_tag_transitions_to_turn_to_excavation_on_later_cycles(monkeypatch):
    """ACQUIRE_TAG must transition to TURN_TO_EXCAVATION when completedCycles > 0."""
    manager = _make_manager(monkeypatch)
    mock_timer = MagicMock(spec=MissionTimer)
    mock_timer.completedCycles = 1
    manager._timer = mock_timer
    next_state = manager._handle_acquire_tag()
    mock_timer.beginCycle.assert_called_once()
    assert next_state == MissionState.TURN_TO_EXCAVATION


# ------------------------------------------------------------------
# PREHOC_TRAVERSAL branching
# ------------------------------------------------------------------


def test_prehoc_traversal_transitions_to_turn_when_budget_allows(monkeypatch):
    """PREHOC_TRAVERSAL must advance to TURN_TO_EXCAVATION when canStartCycle is True."""
    manager = _make_manager(monkeypatch)
    mock_timer = MagicMock(spec=MissionTimer)
    mock_timer.canStartCycle.return_value = True
    manager._timer = mock_timer
    manager.get_parameter = lambda name: SimpleNamespace(
        value={
            "prehoc_v_estimated_mps": 0.5,
            "prehoc_t_margin_s": 30.0,
            "prehoc_s_start_exc_m": 10.0,
            "prehoc_s_exc_dep_m": 5.0,
        }[name]
    )
    next_state = manager._handle_prehoc_traversal()
    mock_timer.canStartCycle.assert_called_once()
    assert next_state == MissionState.TURN_TO_EXCAVATION


def test_prehoc_traversal_transitions_to_halt_when_budget_exceeded(monkeypatch):
    """PREHOC_TRAVERSAL must advance to HALT_MISSION when canStartCycle is False."""
    manager = _make_manager(monkeypatch)
    mock_timer = MagicMock(spec=MissionTimer)
    mock_timer.canStartCycle.return_value = False
    manager._timer = mock_timer
    manager.get_parameter = lambda name: SimpleNamespace(
        value={
            "prehoc_v_estimated_mps": 0.5,
            "prehoc_t_margin_s": 30.0,
            "prehoc_s_start_exc_m": 10.0,
            "prehoc_s_exc_dep_m": 5.0,
        }[name]
    )
    next_state = manager._handle_prehoc_traversal()
    assert next_state == MissionState.HALT_MISSION


@pytest.mark.parametrize(
    ("parameter_name", "parameter_value"),
    [
        ("prehoc_v_estimated_mps", None),
        ("prehoc_v_estimated_mps", 0.0),
        ("prehoc_v_estimated_mps", -0.5),
        ("prehoc_t_margin_s", -1.0),
        ("prehoc_s_start_exc_m", -1.0),
        ("prehoc_s_exc_dep_m", -1.0),
    ],
)
def test_prehoc_traversal_halts_on_invalid_parameters(
    monkeypatch, parameter_name, parameter_value
):
    """PREHOC_TRAVERSAL must halt when its traversal parameters are unsafe."""
    manager = _make_manager(monkeypatch)
    mock_timer = MagicMock(spec=MissionTimer)
    manager._timer = mock_timer

    parameter_values = {
        "prehoc_v_estimated_mps": 0.5,
        "prehoc_t_margin_s": 30.0,
        "prehoc_s_start_exc_m": 10.0,
        "prehoc_s_exc_dep_m": 5.0,
    }
    parameter_values[parameter_name] = parameter_value
    manager.get_parameter = lambda name: SimpleNamespace(value=parameter_values[name])

    next_state = manager._handle_prehoc_traversal()

    assert next_state == MissionState.HALT_MISSION
    mock_timer.canStartCycle.assert_not_called()


# ------------------------------------------------------------------
# Happy-path cycle
# ------------------------------------------------------------------


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
