"""Behaviour tests for excavation action server goal handling and execution."""

from types import SimpleNamespace

from rclpy.action import GoalResponse

from lunabot_excavation import excavation_action_server as excavation_module
from lunabot_excavation.excavation_action_server import ExcavationActionServer
from lunabot_interfaces.action import Excavate
from lunabot_interfaces.msg import ExcavationStatus, ExcavationTelemetry


def _excavation_server():
    server = object.__new__(ExcavationActionServer)
    server._status = SimpleNamespace(
        state=ExcavationStatus.STATE_READY,
        fault_code=ExcavationStatus.FAULT_NONE,
        estop_active=False,
        motor_current_a=0.0,
    )
    server._latest_telemetry = None
    server._home_client = object()
    server._start_client = object()
    server._stop_client = object()
    server.get_parameter = lambda name: SimpleNamespace(
        value={
            "nominal_duration_s": 8.0,
            "loop_period_s": 0.2,
            "stop_timeout_s": 2.0,
        }[name]
    )
    server.get_logger = lambda: SimpleNamespace(
        info=lambda _msg: None,
        warn=lambda _msg: None,
    )
    server._publish_feedback = lambda _goal_handle, _elapsed: None
    return server


def _excavate_goal(**overrides):
    goal = Excavate.Goal()
    goal.mode = Excavate.Goal.MODE_AUTO
    goal.timeout_s = 5.0
    goal.target_fill_fraction = 0.5
    goal.max_drive_speed_mps = 0.2
    for field_name, value in overrides.items():
        setattr(goal, field_name, value)
    return goal


class _FakeGoalHandle:
    def __init__(self, request, *, cancel_requested=False):
        self.request = request
        self.is_cancel_requested = cancel_requested
        self.is_active = True
        self.abort_count = 0
        self.canceled_count = 0
        self.succeed_count = 0
        self.feedback_messages = []

    def publish_feedback(self, feedback):
        self.feedback_messages.append(feedback)

    def abort(self):
        self.abort_count += 1
        self.is_active = False

    def canceled(self):
        self.canceled_count += 1
        self.is_active = False

    def succeed(self):
        self.succeed_count += 1
        self.is_active = False


def test_valid_excavation_goal_is_accepted():
    server = _excavation_server()
    goal = _excavate_goal(timeout_s=-1.0)

    assert server._is_valid_excavation_goal(goal) is True
    assert server.goal_callback(goal) == GoalResponse.ACCEPT


def test_invalid_excavation_goal_is_rejected():
    server = _excavation_server()
    goal = _excavate_goal(target_fill_fraction=1.5)

    assert server._is_valid_excavation_goal(goal) is False
    assert server.goal_callback(goal) == GoalResponse.REJECT


def test_execute_excavate_returns_timeout_result(monkeypatch):
    server = _excavation_server()
    goal_handle = _FakeGoalHandle(_excavate_goal(timeout_s=1.0))
    trigger_calls = []
    monotonic_values = iter([10.0, 11.5])

    monkeypatch.setattr(excavation_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(excavation_module, "monotonic", lambda: next(monotonic_values))
    monkeypatch.setattr(excavation_module, "sleep", lambda _seconds: None)

    server._call_trigger = lambda client, timeout_s: (
        trigger_calls.append((client, timeout_s)) or SimpleNamespace(success=True)
    )
    server._wait_for_stop_settle = lambda _timeout_s: "settled"

    result = server.execute_excavate(goal_handle)

    assert goal_handle.abort_count == 1
    assert goal_handle.canceled_count == 0
    assert result.success is False
    assert result.reason_code == Excavate.Result.REASON_TIMEOUT
    assert result.failure_reason == "Excavation timeout reached"
    assert trigger_calls == [
        (server._start_client, 2.0),
        (server._stop_client, 2.0),
    ]


def test_execute_excavate_returns_canceled_result(monkeypatch):
    server = _excavation_server()
    goal_handle = _FakeGoalHandle(_excavate_goal(timeout_s=5.0), cancel_requested=True)
    trigger_calls = []
    monotonic_values = iter([20.0, 20.1])

    monkeypatch.setattr(excavation_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(excavation_module, "monotonic", lambda: next(monotonic_values))
    monkeypatch.setattr(excavation_module, "sleep", lambda _seconds: None)

    server._call_trigger = lambda client, timeout_s: (
        trigger_calls.append((client, timeout_s)) or SimpleNamespace(success=True)
    )
    server._wait_for_stop_settle = lambda _timeout_s: "settled"

    result = server.execute_excavate(goal_handle)

    assert goal_handle.abort_count == 0
    assert goal_handle.canceled_count == 1
    assert result.success is False
    assert result.reason_code == Excavate.Result.REASON_CANCELED
    assert result.failure_reason == "Excavation goal canceled by client"
    assert trigger_calls == [
        (server._start_client, 2.0),
        (server._stop_client, 2.0),
    ]


def test_execute_excavate_fault_beats_cancel_during_stop_settle(monkeypatch):
    server = _excavation_server()
    goal_handle = _FakeGoalHandle(_excavate_goal(timeout_s=5.0), cancel_requested=True)
    trigger_calls = []
    monotonic_values = iter([30.0, 30.1])

    monkeypatch.setattr(excavation_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(excavation_module, "monotonic", lambda: next(monotonic_values))
    monkeypatch.setattr(excavation_module, "sleep", lambda _seconds: None)

    def _call_trigger(client, timeout_s):
        trigger_calls.append((client, timeout_s))
        if client is server._stop_client:
            server._status = SimpleNamespace(
                state=ExcavationStatus.STATE_FAULT,
                fault_code=ExcavationStatus.FAULT_OVERCURRENT,
                estop_active=False,
                motor_current_a=0.0,
            )
        return SimpleNamespace(success=True)

    server._call_trigger = _call_trigger
    server._wait_for_stop_settle = lambda _timeout_s: "fault"

    result = server.execute_excavate(goal_handle)

    assert goal_handle.abort_count == 1
    assert goal_handle.canceled_count == 0
    assert result.reason_code == Excavate.Result.REASON_JAM_OR_OVERCURRENT
    assert result.failure_reason == "Excavation jam or overcurrent detected"
    assert trigger_calls == [
        (server._start_client, 2.0),
        (server._stop_client, 2.0),
    ]


def test_execute_excavate_fault_beats_timeout_during_stop_settle(monkeypatch):
    server = _excavation_server()
    goal_handle = _FakeGoalHandle(_excavate_goal(timeout_s=1.0))
    trigger_calls = []
    monotonic_values = iter([40.0, 41.5])

    monkeypatch.setattr(excavation_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(excavation_module, "monotonic", lambda: next(monotonic_values))
    monkeypatch.setattr(excavation_module, "sleep", lambda _seconds: None)

    def _call_trigger(client, timeout_s):
        trigger_calls.append((client, timeout_s))
        if client is server._stop_client:
            server._status = SimpleNamespace(
                state=ExcavationStatus.STATE_FAULT,
                fault_code=ExcavationStatus.FAULT_DRIVER,
                estop_active=False,
                motor_current_a=0.0,
            )
        return SimpleNamespace(success=True)

    server._call_trigger = _call_trigger
    server._wait_for_stop_settle = lambda _timeout_s: "fault"

    result = server.execute_excavate(goal_handle)

    assert goal_handle.abort_count == 1
    assert goal_handle.canceled_count == 0
    assert result.reason_code == Excavate.Result.REASON_DRIVER_FAULT
    assert result.failure_reason == "Excavation driver fault active"
    assert trigger_calls == [
        (server._start_client, 2.0),
        (server._stop_client, 2.0),
    ]


def test_execute_excavate_estop_beats_cancel_during_homing_stop_settle():
    server = _excavation_server()
    goal_handle = _FakeGoalHandle(_excavate_goal(timeout_s=5.0), cancel_requested=True)
    trigger_calls = []

    server._status = SimpleNamespace(
        state=ExcavationStatus.STATE_IDLE,
        fault_code=ExcavationStatus.FAULT_NONE,
        estop_active=False,
        motor_current_a=0.0,
    )

    def _call_trigger(client, timeout_s):
        trigger_calls.append((client, timeout_s))
        if client is server._stop_client:
            server._status = SimpleNamespace(
                state=ExcavationStatus.STATE_FAULT,
                fault_code=ExcavationStatus.FAULT_ESTOP,
                estop_active=True,
                motor_current_a=0.0,
            )
        return SimpleNamespace(success=True)

    server._call_trigger = _call_trigger
    server._wait_for_ready_or_fault = lambda *_args: ("cancel", 0.8)
    server._wait_for_stop_settle = lambda _timeout_s: "fault"

    result = server.execute_excavate(goal_handle)

    assert goal_handle.abort_count == 1
    assert goal_handle.canceled_count == 0
    assert result.reason_code == Excavate.Result.REASON_ESTOP
    assert result.failure_reason == "Excavation estop active"
    assert trigger_calls == [
        (server._home_client, 2.0),
        (server._stop_client, 2.0),
    ]


def test_wait_for_stop_settle_accepts_motor_disabled_telemetry(monkeypatch):
    server = _excavation_server()
    server._status = SimpleNamespace(
        state=ExcavationStatus.STATE_STOPPING,
        fault_code=ExcavationStatus.FAULT_NONE,
        estop_active=False,
        motor_current_a=0.0,
    )
    server._latest_telemetry = SimpleNamespace(
        estop_active=False,
        driver_fault=False,
        fault_code=ExcavationTelemetry.FAULT_NONE,
        motor_enabled=False,
    )

    monkeypatch.setattr(excavation_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(excavation_module, "sleep", lambda _seconds: None)

    assert server._wait_for_stop_settle(2.0) == "settled"


def test_wait_for_stop_settle_reports_telemetry_fault(monkeypatch):
    server = _excavation_server()
    server._status = SimpleNamespace(
        state=ExcavationStatus.STATE_STOPPING,
        fault_code=ExcavationStatus.FAULT_NONE,
        estop_active=False,
        motor_current_a=0.0,
    )
    server._latest_telemetry = SimpleNamespace(
        estop_active=False,
        driver_fault=True,
        fault_code=ExcavationTelemetry.FAULT_NONE,
        motor_enabled=True,
    )

    monkeypatch.setattr(excavation_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(excavation_module, "sleep", lambda _seconds: None)

    assert server._wait_for_stop_settle(2.0) == "fault"


def test_execute_excavate_succeeds_when_telemetry_shows_stopped(monkeypatch):
    server = _excavation_server()
    goal_handle = _FakeGoalHandle(_excavate_goal(timeout_s=10.0))
    trigger_calls = []
    monotonic_values = iter([50.0, 51.0, 51.1])

    server._status = SimpleNamespace(
        state=ExcavationStatus.STATE_EXCAVATING,
        fault_code=ExcavationStatus.FAULT_NONE,
        estop_active=False,
        motor_current_a=12.0,
    )
    server._latest_telemetry = SimpleNamespace(
        estop_active=False,
        driver_fault=False,
        fault_code=ExcavationTelemetry.FAULT_NONE,
        motor_enabled=False,
    )

    monkeypatch.setattr(excavation_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(excavation_module, "monotonic", lambda: next(monotonic_values))
    monkeypatch.setattr(excavation_module, "sleep", lambda _seconds: None)

    def _call_trigger(client, timeout_s):
        trigger_calls.append((client, timeout_s))
        return SimpleNamespace(success=True)

    server._call_trigger = _call_trigger

    result = server.execute_excavate(goal_handle)

    assert goal_handle.succeed_count == 1
    assert result.success is True
    assert result.reason_code == Excavate.Result.REASON_SUCCESS
    assert trigger_calls == [
        (server._start_client, 2.0),
        (server._stop_client, 2.0),
    ]
