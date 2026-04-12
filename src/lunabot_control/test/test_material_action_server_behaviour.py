"""Behaviour tests for deposit action server goal handling and execution."""

from types import SimpleNamespace

from rclpy.action import GoalResponse

from lunabot_control import material_action_server as material_module
from lunabot_control.material_action_server import MaterialActionServer
from lunabot_interfaces.action import Deposit


def _material_server():
    server = object.__new__(MaterialActionServer)
    def _parameter(name):
        values = {
            "deposit_nominal_duration_s": 5.0,
            "loop_period_s": 0.2,
            "force_failure_action": "",
        }
        return SimpleNamespace(value=values[name])

    server.get_parameter = _parameter
    return server


def _deposit_goal(**overrides):
    goal = Deposit.Goal()
    goal.mode = Deposit.Goal.MODE_AUTO
    goal.timeout_s = 5.0
    goal.dump_duration_s = 3.0
    goal.require_close_after_dump = True
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


def test_valid_deposit_goal_is_accepted():
    server = _material_server()
    goal = _deposit_goal(timeout_s=-1.0)

    assert server._is_valid_deposit_goal(goal) is True
    assert server.goal_callback(goal) == GoalResponse.ACCEPT


def test_invalid_deposit_goal_is_rejected():
    server = _material_server()
    goal = _deposit_goal(dump_duration_s=-0.1)

    assert server._is_valid_deposit_goal(goal) is False
    assert server.goal_callback(goal) == GoalResponse.REJECT


def test_execute_deposit_returns_timeout_result(monkeypatch):
    server = _material_server()
    goal_handle = _FakeGoalHandle(_deposit_goal(timeout_s=1.0))
    monotonic_values = iter([30.0, 31.5])

    monkeypatch.setattr(material_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(material_module, "monotonic", lambda: next(monotonic_values))
    monkeypatch.setattr(material_module, "sleep", lambda _seconds: None)

    result = server.execute_deposit(goal_handle)

    assert goal_handle.abort_count == 1
    assert goal_handle.canceled_count == 0
    assert result.success is False
    assert result.reason_code == Deposit.Result.REASON_TIMEOUT
    assert result.failure_reason == "Deposition timeout reached"


def test_execute_deposit_returns_canceled_result(monkeypatch):
    server = _material_server()
    goal_handle = _FakeGoalHandle(_deposit_goal(timeout_s=5.0), cancel_requested=True)
    monotonic_values = iter([40.0, 40.1])

    monkeypatch.setattr(material_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(material_module, "monotonic", lambda: next(monotonic_values))
    monkeypatch.setattr(material_module, "sleep", lambda _seconds: None)

    result = server.execute_deposit(goal_handle)

    assert goal_handle.abort_count == 0
    assert goal_handle.canceled_count == 1
    assert result.success is False
    assert result.reason_code == Deposit.Result.REASON_CANCELED
    assert result.failure_reason == "Deposit goal canceled by client"
