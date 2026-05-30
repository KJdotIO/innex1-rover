"""Behaviour tests for excavation bench tooling and jog handling."""

from types import SimpleNamespace

import pytest

import lunabot_excavation.excavation_bench as bench_module
from lunabot_excavation.excavation_controller import (
    ActiveRun,
    ExcavationController,
    ExcavationState,
)
from lunabot_interfaces.msg import ExcavationCommand, ExcavationStatus
from lunabot_interfaces.srv import ExcavationJog


def _controller():
    controller = object.__new__(ExcavationController)
    controller._state = ExcavationState.IDLE
    controller._last_fault_code = ExcavationStatus.FAULT_NONE
    controller._latest_telemetry = None
    controller._active_run = ActiveRun.NONE
    controller._jog_duration_s = None
    controller._jog_started_at = None
    controller.get_parameter = lambda _name: SimpleNamespace(value=2.0)
    controller._published_commands = []
    controller._publish_command = controller._published_commands.append
    controller._publish_status = lambda: None
    return controller


def _telemetry(*, home_switch=False, motor_enabled=False, fault_code=0):
    return SimpleNamespace(
        estop_active=False,
        driver_fault=False,
        home_switch=home_switch,
        motor_enabled=motor_enabled,
        motor_current_a=0.0,
        fault_code=fault_code,
    )


def test_jog_rejected_when_not_ready():
    controller = _controller()
    request = ExcavationJog.Request()
    request.duration_s = 0.5

    response = controller._handle_jog_forward(request, None)

    assert response.success is False
    assert "READY" in response.message


def test_jog_rejected_for_non_finite_duration():
    controller = _controller()
    controller._state = ExcavationState.READY
    request = ExcavationJog.Request()
    request.duration_s = float("nan")

    response = controller._handle_jog_forward(request, None)

    assert response.success is False
    assert "duration" in response.message


def test_jog_timer_starts_when_motion_is_confirmed(monkeypatch):
    controller = _controller()
    controller._state = ExcavationState.READY

    request = ExcavationJog.Request()
    request.duration_s = 0.5
    response = controller._handle_jog_forward(request, None)
    assert response.success is True
    assert controller._state is ExcavationState.STARTING

    times = iter([100.0, 100.4, 100.6])
    monkeypatch.setattr(
        "lunabot_excavation.excavation_controller.monotonic",
        lambda: next(times),
    )

    controller._latest_telemetry = _telemetry(motor_enabled=False)
    controller._tick()
    assert controller._state is ExcavationState.STARTING
    assert controller._published_commands == [ExcavationCommand.COMMAND_START]

    controller._latest_telemetry = _telemetry(motor_enabled=True)
    controller._tick()
    assert controller._state is ExcavationState.EXCAVATING
    assert controller._jog_started_at == 100.0

    controller._tick()
    assert controller._state is ExcavationState.STOPPING
    assert controller._published_commands == [
        ExcavationCommand.COMMAND_START,
        ExcavationCommand.COMMAND_STOP,
    ]


def test_bench_main_returns_clean_error_on_runtime_failure(monkeypatch, capsys):
    class StubBench:
        def _wait_for_status(self, _timeout):
            raise RuntimeError("status timed out")

        def destroy_node(self):
            return None

    monkeypatch.setattr(bench_module, "ExcavationBench", StubBench)
    monkeypatch.setattr(bench_module.rclpy, "init", lambda: None)
    monkeypatch.setattr(bench_module.rclpy, "shutdown", lambda: None)

    with pytest.raises(SystemExit) as exc_info:
        bench_module.main(["status"])

    captured = capsys.readouterr()
    assert exc_info.value.code == 1
    assert "error: status timed out" in captured.out
