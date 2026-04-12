"""Behaviour tests for the excavation telemetry mock."""

from types import SimpleNamespace

from builtin_interfaces.msg import Time

import lunabot_excavation.excavation_telemetry_mock as mock_module
from lunabot_excavation.excavation_telemetry_mock import ExcavationTelemetryMock
from lunabot_interfaces.msg import ExcavationCommand, ExcavationTelemetry


def _mock_node():
    node = object.__new__(ExcavationTelemetryMock)
    node._estop_active = False
    node._driver_fault = False
    node._fault_code = ExcavationTelemetry.FAULT_NONE
    node._home_switch = True
    node._motor_enabled = False
    node._motor_current_a = 0.0
    node._pending_mode = None
    node._pending_started_at = None
    node._published = []
    node.get_parameter = lambda name: SimpleNamespace(
        value={
            "publish_period_s": 0.1,
            "command_transition_s": 0.2,
            "stop_transition_s": 0.2,
            "home_switch": True,
            "motor_current_a": 12.0,
            "fault_on_start_code": ExcavationTelemetry.FAULT_NONE,
            "fault_on_stop_code": ExcavationTelemetry.FAULT_NONE,
        }[name]
    )
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time())
    )
    node._telemetry_pub = SimpleNamespace(publish=node._published.append)
    return node


def test_start_command_enables_motor_after_delay(monkeypatch):
    node = _mock_node()
    times = iter([10.0, 10.3])

    monkeypatch.setattr(mock_module, "monotonic", lambda: next(times))

    command = ExcavationCommand()
    command.command = ExcavationCommand.COMMAND_START
    node._handle_command(command)
    node._tick()

    assert node._motor_enabled is True
    assert node._motor_current_a == 12.0
    assert node._fault_code == ExcavationTelemetry.FAULT_NONE
    assert node._published[-1].motor_enabled is True


def test_stop_fault_is_published_after_delay(monkeypatch):
    node = _mock_node()
    times = iter([20.0, 20.3])
    node._motor_enabled = True
    node._motor_current_a = 12.0
    node.get_parameter = lambda name: SimpleNamespace(
        value={
            "publish_period_s": 0.1,
            "command_transition_s": 0.2,
            "stop_transition_s": 0.2,
            "home_switch": True,
            "motor_current_a": 12.0,
            "fault_on_start_code": ExcavationTelemetry.FAULT_NONE,
            "fault_on_stop_code": ExcavationTelemetry.FAULT_DRIVER,
        }[name]
    )

    monkeypatch.setattr(mock_module, "monotonic", lambda: next(times))

    command = ExcavationCommand()
    command.command = ExcavationCommand.COMMAND_STOP
    node._handle_command(command)
    node._tick()

    assert node._motor_enabled is False
    assert node._driver_fault is True
    assert node._fault_code == ExcavationTelemetry.FAULT_DRIVER
    assert node._published[-1].driver_fault is True


def test_home_command_clears_switch_then_restores_it(monkeypatch):
    node = _mock_node()
    times = iter([30.0, 30.3])
    node._home_switch = True

    monkeypatch.setattr(mock_module, "monotonic", lambda: next(times))

    command = ExcavationCommand()
    command.command = ExcavationCommand.COMMAND_HOME
    node._handle_command(command)

    assert node._home_switch is False

    node._tick()

    assert node._home_switch is True
    assert node._published[-1].home_switch is True


def test_clear_fault_command_clears_latched_fault_immediately(monkeypatch):
    node = _mock_node()
    times = iter([40.0])
    node._driver_fault = True
    node._fault_code = ExcavationTelemetry.FAULT_DRIVER

    monkeypatch.setattr(mock_module, "monotonic", lambda: next(times))

    command = ExcavationCommand()
    command.command = ExcavationCommand.COMMAND_CLEAR_FAULT
    node._handle_command(command)
    node._tick()

    assert node._driver_fault is False
    assert node._fault_code == ExcavationTelemetry.FAULT_NONE
    assert node._published[-1].fault_code == ExcavationTelemetry.FAULT_NONE
