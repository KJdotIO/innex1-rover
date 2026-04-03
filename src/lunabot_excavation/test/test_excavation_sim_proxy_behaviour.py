"""Behaviour tests for the excavation sim proxy."""

from types import SimpleNamespace

from builtin_interfaces.msg import Time

from lunabot_excavation import excavation_sim_proxy as sim_proxy_module
from lunabot_excavation.excavation_sim_proxy import ExcavationSimProxy
from lunabot_interfaces.msg import ExcavationCommand, ExcavationTelemetry


def _proxy(**overrides):
    params = {
        "telemetry_period_s": 0.1,
        "home_delay_s": 0.5,
        "start_delay_s": 0.3,
        "stop_delay_s": 0.2,
        "nominal_motor_current_a": 12.0,
        "overcurrent_motor_current_a": 35.0,
        "force_overcurrent": False,
        "force_driver_fault": False,
        "hold_home_switch_false": False,
    }
    params.update(overrides)

    proxy = object.__new__(ExcavationSimProxy)
    proxy._last_command = ExcavationCommand.COMMAND_STOP
    proxy._home_switch = False
    proxy._motor_enabled = False
    proxy._motor_current_a = 0.0
    proxy._fault_code = ExcavationTelemetry.FAULT_NONE
    proxy._home_requested_at = None
    proxy._start_requested_at = None
    proxy._stop_requested_at = None
    proxy.get_parameter = lambda name: SimpleNamespace(value=params[name])
    proxy.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time())
    )
    proxy.get_logger = lambda: SimpleNamespace(warn=lambda _msg: None)
    proxy._published_telemetry = []
    proxy._telemetry_pub = SimpleNamespace(publish=proxy._published_telemetry.append)
    return proxy


def _command(command: int):
    msg = ExcavationCommand()
    msg.command = command
    return msg


def test_home_command_sets_home_switch_after_delay(monkeypatch):
    proxy = _proxy()
    times = iter([10.0, 10.2, 10.6])

    monkeypatch.setattr(sim_proxy_module, "monotonic", lambda: next(times))

    proxy._handle_command(_command(ExcavationCommand.COMMAND_HOME))
    proxy._publish_telemetry()
    proxy._publish_telemetry()

    assert proxy._last_command == ExcavationCommand.COMMAND_HOME
    assert proxy._published_telemetry[0].home_switch is False
    assert proxy._published_telemetry[-1].home_switch is True


def test_start_command_enables_motor_after_delay(monkeypatch):
    proxy = _proxy()
    times = iter([20.0, 20.1, 20.4])

    monkeypatch.setattr(sim_proxy_module, "monotonic", lambda: next(times))

    proxy._handle_command(_command(ExcavationCommand.COMMAND_START))
    proxy._publish_telemetry()
    proxy._publish_telemetry()

    assert proxy._published_telemetry[0].motor_enabled is False
    assert proxy._published_telemetry[-1].motor_enabled is True
    assert proxy._published_telemetry[-1].motor_current_a == 12.0


def test_hold_home_switch_false_prevents_homing_completion(monkeypatch):
    proxy = _proxy(hold_home_switch_false=True)
    times = iter([25.0, 25.6])

    monkeypatch.setattr(sim_proxy_module, "monotonic", lambda: next(times))

    proxy._handle_command(_command(ExcavationCommand.COMMAND_HOME))
    proxy._publish_telemetry()

    assert proxy._published_telemetry[-1].home_switch is False


def test_stop_command_wins_over_pending_start(monkeypatch):
    proxy = _proxy()
    times = iter([30.0, 30.1, 30.35])

    monkeypatch.setattr(sim_proxy_module, "monotonic", lambda: next(times))

    proxy._handle_command(_command(ExcavationCommand.COMMAND_START))
    proxy._handle_command(_command(ExcavationCommand.COMMAND_STOP))
    proxy._publish_telemetry()

    assert proxy._last_command == ExcavationCommand.COMMAND_STOP
    assert proxy._published_telemetry[-1].motor_enabled is False
    assert proxy._published_telemetry[-1].motor_current_a == 0.0


def test_force_overcurrent_fault_is_reported_when_motor_starts(monkeypatch):
    proxy = _proxy(force_overcurrent=True)
    times = iter([40.0, 40.4])

    monkeypatch.setattr(sim_proxy_module, "monotonic", lambda: next(times))

    proxy._handle_command(_command(ExcavationCommand.COMMAND_START))
    proxy._publish_telemetry()

    assert proxy._published_telemetry[-1].motor_enabled is True
    assert (
        proxy._published_telemetry[-1].fault_code
        == ExcavationTelemetry.FAULT_OVERCURRENT
    )
    assert proxy._published_telemetry[-1].motor_current_a == 35.0


def test_clear_fault_clears_latched_fault_when_injection_is_off(monkeypatch):
    proxy = _proxy()
    proxy._fault_code = ExcavationTelemetry.FAULT_OVERCURRENT
    times = iter([50.0, 50.0])

    monkeypatch.setattr(sim_proxy_module, "monotonic", lambda: next(times))

    proxy._handle_command(_command(ExcavationCommand.COMMAND_CLEAR_FAULT))
    proxy._publish_telemetry()

    assert proxy._published_telemetry[-1].fault_code == ExcavationTelemetry.FAULT_NONE


def test_force_driver_fault_is_reported_in_telemetry(monkeypatch):
    proxy = _proxy(force_driver_fault=True)
    monkeypatch.setattr(sim_proxy_module, "monotonic", lambda: 60.0)

    proxy._publish_telemetry()

    assert proxy._published_telemetry[-1].driver_fault is True
    assert proxy._published_telemetry[-1].fault_code == ExcavationTelemetry.FAULT_DRIVER
