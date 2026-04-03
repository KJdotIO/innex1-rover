"""Behaviour tests for excavation controller state transitions."""

from types import SimpleNamespace

from lunabot_excavation.excavation_controller import (
    ActiveRun,
    ExcavationController,
    ExcavationState,
)
from lunabot_interfaces.msg import ExcavationCommand, ExcavationStatus


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


def test_home_command_leads_to_ready_state():
    controller = _controller()

    response = controller._handle_home(None, None)

    assert response.success is True
    assert controller._state is ExcavationState.HOMING
    assert controller._published_commands == [ExcavationCommand.COMMAND_HOME]

    controller._latest_telemetry = _telemetry(home_switch=True)
    controller._tick()

    assert controller._state is ExcavationState.READY


def test_start_and_stop_transition_cleanly():
    controller = _controller()
    controller._state = ExcavationState.READY

    start_response = controller._handle_start(None, None)

    assert start_response.success is True
    assert controller._state is ExcavationState.STARTING
    assert controller._published_commands == [ExcavationCommand.COMMAND_START]

    controller._latest_telemetry = _telemetry(home_switch=True, motor_enabled=True)
    controller._tick()

    assert controller._state is ExcavationState.EXCAVATING

    stop_response = controller._handle_stop(None, None)

    assert stop_response.success is True
    assert controller._state is ExcavationState.STOPPING
    assert controller._published_commands == [
        ExcavationCommand.COMMAND_START,
        ExcavationCommand.COMMAND_STOP,
    ]

    controller._latest_telemetry = _telemetry(home_switch=True, motor_enabled=False)
    controller._tick()

    assert controller._state is ExcavationState.READY
