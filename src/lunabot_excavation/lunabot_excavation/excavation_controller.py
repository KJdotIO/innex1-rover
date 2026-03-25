"""Excavation controller skeleton."""

from enum import Enum

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from lunabot_interfaces.msg import (
    ExcavationCommand,
    ExcavationStatus,
    ExcavationTelemetry,
)


class ExcavationState(str, Enum):
    """Controller states for the excavation subsystem."""

    IDLE = "idle"
    HOMING = "homing"
    READY = "ready"
    STARTING = "starting"
    EXCAVATING = "excavating"
    STOPPING = "stopping"
    FAULT = "fault"


class ExcavationController(Node):
    """Own the excavation subsystem state machine."""

    def __init__(self):
        """Initialise excavation state and hardware-facing interfaces."""
        super().__init__("excavation_controller")

        self.declare_parameter("control_period_s", 0.1)

        self._state = ExcavationState.IDLE
        self._last_fault_code = ExcavationStatus.FAULT_NONE
        self._latest_telemetry = None

        self._command_pub = self.create_publisher(
            ExcavationCommand,
            "/excavation/command",
            10,
        )
        self._status_pub = self.create_publisher(
            ExcavationStatus,
            "/excavation/status",
            10,
        )
        self._telemetry_sub = self.create_subscription(
            ExcavationTelemetry,
            "/excavation/telemetry",
            self._handle_telemetry,
            10,
        )
        self.create_service(Trigger, "/excavation/home", self._handle_home)
        self.create_service(Trigger, "/excavation/start", self._handle_start)
        self.create_service(Trigger, "/excavation/stop", self._handle_stop)
        self.create_service(
            Trigger,
            "/excavation/clear_fault",
            self._handle_clear_fault,
        )
        self._control_timer = self.create_timer(
            float(self.get_parameter("control_period_s").value),
            self._tick,
        )

        self.get_logger().info("Excavation controller skeleton ready")

    def _handle_telemetry(self, msg: ExcavationTelemetry):
        """Store the latest excavation telemetry sample."""
        self._latest_telemetry = msg

    def _publish_command(self, command: int):
        """Publish one excavation hardware command."""
        msg = ExcavationCommand()
        msg.command = command
        self._command_pub.publish(msg)

    def _set_state(self, state: ExcavationState):
        """Transition the controller to a new state."""
        self._state = state

    def _publish_status(self):
        """Publish the current excavation controller state."""
        msg = ExcavationStatus()
        if self._latest_telemetry is not None:
            msg.header = self._latest_telemetry.header
            msg.estop_active = self._latest_telemetry.estop_active
            msg.driver_fault = self._latest_telemetry.driver_fault
            msg.homed = self._latest_telemetry.home_switch
            msg.motor_enabled = self._latest_telemetry.motor_enabled
            msg.motor_current_a = self._latest_telemetry.motor_current_a
        msg.fault_code = self._last_fault_code
        msg.state = {
            ExcavationState.IDLE: ExcavationStatus.STATE_IDLE,
            ExcavationState.HOMING: ExcavationStatus.STATE_HOMING,
            ExcavationState.READY: ExcavationStatus.STATE_READY,
            ExcavationState.STARTING: ExcavationStatus.STATE_STARTING,
            ExcavationState.EXCAVATING: ExcavationStatus.STATE_EXCAVATING,
            ExcavationState.STOPPING: ExcavationStatus.STATE_STOPPING,
            ExcavationState.FAULT: ExcavationStatus.STATE_FAULT,
        }[self._state]
        self._status_pub.publish(msg)

    def _enter_fault(self, fault_code: int):
        """Latch a fault and force a stop command."""
        self._set_state(ExcavationState.FAULT)
        self._last_fault_code = fault_code
        self._publish_command(ExcavationCommand.COMMAND_STOP)

    @staticmethod
    def _trigger_response(success: bool, message: str):
        """Build one Trigger response."""
        response = Trigger.Response()
        response.success = bool(success)
        response.message = str(message)
        return response

    def _handle_home(self, _request, _response):
        """Start a homing request if the controller is able to do so."""
        if self._state is ExcavationState.FAULT:
            return self._trigger_response(False, "Excavation controller faulted")
        if self._state in (
            ExcavationState.HOMING,
            ExcavationState.STARTING,
            ExcavationState.EXCAVATING,
            ExcavationState.STOPPING,
        ):
            return self._trigger_response(
                False,
                f"Excavation controller busy in state '{self._state.value}'",
            )
        self._publish_command(ExcavationCommand.COMMAND_HOME)
        self._set_state(ExcavationState.HOMING)
        return self._trigger_response(True, "Excavation homing started")

    def _handle_start(self, _request, _response):
        """Start excavation if the subsystem is ready."""
        if self._state is not ExcavationState.READY:
            return self._trigger_response(
                False,
                f"Excavation start requires READY state, got '{self._state.value}'",
            )
        self._publish_command(ExcavationCommand.COMMAND_START)
        self._set_state(ExcavationState.STARTING)
        return self._trigger_response(True, "Excavation start accepted")

    def _handle_stop(self, _request, _response):
        """Request a stop on the excavation subsystem."""
        self._publish_command(ExcavationCommand.COMMAND_STOP)
        if self._state is not ExcavationState.FAULT:
            self._set_state(ExcavationState.STOPPING)
        return self._trigger_response(True, "Excavation stop requested")

    def _handle_clear_fault(self, _request, _response):
        """Clear a latched excavation fault."""
        if self._state is not ExcavationState.FAULT:
            return self._trigger_response(False, "Excavation controller is not faulted")
        self._publish_command(ExcavationCommand.COMMAND_CLEAR_FAULT)
        self._last_fault_code = ExcavationStatus.FAULT_NONE
        self._set_state(ExcavationState.IDLE)
        return self._trigger_response(True, "Excavation fault cleared")

    def _tick(self):
        """Advance the excavation controller loop."""
        if self._latest_telemetry is None:
            self._publish_status()
            return

        if self._latest_telemetry.estop_active:
            self._enter_fault(ExcavationStatus.FAULT_ESTOP)
            self._publish_status()
            return

        if self._latest_telemetry.driver_fault:
            self._enter_fault(ExcavationStatus.FAULT_DRIVER)
            self._publish_status()
            return

        if self._latest_telemetry.fault_code != ExcavationTelemetry.FAULT_NONE:
            self._enter_fault(self._latest_telemetry.fault_code)
            self._publish_status()
            return

        if self._state is ExcavationState.HOMING and self._latest_telemetry.home_switch:
            self._set_state(ExcavationState.READY)
        elif (
            self._state is ExcavationState.STARTING
            and self._latest_telemetry.motor_enabled
        ):
            self._set_state(ExcavationState.EXCAVATING)
        elif (
            self._state is ExcavationState.STOPPING
            and not self._latest_telemetry.motor_enabled
        ):
            if self._latest_telemetry.home_switch:
                self._set_state(ExcavationState.READY)
            else:
                self._set_state(ExcavationState.IDLE)

        self._publish_status()


def main(args=None):
    """Run the excavation controller node."""
    rclpy.init(args=args)
    node = ExcavationController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
