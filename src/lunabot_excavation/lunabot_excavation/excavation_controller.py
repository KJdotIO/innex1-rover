"""Excavation controller skeleton."""

from enum import Enum

import rclpy
from rclpy.node import Node

from lunabot_interfaces.msg import ExcavationCommand, ExcavationTelemetry


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
        self._last_fault_code = ExcavationTelemetry.FAULT_NONE
        self._latest_telemetry = None

        self._command_pub = self.create_publisher(
            ExcavationCommand,
            "/excavation/command",
            10,
        )
        self._telemetry_sub = self.create_subscription(
            ExcavationTelemetry,
            "/excavation/telemetry",
            self._handle_telemetry,
            10,
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

    def _enter_fault(self, fault_code: int):
        """Latch a fault and force a stop command."""
        self._state = ExcavationState.FAULT
        self._last_fault_code = fault_code
        self._publish_command(ExcavationCommand.COMMAND_STOP)

    def _tick(self):
        """Advance the excavation controller loop."""
        if self._latest_telemetry is None:
            return

        if self._latest_telemetry.estop_active:
            self._enter_fault(ExcavationTelemetry.FAULT_ESTOP)
            return

        if self._latest_telemetry.driver_fault:
            self._enter_fault(ExcavationTelemetry.FAULT_DRIVER)


def main(args=None):
    """Run the excavation controller node."""
    rclpy.init(args=args)
    node = ExcavationController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
