"""Bench telemetry mock for the excavation controller."""

from time import monotonic

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from lunabot_interfaces.msg import ExcavationCommand, ExcavationTelemetry


class ExcavationTelemetryMock(Node):
    """Publish simple excavation telemetry for bench and launch smoke tests."""

    def __init__(self):
        """Initialise mock state, command subscription, and telemetry output."""
        super().__init__("excavation_telemetry_mock")

        self.declare_parameter("publish_period_s", 0.1)
        self.declare_parameter("command_transition_s", 0.2)
        self.declare_parameter("stop_transition_s", 0.2)
        self.declare_parameter("home_switch", True)
        self.declare_parameter("motor_current_a", 12.0)
        self.declare_parameter("fault_on_start_code", 0)
        self.declare_parameter("fault_on_stop_code", 0)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._telemetry_pub = self.create_publisher(
            ExcavationTelemetry,
            "/excavation/telemetry",
            qos,
        )
        self.create_subscription(
            ExcavationCommand,
            "/excavation/command",
            self._handle_command,
            qos,
        )

        self._estop_active = False
        self._driver_fault = False
        self._fault_code = ExcavationTelemetry.FAULT_NONE
        self._home_switch = bool(self.get_parameter("home_switch").value)
        self._motor_enabled = False
        self._motor_current_a = 0.0

        self._pending_mode = None
        self._pending_started_at = None

        self.create_timer(
            float(self.get_parameter("publish_period_s").value),
            self._tick,
        )

        self.get_logger().info("Excavation telemetry mock ready")

    def _transition_delay_s(self, pending_mode: str):
        """Return the command settle delay for the pending mode."""
        if pending_mode == "stop":
            return float(self.get_parameter("stop_transition_s").value)
        return float(self.get_parameter("command_transition_s").value)

    def _set_fault(self, fault_code: int):
        """Apply one telemetry-visible fault state."""
        self._fault_code = int(fault_code)
        self._motor_enabled = False
        self._motor_current_a = 0.0
        self._driver_fault = fault_code == ExcavationTelemetry.FAULT_DRIVER
        self._estop_active = fault_code == ExcavationTelemetry.FAULT_ESTOP

    def _clear_fault(self):
        """Clear any latched mock fault state."""
        self._estop_active = False
        self._driver_fault = False
        self._fault_code = ExcavationTelemetry.FAULT_NONE

    def _handle_command(self, msg: ExcavationCommand):
        """React to one controller command with simple deterministic telemetry."""
        command = int(msg.command)
        self._pending_started_at = monotonic()

        if command == ExcavationCommand.COMMAND_CLEAR_FAULT:
            self._clear_fault()
            self._pending_mode = None
            return

        if command == ExcavationCommand.COMMAND_HOME:
            self._clear_fault()
            self._home_switch = False
            self._motor_enabled = False
            self._motor_current_a = 0.0
            self._pending_mode = "home"
            return

        if command == ExcavationCommand.COMMAND_START:
            self._clear_fault()
            self._pending_mode = "start"
            return

        if command == ExcavationCommand.COMMAND_STOP:
            self._pending_mode = "stop"

    def _tick(self):
        """Advance any pending transition and publish one telemetry sample."""
        if self._pending_mode is not None and self._pending_started_at is not None:
            elapsed = monotonic() - self._pending_started_at
            if elapsed >= self._transition_delay_s(self._pending_mode):
                if self._pending_mode == "home":
                    self._home_switch = True
                elif self._pending_mode == "start":
                    fault_code = int(self.get_parameter("fault_on_start_code").value)
                    if fault_code != ExcavationTelemetry.FAULT_NONE:
                        self._set_fault(fault_code)
                    else:
                        self._motor_enabled = True
                        self._motor_current_a = float(
                            self.get_parameter("motor_current_a").value
                        )
                elif self._pending_mode == "stop":
                    self._motor_enabled = False
                    self._motor_current_a = 0.0
                    fault_code = int(self.get_parameter("fault_on_stop_code").value)
                    if fault_code != ExcavationTelemetry.FAULT_NONE:
                        self._set_fault(fault_code)
                self._pending_mode = None
                self._pending_started_at = None

        msg = ExcavationTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.estop_active = self._estop_active
        msg.driver_fault = self._driver_fault
        msg.home_switch = self._home_switch
        msg.motor_enabled = self._motor_enabled
        msg.motor_current_a = float(self._motor_current_a)
        msg.fault_code = int(self._fault_code)
        self._telemetry_pub.publish(msg)


def main(args=None):
    """Run the excavation telemetry mock node."""
    rclpy.init(args=args)
    node = None
    try:
        node = ExcavationTelemetryMock()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
