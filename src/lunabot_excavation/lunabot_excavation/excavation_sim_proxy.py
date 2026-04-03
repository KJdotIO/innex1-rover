"""Simulate excavation hardware for controller and action testing."""

from time import monotonic

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from lunabot_interfaces.msg import ExcavationCommand, ExcavationTelemetry


class ExcavationSimProxy(Node):
    """Bridge excavation commands into deterministic fake telemetry."""

    def __init__(self):
        """Initialise proxy state, command subscription, and telemetry timer."""
        super().__init__("excavation_sim_proxy")

        self.declare_parameter("telemetry_period_s", 0.1)
        self.declare_parameter("home_delay_s", 0.5)
        self.declare_parameter("start_delay_s", 0.3)
        self.declare_parameter("stop_delay_s", 0.2)
        self.declare_parameter("nominal_motor_current_a", 12.0)
        self.declare_parameter("overcurrent_motor_current_a", 35.0)
        self.declare_parameter("force_overcurrent", False)
        self.declare_parameter("force_driver_fault", False)
        self.declare_parameter("hold_home_switch_false", False)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._last_command = ExcavationCommand.COMMAND_STOP
        self._home_switch = False
        self._motor_enabled = False
        self._motor_current_a = 0.0
        self._fault_code = ExcavationTelemetry.FAULT_NONE
        self._home_requested_at = None
        self._start_requested_at = None
        self._stop_requested_at = None

        self.create_subscription(
            ExcavationCommand,
            "/excavation/command",
            self._handle_command,
            command_qos,
        )
        self._telemetry_pub = self.create_publisher(
            ExcavationTelemetry,
            "/excavation/telemetry",
            qos,
        )
        self._telemetry_timer = self.create_timer(
            self._float_parameter("telemetry_period_s"),
            self._publish_telemetry,
        )

        self.get_logger().info("Excavation sim proxy ready")

    def _float_parameter(self, name: str) -> float:
        """Return one float-valued parameter."""
        return float(self.get_parameter(name).value)

    def _bool_parameter(self, name: str) -> bool:
        """Return one bool-valued parameter."""
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return value.lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _handle_command(self, msg: ExcavationCommand):
        """Store the latest command and arm the matching transition."""
        now = monotonic()
        self._last_command = int(msg.command)

        if msg.command == ExcavationCommand.COMMAND_STOP:
            self._home_requested_at = None
            self._start_requested_at = None
            self._stop_requested_at = now
            return

        if msg.command == ExcavationCommand.COMMAND_HOME:
            self._home_switch = False
            self._motor_enabled = False
            self._motor_current_a = 0.0
            self._start_requested_at = None
            self._stop_requested_at = None
            self._home_requested_at = now
            return

        if msg.command == ExcavationCommand.COMMAND_START:
            self._stop_requested_at = None
            self._start_requested_at = now
            return

        if msg.command == ExcavationCommand.COMMAND_CLEAR_FAULT:
            self._fault_code = ExcavationTelemetry.FAULT_NONE
            return

        self.get_logger().warn(f"Unknown excavation command {msg.command}")

    def _apply_pending_transitions(self, now: float):
        """Advance the fake hardware state from pending commands."""
        if (
            self._home_requested_at is not None
            and now - self._home_requested_at >= self._float_parameter("home_delay_s")
        ):
            self._home_requested_at = None
            if not self._bool_parameter("hold_home_switch_false"):
                self._home_switch = True

        if (
            self._start_requested_at is not None
            and now - self._start_requested_at >= self._float_parameter("start_delay_s")
        ):
            self._start_requested_at = None
            self._motor_enabled = True
            if self._bool_parameter("force_overcurrent"):
                self._motor_current_a = self._float_parameter(
                    "overcurrent_motor_current_a"
                )
                self._fault_code = ExcavationTelemetry.FAULT_OVERCURRENT
            else:
                self._motor_current_a = self._float_parameter(
                    "nominal_motor_current_a"
                )

        if (
            self._stop_requested_at is not None
            and now - self._stop_requested_at >= self._float_parameter("stop_delay_s")
        ):
            self._stop_requested_at = None
            self._motor_enabled = False
            self._motor_current_a = 0.0

    def _build_telemetry(self) -> ExcavationTelemetry:
        """Build one telemetry sample from the current fake hardware state."""
        msg = ExcavationTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.estop_active = False
        msg.driver_fault = self._bool_parameter("force_driver_fault")
        msg.home_switch = (
            self._home_switch and not self._bool_parameter("hold_home_switch_false")
        )
        msg.motor_enabled = self._motor_enabled
        msg.motor_current_a = self._motor_current_a
        msg.fault_code = self._fault_code

        if msg.driver_fault:
            msg.fault_code = ExcavationTelemetry.FAULT_DRIVER
        elif self._bool_parameter("force_overcurrent") and msg.motor_enabled:
            msg.fault_code = ExcavationTelemetry.FAULT_OVERCURRENT
            msg.motor_current_a = self._float_parameter("overcurrent_motor_current_a")

        return msg

    def _publish_telemetry(self):
        """Publish one excavation telemetry update."""
        self._apply_pending_transitions(monotonic())
        self._telemetry_pub.publish(self._build_telemetry())


def main(args=None):
    """Run the excavation sim proxy node."""
    rclpy.init(args=args)
    node = ExcavationSimProxy()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
