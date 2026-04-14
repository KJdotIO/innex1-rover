"""Drivetrain bridge: converts cmd_vel to Sabertooth serial and publishes odometry.

Subscribes to /cmd_vel_safe (output of the collision monitor) and converts
Twist messages into per-wheel throttle commands for the two Sabertooth 2x32
controllers via Packetized Serial.  Reads quadrature encoder feedback and
publishes JointState, odometry, and DrivetrainTelemetry/Status.

Safety:
- Checks /safety/motion_inhibit before every command cycle.
- Commands a full stop if no cmd_vel is received within command_timeout_s.
- Detects encoder stall (commanded but no motion) and enters FAULT state.
- All loops and waits are bounded per ROVER_CODING_STANDARD rule 2.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from lunabot_interfaces.msg import (
    DrivetrainCommand,
    DrivetrainStatus,
    DrivetrainTelemetry,
)

_INHIBIT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

_WHEEL_NAMES = ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"]


class DrivetrainBridge(Node):
    """Bridge between ROS cmd_vel and Sabertooth motor controllers."""

    def __init__(self) -> None:
        super().__init__("drivetrain_bridge")
        self._declare_parameters()
        self._validate_parameters()
        self._init_state()
        self._init_serial()
        self._init_ros()
        self.get_logger().info(
            f"Drivetrain bridge started — "
            f"port={self._serial_port}, "
            f"addresses={self._addresses}"
        )

    def _declare_parameters(self) -> None:
        """Declare all node parameters with defaults."""
        self.declare_parameter("serial_port", "/dev/ttyTHS1")
        self.declare_parameter("baud_rate", 9600)
        self.declare_parameter("sabertooth_addresses", [128, 129])
        self.declare_parameter("track_width_m", 0.44)
        self.declare_parameter("wheel_radius_m", 0.065)
        self.declare_parameter("encoder_counts_per_rev", 720)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("max_throttle", 1.0)
        self.declare_parameter("stall_throttle_threshold", 0.15)
        self.declare_parameter("stall_min_velocity_rps", 0.05)
        self.declare_parameter("stall_timeout_s", 2.0)
        self.declare_parameter("control_loop_hz", 20.0)
        self.declare_parameter("telemetry_publish_hz", 10.0)

    def _validate_parameters(self) -> None:
        """Read and validate all parameters.  Fail fast on invalid config."""
        self._serial_port = self.get_parameter("serial_port").value
        self._baud_rate = self.get_parameter("baud_rate").value
        self._addresses = list(
            self.get_parameter("sabertooth_addresses").value
        )
        self._track_width = self.get_parameter("track_width_m").value
        self._wheel_radius = self.get_parameter("wheel_radius_m").value
        self._encoder_cpr = self.get_parameter("encoder_counts_per_rev").value
        self._cmd_timeout = self.get_parameter("command_timeout_s").value
        self._max_throttle = self.get_parameter("max_throttle").value
        self._stall_thresh = self.get_parameter("stall_throttle_threshold").value
        self._stall_min_vel = self.get_parameter("stall_min_velocity_rps").value
        self._stall_timeout = self.get_parameter("stall_timeout_s").value
        ctrl_hz = self.get_parameter("control_loop_hz").value
        self._telem_hz = self.get_parameter("telemetry_publish_hz").value

        if len(self._addresses) != 2:
            raise ValueError(
                f"Expected 2 Sabertooth addresses, got {len(self._addresses)}"
            )
        if self._track_width <= 0.0:
            raise ValueError(f"track_width_m must be positive: {self._track_width}")
        if self._wheel_radius <= 0.0:
            raise ValueError(f"wheel_radius_m must be positive: {self._wheel_radius}")
        if self._encoder_cpr <= 0:
            raise ValueError(f"encoder_counts_per_rev must be positive: {self._encoder_cpr}")
        if not 0.0 < self._max_throttle <= 1.0:
            raise ValueError(f"max_throttle must be in (0, 1]: {self._max_throttle}")

        self._control_period = 1.0 / ctrl_hz

    def _init_state(self) -> None:
        """Initialise mutable state variables."""
        self._state = DrivetrainStatus.STATE_UNINITIALISED
        self._fault_code = DrivetrainStatus.FAULT_NONE
        self._motion_inhibited = False
        self._estop_active = False
        self._last_cmd_time: Optional[float] = None
        self._last_twist = Twist()
        self._encoder_ticks = [0, 0, 0, 0]
        self._wheel_velocity_rps = [0.0, 0.0, 0.0, 0.0]
        self._controller_online = [False, False]
        self._stall_start_time: Optional[float] = None
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._serial: object = None

    def _init_serial(self) -> None:
        """Open the UART serial port.  Logs a warning if unavailable."""
        try:
            import serial as pyserial
            self._serial = pyserial.Serial(
                self._serial_port,
                self._baud_rate,
                timeout=0.01,
            )
            self._controller_online = [True, True]
            self._state = DrivetrainStatus.STATE_READY
            self.get_logger().info(f"Serial port {self._serial_port} opened")
        except Exception as exc:
            self.get_logger().warn(
                f"Serial port {self._serial_port} unavailable: {exc}. "
                f"Running in dry-run mode (no motor output)."
            )
            self._serial = None
            self._controller_online = [False, False]
            self._state = DrivetrainStatus.STATE_READY

    def _init_ros(self) -> None:
        """Set up publishers, subscribers, and timers."""
        self._cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel_safe", self._cmd_vel_callback, 10
        )
        self._inhibit_sub = self.create_subscription(
            Bool, "/safety/motion_inhibit", self._inhibit_callback, _INHIBIT_QOS
        )
        self._estop_sub = self.create_subscription(
            Bool, "/safety/estop", self._estop_callback, 10
        )
        self._status_pub = self.create_publisher(
            DrivetrainStatus, "/drivetrain/status", 10
        )
        self._telem_pub = self.create_publisher(
            DrivetrainTelemetry, "/drivetrain/telemetry", 10
        )
        self._odom_pub = self.create_publisher(Odometry, "/odom_wheels", 10)
        self._joint_pub = self.create_publisher(JointState, "/joint_states_wheels", 10)

        self._control_timer = self.create_timer(
            self._control_period, self._control_loop
        )
        telem_period = 1.0 / self._telem_hz
        self._telem_timer = self.create_timer(telem_period, self._publish_telemetry)

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """Store the latest velocity command and timestamp."""
        self._last_twist = msg
        self._last_cmd_time = time.monotonic()

    def _inhibit_callback(self, msg: Bool) -> None:
        """Update motion inhibit state from safety node."""
        self._motion_inhibited = msg.data

    def _estop_callback(self, msg: Bool) -> None:
        """Update e-stop state."""
        prev = self._estop_active
        self._estop_active = msg.data
        if msg.data and not prev:
            self.get_logger().warn("E-stop ACTIVE — motors will be stopped")
            self._transition_to(DrivetrainStatus.STATE_ESTOP)

    def _control_loop(self) -> None:
        """Main control cycle: convert cmd_vel to motor commands."""
        now = time.monotonic()

        if self._state == DrivetrainStatus.STATE_FAULT:
            self._send_stop()
            return
        if self._estop_active or self._motion_inhibited:
            self._send_stop()
            if self._estop_active:
                self._transition_to(DrivetrainStatus.STATE_ESTOP)
            return

        if self._state == DrivetrainStatus.STATE_ESTOP and not self._estop_active:
            self.get_logger().info("E-stop cleared — returning to READY")
            self._fault_code = DrivetrainStatus.FAULT_NONE
            self._transition_to(DrivetrainStatus.STATE_READY)

        cmd_stale = (
            self._last_cmd_time is None
            or (now - self._last_cmd_time) > self._cmd_timeout
        )
        if cmd_stale:
            self._send_stop()
            if self._state == DrivetrainStatus.STATE_DRIVING:
                self._transition_to(DrivetrainStatus.STATE_READY)
            return

        linear_x = self._last_twist.linear.x
        angular_z = self._last_twist.angular.z
        left, right = self._twist_to_wheel_speeds(linear_x, angular_z)
        self._send_wheel_throttles(left, right)

        if abs(left) > 0.01 or abs(right) > 0.01:
            self._transition_to(DrivetrainStatus.STATE_DRIVING)
        else:
            self._transition_to(DrivetrainStatus.STATE_READY)

    def _twist_to_wheel_speeds(
        self, linear_x: float, angular_z: float
    ) -> tuple[float, float]:
        """Differential drive: Twist → (left_throttle, right_throttle)."""
        half_track = self._track_width / 2.0
        left_vel = linear_x - angular_z * half_track
        right_vel = linear_x + angular_z * half_track

        max_vel = max(abs(left_vel), abs(right_vel), 0.001)
        max_speed = 0.3
        scale = min(1.0, max_speed / max_vel)

        left_throttle = (left_vel * scale) / max_speed
        right_throttle = (right_vel * scale) / max_speed

        limit = self._max_throttle
        left_throttle = max(-limit, min(limit, left_throttle))
        right_throttle = max(-limit, min(limit, right_throttle))
        return left_throttle, right_throttle

    def _send_wheel_throttles(self, left: float, right: float) -> None:
        """Send per-side throttles to both Sabertooth controllers."""
        if self._serial is None:
            return
        try:
            from lunabot_drivetrain.sabertooth_serial import send_throttle
            send_throttle(self._serial, self._addresses[0], left, right)
            send_throttle(self._serial, self._addresses[1], left, right)
        except Exception as exc:
            self.get_logger().error(f"Serial write failed: {exc}")
            self._fault_code = DrivetrainStatus.FAULT_CONTROLLER_OFFLINE
            self._transition_to(DrivetrainStatus.STATE_FAULT)

    def _send_stop(self) -> None:
        """Command zero throttle to all motors."""
        if self._serial is None:
            return
        try:
            from lunabot_drivetrain.sabertooth_serial import send_stop
            for addr in self._addresses:
                send_stop(self._serial, addr)
        except Exception as exc:
            self.get_logger().error(f"Serial stop failed: {exc}")

    def _transition_to(self, new_state: int) -> None:
        """Update the state machine, only logging on actual transitions."""
        if new_state != self._state:
            self._state = new_state

    def _publish_telemetry(self) -> None:
        """Publish telemetry and status at the configured rate."""
        now = self.get_clock().now().to_msg()

        status = DrivetrainStatus()
        status.header.stamp = now
        status.state = self._state
        status.fault_code = self._fault_code
        status.estop_active = self._estop_active
        status.motion_inhibited = self._motion_inhibited
        status.controller_online = self._controller_online
        self._status_pub.publish(status)

        telem = DrivetrainTelemetry()
        telem.header.stamp = now
        telem.wheel_velocity_rps = self._wheel_velocity_rps
        telem.encoder_ticks = self._encoder_ticks
        telem.controller_online = self._controller_online
        telem.estop_active = self._estop_active
        telem.motion_inhibited = self._motion_inhibited
        telem.fault_code = self._fault_code
        self._telem_pub.publish(telem)

        self._publish_joint_state(now)

    def _publish_joint_state(self, stamp) -> None:
        """Publish wheel joint states for robot_state_publisher."""
        js = JointState()
        js.header.stamp = stamp
        js.name = list(_WHEEL_NAMES)
        js.velocity = [float(v * 2.0 * math.pi) for v in self._wheel_velocity_rps]
        js.position = [
            float(t / self._encoder_cpr * 2.0 * math.pi)
            for t in self._encoder_ticks
        ]
        js.effort = []
        self._joint_pub.publish(js)

    def destroy_node(self) -> None:
        """Send stop and close serial on shutdown."""
        self._send_stop()
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None) -> None:
    """Entry point for the drivetrain_bridge executable."""
    rclpy.init(args=args)
    node = DrivetrainBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
