# Copyright 2026 Leicester Lunabotics Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Convert cmd_vel to Sabertooth serial and publish drivetrain telemetry."""

from __future__ import annotations

import math
import time
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from lunabot_drivetrain import sabertooth_serial, teensy_serial
from lunabot_interfaces.msg import (
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
_LEGACY_SIMPLIFIED_PROTOCOLS = {"simplified", "legacy_simplified"}
_PACKETIZED_PROTOCOLS = {"packetized", "packet_serial"}
_TEENSY_PROTOCOLS = {"teensy", "teensy_line"}
_SUPPORTED_PROTOCOLS = (
    _LEGACY_SIMPLIFIED_PROTOCOLS | _PACKETIZED_PROTOCOLS | _TEENSY_PROTOCOLS
)


class DrivetrainBridge(Node):
    """Bridge between ROS cmd_vel and Sabertooth motor controllers."""

    def __init__(self) -> None:
        """Initialise serial, parameters, and ROS interfaces."""
        super().__init__("drivetrain_bridge")
        self._declare_parameters()
        self._validate_parameters()
        self._init_state()
        self._init_serial()
        self._init_ros()
        self.get_logger().info(
            f"Drivetrain bridge started — "
            f"port={self._serial_port}, "
            f"protocol={self._serial_protocol}, "
            f"dry_run={self._dry_run}, "
            f"cmd_vel_topic={self._cmd_vel_topic}, "
            f"addresses={self._addresses}"
        )

    def _declare_parameters(self) -> None:
        """Declare all node parameters with defaults."""
        self.declare_parameter("serial_port", "/dev/ttyTHS1")
        self.declare_parameter("baud_rate", 9600)
        self.declare_parameter("serial_protocol", "legacy_simplified")
        self.declare_parameter("reset_encoders_on_start", True)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_gated")
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
        self._serial_protocol = str(
            self.get_parameter("serial_protocol").value
        ).strip().lower()
        self._dry_run = bool(self.get_parameter("dry_run").value)
        self._reset_encoders_on_start = bool(
            self.get_parameter("reset_encoders_on_start").value
        )
        self._cmd_vel_topic = str(
            self.get_parameter("cmd_vel_topic").value
        ).strip()
        self._addresses = list(
            self.get_parameter("sabertooth_addresses").value
        )
        self._track_width = self.get_parameter("track_width_m").value
        self._wheel_radius = self.get_parameter("wheel_radius_m").value
        self._encoder_cpr = self.get_parameter(
            "encoder_counts_per_rev"
        ).value
        self._cmd_timeout = self.get_parameter(
            "command_timeout_s"
        ).value
        self._max_throttle = self.get_parameter("max_throttle").value
        self._stall_thresh = self.get_parameter(
            "stall_throttle_threshold"
        ).value
        self._stall_min_vel = self.get_parameter(
            "stall_min_velocity_rps"
        ).value
        self._stall_timeout = self.get_parameter(
            "stall_timeout_s"
        ).value
        ctrl_hz = self.get_parameter("control_loop_hz").value
        self._telem_hz = self.get_parameter("telemetry_publish_hz").value

        if not str(self._serial_port).strip():
            raise ValueError("serial_port must not be empty")
        if self._baud_rate <= 0:
            raise ValueError(f"baud_rate must be positive: {self._baud_rate}")
        if len(self._addresses) != 2:
            raise ValueError(
                f"Expected 2 Sabertooth addresses, got {len(self._addresses)}"
            )
        for address in self._addresses:
            if not 0 <= int(address) <= 255:
                raise ValueError(
                    f"Sabertooth address out of range: {address}"
                )
        if self._serial_protocol not in _SUPPORTED_PROTOCOLS:
            raise ValueError(
                "serial_protocol must be one of "
                f"{sorted(_SUPPORTED_PROTOCOLS)}, "
                f"got {self._serial_protocol!r}"
            )
        if not self._cmd_vel_topic:
            raise ValueError("cmd_vel_topic must not be empty")
        if self._track_width <= 0.0:
            raise ValueError(
                f"track_width_m must be positive: "
                f"{self._track_width}"
            )
        if self._wheel_radius <= 0.0:
            raise ValueError(
                f"wheel_radius_m must be positive: "
                f"{self._wheel_radius}"
            )
        if self._encoder_cpr <= 0:
            raise ValueError(
                f"encoder_counts_per_rev must be positive: "
                f"{self._encoder_cpr}"
            )
        if not 0.0 < self._max_throttle <= 1.0:
            raise ValueError(
                f"max_throttle must be in (0, 1]: "
                f"{self._max_throttle}"
            )
        if self._cmd_timeout <= 0.0:
            raise ValueError(
                f"command_timeout_s must be positive: {self._cmd_timeout}"
            )
        if self._stall_thresh < 0.0:
            raise ValueError(
                "stall_throttle_threshold must be zero or positive: "
                f"{self._stall_thresh}"
            )
        if self._stall_min_vel < 0.0:
            raise ValueError(
                "stall_min_velocity_rps must be zero or positive: "
                f"{self._stall_min_vel}"
            )
        if self._stall_timeout <= 0.0:
            raise ValueError(
                f"stall_timeout_s must be positive: {self._stall_timeout}"
            )
        if ctrl_hz <= 0.0:
            raise ValueError(f"control_loop_hz must be positive: {ctrl_hz}")
        if self._telem_hz <= 0.0:
            raise ValueError(
                f"telemetry_publish_hz must be positive: {self._telem_hz}"
            )

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
        self._last_teensy_ticks: Optional[list[int]] = None
        self._last_teensy_sample_time: Optional[float] = None
        self._controller_online = [False, False]
        self._stall_start_time: Optional[float] = None
        self._estop_clear_time: Optional[float] = None
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._serial: Any = None

    def _init_serial(self) -> None:
        """Open the UART serial port, failing closed unless dry-run is explicit."""
        try:
            import serial as pyserial
        except ImportError as exc:
            self._serial = None
            self._controller_online = [False, False]
            self._handle_serial_unavailable(exc)
            return

        try:
            self._serial = pyserial.Serial(
                self._serial_port,
                self._baud_rate,
                timeout=0.01,
            )
            self._controller_online = [True, True]
            if self._serial_protocol in _TEENSY_PROTOCOLS:
                self._initialise_teensy_serial()
                if self._state == DrivetrainStatus.STATE_FAULT:
                    return
            self._state = DrivetrainStatus.STATE_READY
            self.get_logger().info(f"Serial port {self._serial_port} opened")
        except (OSError, pyserial.SerialException) as exc:
            self._serial = None
            self._controller_online = [False, False]
            self._handle_serial_unavailable(exc)

    def _handle_serial_unavailable(self, exc: Exception) -> None:
        """Put the bridge in the configured no-serial state."""
        if self._dry_run:
            self.get_logger().warn(
                f"Serial port {self._serial_port} unavailable: {exc}. "
                "Explicit dry-run enabled, so motor output is disabled "
                "but ROS interfaces stay active."
            )
            self._state = DrivetrainStatus.STATE_READY
            return

        self.get_logger().error(
            f"Serial port {self._serial_port} unavailable: {exc}. "
            "dry_run is false, so drivetrain bridge is FAULTED and "
            "will not accept motion commands."
        )
        self._fault_code = DrivetrainStatus.FAULT_CONTROLLER_OFFLINE
        self._state = DrivetrainStatus.STATE_FAULT

    def _initialise_teensy_serial(self) -> None:
        """Flush startup chatter and put the Teensy firmware in READY."""
        if self._serial is None:
            return

        try:
            time.sleep(0.2)
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
            teensy_serial.send_release_estop(self._serial)
            teensy_serial.send_restart(self._serial)
            if self._reset_encoders_on_start:
                teensy_serial.send_zero_encoders(self._serial)
        except (AttributeError, OSError) as exc:
            self._mark_controller_offline(
                f"Teensy serial initialisation failed: {exc}"
            )

    def _mark_controller_offline(self, reason: str) -> None:
        """Record a controller IO failure and fail closed."""
        self.get_logger().error(reason)
        self._controller_online = [False, False]
        self._fault_code = DrivetrainStatus.FAULT_CONTROLLER_OFFLINE
        self._transition_to(DrivetrainStatus.STATE_FAULT)

    def _init_ros(self) -> None:
        """Set up publishers, subscribers, and timers."""
        self._cmd_vel_sub = self.create_subscription(
            Twist, self._cmd_vel_topic, self._cmd_vel_callback, 10
        )
        self._inhibit_sub = self.create_subscription(
            Bool,
            "/safety/motion_inhibit",
            self._inhibit_callback,
            _INHIBIT_QOS,
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
        self._odom_pub = self.create_publisher(
            Odometry, "/odom_wheels", 10
        )
        self._joint_pub = self.create_publisher(
            JointState, "/joint_states_wheels", 10
        )

        self._control_timer = self.create_timer(
            self._control_period, self._control_loop
        )
        telem_period = 1.0 / self._telem_hz
        self._telem_timer = self.create_timer(
            telem_period, self._publish_telemetry
        )

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
            self._send_estop()
            self._transition_to(DrivetrainStatus.STATE_ESTOP)

    def _control_loop(self) -> None:
        """Run one control cycle: convert cmd_vel to motor commands."""
        now = time.monotonic()
        self._read_teensy_feedback(now)

        if self._state == DrivetrainStatus.STATE_FAULT:
            self._send_stop()
            return
        if self._estop_active or self._motion_inhibited:
            self._send_stop()
            if self._estop_active:
                self._transition_to(DrivetrainStatus.STATE_ESTOP)
            return

        estop_cleared = (
            self._state == DrivetrainStatus.STATE_ESTOP
            and not self._estop_active
        )
        if estop_cleared:
            if self._estop_clear_time is None:
                self._estop_clear_time = now
                self.get_logger().info(
                    "E-stop cleared — waiting 2s for "
                    "Sabertooth re-init"
                )
            elif (now - self._estop_clear_time) >= 2.0:
                self._release_estop()
                self._estop_clear_time = None
                self._fault_code = DrivetrainStatus.FAULT_NONE
                self._transition_to(DrivetrainStatus.STATE_READY)
                self.get_logger().info(
                    "Sabertooth re-init complete — READY"
                )
            self._send_stop()
            return

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
            self._check_encoder_stall(now, left, right)
        else:
            self._transition_to(DrivetrainStatus.STATE_READY)
            self._stall_start_time = None

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

    def _check_encoder_stall(
        self, now: float, left: float, right: float
    ) -> None:
        """Detect encoder stall: throttle applied but no wheel motion."""
        max_throttle = max(abs(left), abs(right))
        if max_throttle < self._stall_thresh:
            self._stall_start_time = None
            return

        max_vel = max(
            abs(v) for v in self._wheel_velocity_rps
        )
        if max_vel >= self._stall_min_vel:
            self._stall_start_time = None
            return

        if self._stall_start_time is None:
            self._stall_start_time = now
            return

        elapsed = now - self._stall_start_time
        if elapsed >= self._stall_timeout:
            self.get_logger().error(
                f"Encoder stall detected: throttle={max_throttle:.2f}"
                f" but velocity={max_vel:.3f} rps for {elapsed:.1f}s"
            )
            self._fault_code = (
                DrivetrainStatus.FAULT_ENCODER_STALL
            )
            self._transition_to(DrivetrainStatus.STATE_FAULT)

    def _send_wheel_throttles(self, left: float, right: float) -> None:
        """Send per-side throttles to both Sabertooth controllers."""
        if self._serial is None:
            return
        try:
            if self._serial_protocol in _TEENSY_PROTOCOLS:
                teensy_serial.send_throttle(self._serial, left, right)
                return
            if self._serial_protocol in _LEGACY_SIMPLIFIED_PROTOCOLS:
                sabertooth_serial.send_simplified_throttle(
                    self._serial, left, right
                )
                return
            sabertooth_serial.send_throttle(
                self._serial, self._addresses[0], left, right
            )
            sabertooth_serial.send_throttle(
                self._serial, self._addresses[1], left, right
            )
        except OSError as exc:
            self._mark_controller_offline(f"Serial write failed: {exc}")

    def _send_stop(self) -> None:
        """Command zero throttle to all motors."""
        if self._serial is None:
            return
        try:
            if self._serial_protocol in _TEENSY_PROTOCOLS:
                teensy_serial.send_stop(self._serial)
                return
            if self._serial_protocol in _LEGACY_SIMPLIFIED_PROTOCOLS:
                sabertooth_serial.send_simplified_stop(self._serial)
                return
            for addr in self._addresses:
                sabertooth_serial.send_stop(self._serial, addr)
        except OSError as exc:
            self._mark_controller_offline(f"Serial stop failed: {exc}")

    def _send_estop(self) -> None:
        """Send a software e-stop latch when the backend supports one."""
        if self._serial is None or self._serial_protocol not in _TEENSY_PROTOCOLS:
            self._send_stop()
            return
        try:
            teensy_serial.send_estop(self._serial)
        except OSError as exc:
            self._mark_controller_offline(f"Serial e-stop failed: {exc}")

    def _release_estop(self) -> None:
        """Release and restart a Teensy software e-stop latch."""
        if self._serial is None or self._serial_protocol not in _TEENSY_PROTOCOLS:
            return
        try:
            teensy_serial.send_release_estop(self._serial)
            teensy_serial.send_restart(self._serial)
        except OSError as exc:
            self._mark_controller_offline(
                f"Serial e-stop release failed: {exc}"
            )

    def _read_teensy_feedback(self, now: float) -> None:
        """Consume available Teensy telemetry without blocking control output."""
        if self._serial is None or self._serial_protocol not in _TEENSY_PROTOCOLS:
            return

        while True:
            try:
                if getattr(self._serial, "in_waiting", 0) <= 0:
                    return
                raw_line = self._serial.readline()
            except OSError as exc:
                self._mark_controller_offline(f"Serial read failed: {exc}")
                return
            if not raw_line:
                return

            try:
                telemetry = teensy_serial.parse_telemetry_line(raw_line)
            except ValueError as exc:
                self.get_logger().warn(
                    f"Ignoring malformed Teensy telemetry: {exc}"
                )
                continue
            if telemetry is None:
                continue
            self._apply_teensy_telemetry(telemetry, now)

    def _apply_teensy_telemetry(
        self, telemetry: teensy_serial.TeensyTelemetry, now: float
    ) -> None:
        """Update encoder ticks and wheel velocities from Teensy feedback."""
        ticks = telemetry.encoder_ticks
        if (
            self._last_teensy_ticks is not None
            and self._last_teensy_sample_time is not None
        ):
            dt = now - self._last_teensy_sample_time
            if dt > 0.0:
                self._wheel_velocity_rps = [
                    (ticks[i] - self._last_teensy_ticks[i])
                    / self._encoder_cpr
                    / dt
                    for i in range(4)
                ]

        self._encoder_ticks = ticks
        self._last_teensy_ticks = list(ticks)
        self._last_teensy_sample_time = now
        self._controller_online = [True, True]

    def _transition_to(self, new_state: int) -> None:
        """Update the state machine, only logging on actual transitions."""
        if new_state != self._state:
            self._state = new_state

    def _publish_telemetry(self) -> None:
        """Publish telemetry and status at the configured rate."""
        self._read_teensy_feedback(time.monotonic())
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
        js.velocity = [
            float(v * 2.0 * math.pi)
            for v in self._wheel_velocity_rps
        ]
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
            except OSError as exc:
                self.get_logger().warn(f"Serial close failed: {exc}")
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
        if rclpy.ok():
            rclpy.shutdown()
