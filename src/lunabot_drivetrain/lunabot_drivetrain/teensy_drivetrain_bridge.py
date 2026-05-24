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

"""ROS bridge for the Teensy 4.1 motor IO controller."""

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

from lunabot_drivetrain.teensy_protocol import (
    DriveCommand,
    MessageType,
    ProtocolError,
    unpack_drivetrain_telemetry,
    unpack_status,
)
from lunabot_drivetrain.teensy_serial import TeensySerialClient
from lunabot_interfaces.msg import DrivetrainStatus, DrivetrainTelemetry

_INHIBIT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

_WHEEL_NAMES = ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"]


class TeensyDrivetrainBridge(Node):
    """Bridge ROS drivetrain contracts to the Teensy USB serial endpoint."""

    def __init__(self) -> None:
        super().__init__("teensy_drivetrain_bridge")
        self._declare_parameters()
        self._validate_parameters()
        self._init_state()
        self._init_serial()
        self._init_ros()
        self.get_logger().info(
            f"Teensy drivetrain bridge started — port={self._serial_port}, "
            f"dry_run={self._dry_run}, cmd_vel_topic={self._cmd_vel_topic}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("serial_port", "/dev/teensy_motor_io")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_gated")
        self.declare_parameter("track_width_m", 0.44)
        self.declare_parameter("wheel_radius_m", 0.065)
        self.declare_parameter("encoder_counts_per_rev", 720)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("teensy_timeout_s", 0.35)
        self.declare_parameter("max_throttle", 1.0)
        self.declare_parameter("control_loop_hz", 20.0)
        self.declare_parameter("telemetry_publish_hz", 10.0)

    def _validate_parameters(self) -> None:
        self._serial_port = str(self.get_parameter("serial_port").value).strip()
        self._baud_rate = int(self.get_parameter("baud_rate").value)
        self._dry_run = bool(self.get_parameter("dry_run").value)
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value).strip()
        self._track_width = float(self.get_parameter("track_width_m").value)
        self._wheel_radius = float(self.get_parameter("wheel_radius_m").value)
        self._encoder_cpr = int(self.get_parameter("encoder_counts_per_rev").value)
        self._cmd_timeout = float(self.get_parameter("command_timeout_s").value)
        self._teensy_timeout = float(self.get_parameter("teensy_timeout_s").value)
        self._max_throttle = float(self.get_parameter("max_throttle").value)
        ctrl_hz = float(self.get_parameter("control_loop_hz").value)
        self._telem_hz = float(self.get_parameter("telemetry_publish_hz").value)

        if not self._serial_port:
            raise ValueError("serial_port must not be empty")
        if self._baud_rate <= 0:
            raise ValueError("baud_rate must be positive")
        if not self._cmd_vel_topic:
            raise ValueError("cmd_vel_topic must not be empty")
        if self._track_width <= 0.0:
            raise ValueError("track_width_m must be positive")
        if self._wheel_radius <= 0.0:
            raise ValueError("wheel_radius_m must be positive")
        if self._encoder_cpr <= 0:
            raise ValueError("encoder_counts_per_rev must be positive")
        if self._cmd_timeout <= 0.0:
            raise ValueError("command_timeout_s must be positive")
        if self._teensy_timeout <= 0.0:
            raise ValueError("teensy_timeout_s must be positive")
        if not 0.0 < self._max_throttle <= 1.0:
            raise ValueError("max_throttle must be in (0, 1]")
        if ctrl_hz <= 0.0:
            raise ValueError("control_loop_hz must be positive")
        if self._telem_hz <= 0.0:
            raise ValueError("telemetry_publish_hz must be positive")
        self._control_period = 1.0 / ctrl_hz

    def _init_state(self) -> None:
        self._state = DrivetrainStatus.STATE_UNINITIALISED
        self._fault_code = DrivetrainStatus.FAULT_NONE
        self._teensy_state = DrivetrainStatus.STATE_UNINITIALISED
        self._teensy_fault_code = DrivetrainStatus.FAULT_NONE
        self._teensy_status_fault_code = DrivetrainStatus.FAULT_NONE
        self._teensy_telemetry_fault_code = DrivetrainStatus.FAULT_NONE
        self._ros_estop_active = False
        self._ros_motion_inhibited = False
        self._teensy_estop_active = False
        self._teensy_motion_inhibited = False
        self._motion_inhibited = False
        self._estop_active = False
        self._controller_online = [False, False]
        self._controller_offline_latched = False
        self._startup_time = time.monotonic()
        self._last_cmd_time: Optional[float] = None
        self._last_twist = Twist()
        self._encoder_ticks = [0, 0, 0, 0]
        self._wheel_velocity_rps = [0.0, 0.0, 0.0, 0.0]
        self._last_odom_time: Optional[float] = None
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._client: TeensySerialClient | None = None
        self._serial: Any = None

    def _init_serial(self) -> None:
        try:
            import serial as pyserial
        except ImportError as exc:
            self._handle_serial_unavailable(exc)
            return

        try:
            self._serial = pyserial.Serial(
                self._serial_port,
                self._baud_rate,
                timeout=0,
                write_timeout=0.1,
            )
        except (OSError, pyserial.SerialException) as exc:
            self._handle_serial_unavailable(exc)
            return

        self._client = TeensySerialClient(self._serial)
        self._state = DrivetrainStatus.STATE_READY
        self.get_logger().info(f"Teensy serial port {self._serial_port} opened")

    def _handle_serial_unavailable(self, exc: Exception) -> None:
        if self._dry_run:
            self.get_logger().warn(
                f"Teensy serial port {self._serial_port} unavailable: {exc}. "
                "Explicit dry-run enabled; no motor output will be sent."
            )
            self._state = DrivetrainStatus.STATE_READY
            return
        self.get_logger().error(
            f"Teensy serial port {self._serial_port} unavailable: {exc}. "
            "Bridge is FAULTED and will not accept motion commands."
        )
        self._fault_code = DrivetrainStatus.FAULT_CONTROLLER_OFFLINE
        self._state = DrivetrainStatus.STATE_FAULT

    def _init_ros(self) -> None:
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
        self._odom_pub = self.create_publisher(Odometry, "/odom_wheels", 10)
        self._joint_pub = self.create_publisher(
            JointState, "/joint_states_wheels", 10
        )

        self._control_timer = self.create_timer(
            self._control_period, self._control_loop
        )
        self._telem_timer = self.create_timer(
            1.0 / self._telem_hz, self._publish_telemetry
        )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._last_twist = msg
        self._last_cmd_time = time.monotonic()

    def _inhibit_callback(self, msg: Bool) -> None:
        self._ros_motion_inhibited = msg.data
        self._sync_safety_state()

    def _estop_callback(self, msg: Bool) -> None:
        self._ros_estop_active = msg.data
        self._sync_safety_state()
        if msg.data:
            self._state = DrivetrainStatus.STATE_ESTOP
            self._fault_code = DrivetrainStatus.FAULT_ESTOP

    def _control_loop(self) -> None:
        now = time.monotonic()
        self._read_teensy_frames(now)

        if self._client is None:
            return

        try:
            self._client.send_heartbeat()
            self._client.send_drive_command(self._build_drive_command(now))
        except OSError as exc:
            self._mark_controller_offline(f"Teensy serial write failed: {exc}")
            return

        if self._link_stale(now):
            self._mark_controller_offline("No fresh Teensy telemetry")

    def _build_drive_command(self, now: float) -> DriveCommand:
        stale = (
            self._last_cmd_time is None
            or (now - self._last_cmd_time) > self._cmd_timeout
        )
        blocked = self._estop_active or self._motion_inhibited or stale
        blocked = blocked or self._teensy_fault_code != DrivetrainStatus.FAULT_NONE
        blocked = blocked or self._teensy_state == DrivetrainStatus.STATE_FAULT
        if blocked:
            if (
                stale
                and self._state == DrivetrainStatus.STATE_DRIVING
                and self._teensy_fault_code == DrivetrainStatus.FAULT_NONE
            ):
                self._state = DrivetrainStatus.STATE_READY
            return DriveCommand(
                left=0.0,
                right=0.0,
                enabled=False,
                estop_active=self._estop_active,
                motion_inhibited=self._motion_inhibited,
            )

        left, right = self._twist_to_wheel_speeds(
            self._last_twist.linear.x,
            self._last_twist.angular.z,
        )
        enabled = abs(left) > 0.01 or abs(right) > 0.01
        self._state = (
            DrivetrainStatus.STATE_DRIVING
            if enabled
            else DrivetrainStatus.STATE_READY
        )
        self._fault_code = DrivetrainStatus.FAULT_NONE
        return DriveCommand(
            left=left,
            right=right,
            enabled=enabled,
            estop_active=False,
            motion_inhibited=False,
        )

    def _read_teensy_frames(self, now: float) -> None:
        if self._client is None:
            return
        try:
            frames = self._client.read_frames()
        except OSError as exc:
            self._mark_controller_offline(f"Teensy serial read failed: {exc}")
            return

        for frame in frames:
            try:
                if frame.msg_type == MessageType.STATUS:
                    self._apply_status(frame.body)
                elif frame.msg_type == MessageType.DRIVETRAIN_TELEMETRY:
                    self._apply_telemetry(frame.body)
            except ProtocolError as exc:
                self.get_logger().warn(f"Ignoring bad Teensy payload: {exc}")

        if self._client.last_rx_time is not None and not self._link_stale(now):
            if self._controller_offline_latched and not self._estop_active:
                self._controller_offline_latched = False
                self._fault_code = DrivetrainStatus.FAULT_NONE
                self._state = DrivetrainStatus.STATE_READY

    def _apply_status(self, body: bytes) -> None:
        status = unpack_status(body)
        self._teensy_state = status.state
        self._teensy_status_fault_code = status.fault_code
        self._apply_teensy_health()
        self._teensy_estop_active = status.estop_active
        self._teensy_motion_inhibited = status.motion_inhibited
        self._sync_safety_state()
        self._controller_online = list(status.controller_online)

    def _apply_telemetry(self, body: bytes) -> None:
        telemetry = unpack_drivetrain_telemetry(body)
        self._encoder_ticks = list(telemetry.encoder_ticks)
        self._wheel_velocity_rps = list(telemetry.wheel_velocity_rps)
        self._teensy_telemetry_fault_code = telemetry.fault_code
        self._apply_teensy_health()
        self._teensy_estop_active = telemetry.estop_active
        self._teensy_motion_inhibited = telemetry.motion_inhibited
        self._sync_safety_state()
        self._controller_online = list(telemetry.controller_online)

    def _apply_teensy_health(self) -> None:
        self._teensy_fault_code = max(
            self._teensy_status_fault_code,
            self._teensy_telemetry_fault_code,
        )

        if self._teensy_state == DrivetrainStatus.STATE_FAULT:
            self._state = DrivetrainStatus.STATE_FAULT
        elif self._teensy_state in {
            DrivetrainStatus.STATE_READY,
            DrivetrainStatus.STATE_DRIVING,
            DrivetrainStatus.STATE_ESTOP,
        }:
            self._state = self._teensy_state

        self._fault_code = self._teensy_fault_code
        if self._teensy_fault_code != DrivetrainStatus.FAULT_NONE:
            self._state = DrivetrainStatus.STATE_FAULT

    def _sync_safety_state(self) -> None:
        self._estop_active = self._ros_estop_active or self._teensy_estop_active
        self._motion_inhibited = (
            self._ros_motion_inhibited or self._teensy_motion_inhibited
        )
        if self._estop_active:
            self._state = DrivetrainStatus.STATE_ESTOP
            self._fault_code = DrivetrainStatus.FAULT_ESTOP

    def _link_stale(self, now: float) -> bool:
        if self._dry_run:
            return False
        if self._client is None or self._client.last_rx_time is None:
            return (now - self._startup_time) > self._teensy_timeout
        return (now - self._client.last_rx_time) > self._teensy_timeout

    def _mark_controller_offline(self, reason: str) -> None:
        if self._state != DrivetrainStatus.STATE_FAULT:
            self.get_logger().error(reason)
        self._controller_online = [False, False]
        self._controller_offline_latched = True
        self._fault_code = DrivetrainStatus.FAULT_CONTROLLER_OFFLINE
        self._state = DrivetrainStatus.STATE_FAULT

    def _twist_to_wheel_speeds(
        self, linear_x: float, angular_z: float
    ) -> tuple[float, float]:
        half_track = self._track_width / 2.0
        left_vel = linear_x - angular_z * half_track
        right_vel = linear_x + angular_z * half_track
        max_vel = max(abs(left_vel), abs(right_vel), 0.001)
        max_speed = 0.3
        scale = min(1.0, max_speed / max_vel)
        limit = self._max_throttle
        left = max(-limit, min(limit, (left_vel * scale) / max_speed))
        right = max(-limit, min(limit, (right_vel * scale) / max_speed))
        return left, right

    def _publish_telemetry(self) -> None:
        stamp = self.get_clock().now().to_msg()
        status = DrivetrainStatus()
        status.header.stamp = stamp
        status.state = self._state
        status.fault_code = self._fault_code
        status.estop_active = self._estop_active
        status.motion_inhibited = self._motion_inhibited
        status.controller_online = self._controller_online
        self._status_pub.publish(status)

        telem = DrivetrainTelemetry()
        telem.header.stamp = stamp
        telem.wheel_velocity_rps = self._wheel_velocity_rps
        telem.encoder_ticks = self._encoder_ticks
        telem.controller_online = self._controller_online
        telem.estop_active = self._estop_active
        telem.motion_inhibited = self._motion_inhibited
        telem.fault_code = self._fault_code
        self._telem_pub.publish(telem)
        self._publish_odometry(stamp)
        self._publish_joint_state(stamp)

    def _publish_odometry(self, stamp) -> None:
        now = time.monotonic()
        if self._last_odom_time is None:
            dt = 0.0
        else:
            dt = max(0.0, now - self._last_odom_time)
        self._last_odom_time = now

        left_rps = (self._wheel_velocity_rps[0] + self._wheel_velocity_rps[2]) / 2.0
        right_rps = (self._wheel_velocity_rps[1] + self._wheel_velocity_rps[3]) / 2.0
        wheel_circumference = 2.0 * math.pi * self._wheel_radius
        left_mps = left_rps * wheel_circumference
        right_mps = right_rps * wheel_circumference
        linear_x = (left_mps + right_mps) / 2.0
        angular_z = (right_mps - left_mps) / self._track_width

        if dt > 0.0:
            self._odom_x += linear_x * math.cos(self._odom_yaw) * dt
            self._odom_y += linear_x * math.sin(self._odom_yaw) * dt
            self._odom_yaw += angular_z * dt

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = float(self._odom_x)
        odom.pose.pose.position.y = float(self._odom_y)
        odom.pose.pose.orientation.z = math.sin(self._odom_yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._odom_yaw / 2.0)
        odom.twist.twist.linear.x = float(linear_x)
        odom.twist.twist.angular.z = float(angular_z)
        self._odom_pub.publish(odom)

    def _publish_joint_state(self, stamp) -> None:
        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = list(_WHEEL_NAMES)
        joint_state.velocity = [
            float(v * 2.0 * math.pi) for v in self._wheel_velocity_rps
        ]
        joint_state.position = [
            float(t / self._encoder_cpr * 2.0 * math.pi)
            for t in self._encoder_ticks
        ]
        joint_state.effort = []
        self._joint_pub.publish(joint_state)

    def destroy_node(self) -> None:
        if self._client is not None:
            try:
                self._client.send_drive_command(
                    DriveCommand(
                        left=0.0,
                        right=0.0,
                        enabled=False,
                        estop_active=True,
                        motion_inhibited=True,
                    )
                )
            except OSError:
                pass
        if self._serial is not None:
            try:
                self._serial.close()
            except OSError as exc:
                self.get_logger().warn(f"Teensy serial close failed: {exc}")
        super().destroy_node()


def main(args=None) -> None:
    """Entry point for the teensy_drivetrain_bridge executable."""
    rclpy.init(args=args)
    node = TeensyDrivetrainBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
