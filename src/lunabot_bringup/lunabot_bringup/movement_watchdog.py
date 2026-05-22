"""Advisory mission movement watchdog diagnostics."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import SetBool

from lunabot_interfaces import msg as lunabot_msgs

LEVEL_OK = DiagnosticStatus.OK
LEVEL_WARN = DiagnosticStatus.WARN
LEVEL_ERROR = DiagnosticStatus.ERROR
LEVEL_STALE = DiagnosticStatus.STALE

DrivetrainStatus: Any = lunabot_msgs.__dict__["DrivetrainStatus"]
DrivetrainTelemetry: Any = lunabot_msgs.__dict__["DrivetrainTelemetry"]


def _key_value(key: str, value: object) -> KeyValue:
    item = KeyValue()
    item.key = key
    item.value = str(value)
    return item


def _yaw_from_quaternion(orientation: Any) -> float:
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_delta(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def _distance_2d(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


@dataclass
class EvidenceSource:
    """Movement evidence accumulated for one physical feedback source."""

    name: str
    priority: int
    last_received_s: float | None = None
    last_position: tuple[float, float] | None = None
    last_yaw: float | None = None
    last_encoder_ticks: tuple[int, ...] | None = None
    motion_window_started_s: float | None = None
    accumulated_translation_m: float = 0.0
    accumulated_yaw_rad: float = 0.0
    accumulated_encoder_ticks: int = 0
    max_wheel_velocity_rps: float = 0.0

    def age_s(self, now_s: float) -> float | None:
        """Return source age in seconds, or None if it has never published."""
        if self.last_received_s is None:
            return None
        return max(0.0, now_s - self.last_received_s)

    def fresh(self, now_s: float, stale_timeout_s: float) -> bool:
        """Return whether this source is fresh enough to use."""
        age_s = self.age_s(now_s)
        return age_s is not None and age_s <= stale_timeout_s

    def reset_motion_window(self, now_s: float) -> None:
        """Reset short-window accumulation after stillness or confirmation."""
        self.motion_window_started_s = now_s
        self.accumulated_translation_m = 0.0
        self.accumulated_yaw_rad = 0.0
        self.accumulated_encoder_ticks = 0
        self.max_wheel_velocity_rps = 0.0


@dataclass
class MovementWatchdogConfig:
    """Configurable thresholds for rulebook movement evidence."""

    timeout_s: float = 300.0
    warn_s: float = 240.0
    stale_timeout_s: float = 2.5
    confirmation_window_s: float = 1.0
    min_translation_m: float = 0.05
    min_yaw_rad: float = 0.05
    min_wheel_velocity_rps: float = 0.05
    min_encoder_ticks: int = 4
    max_odom_jump_m: float = 2.0
    max_odom_jump_rad: float = math.pi


@dataclass
class MovementWatchdog:
    """Track confirmed physical movement for the 5-minute mission anomaly."""

    config: MovementWatchdogConfig = field(default_factory=MovementWatchdogConfig)
    odom_topics: tuple[str, ...] = ("/odometry/local", "/odom", "/odom_wheels")
    telemetry_topic: str = "/drivetrain/telemetry"
    armed: bool = False
    ever_moved: bool = False
    armed_at_s: float | None = None
    last_confirmed_motion_s: float | None = None
    last_motion_source: str = ""
    drivetrain_state: int | str = "unknown"
    fault_code: int | str = "unknown"
    sources: dict[str, EvidenceSource] = field(default_factory=dict)

    def __post_init__(self) -> None:
        for priority, topic in enumerate(self.odom_topics):
            self.sources[topic] = EvidenceSource(topic, priority)
        self.sources[self.telemetry_topic] = EvidenceSource(
            self.telemetry_topic, len(self.odom_topics)
        )

    def set_armed(self, armed: bool, now_s: float) -> None:
        """Arm/reset or disarm the watchdog."""
        self.armed = armed
        self.ever_moved = False
        self.last_confirmed_motion_s = None
        self.last_motion_source = ""
        self.armed_at_s = now_s if armed else None
        for source in self.sources.values():
            source.reset_motion_window(now_s)

    def update_drivetrain_status(self, msg: Any) -> None:
        """Store command-derived drivetrain state only as diagnostic context."""
        self.drivetrain_state = msg.state
        self.fault_code = msg.fault_code

    def update_odom(self, topic: str, msg: Odometry, now_s: float) -> None:
        """Update odometry-derived physical movement evidence."""
        source = self.sources[topic]
        position = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        yaw = _yaw_from_quaternion(msg.pose.pose.orientation)

        if source.last_position is None or source.last_yaw is None:
            source.last_position = position
            source.last_yaw = yaw
            source.last_received_s = now_s
            source.reset_motion_window(now_s)
            return

        translation_m = _distance_2d(position, source.last_position)
        yaw_delta_rad = _angle_delta(yaw, source.last_yaw)
        source.last_position = position
        source.last_yaw = yaw
        source.last_received_s = now_s

        if (
            translation_m > self.config.max_odom_jump_m
            or yaw_delta_rad > self.config.max_odom_jump_rad
        ):
            source.reset_motion_window(now_s)
            return

        if translation_m <= 0.0 and yaw_delta_rad <= 0.0:
            source.reset_motion_window(now_s)
            return

        self._accumulate_source_motion(
            source,
            now_s=now_s,
            translation_m=translation_m,
            yaw_delta_rad=yaw_delta_rad,
        )

    def update_telemetry(self, msg: Any, now_s: float) -> None:
        """Update drivetrain telemetry fallback movement evidence."""
        source = self.sources[self.telemetry_topic]
        wheel_velocity = tuple(float(v) for v in msg.wheel_velocity_rps)
        encoder_ticks = tuple(int(v) for v in msg.encoder_ticks)
        max_velocity = max((abs(v) for v in wheel_velocity), default=0.0)

        if source.last_encoder_ticks is None:
            source.last_encoder_ticks = encoder_ticks
            source.last_received_s = now_s
            source.reset_motion_window(now_s)
            source.max_wheel_velocity_rps = max_velocity
            return

        encoder_delta = sum(
            abs(current - previous)
            for current, previous in zip(encoder_ticks, source.last_encoder_ticks)
        )
        source.last_encoder_ticks = encoder_ticks
        source.last_received_s = now_s

        if max_velocity <= 0.0 and encoder_delta <= 0:
            source.reset_motion_window(now_s)
            return

        self._accumulate_source_motion(
            source,
            now_s=now_s,
            max_wheel_velocity_rps=max_velocity,
            encoder_delta=encoder_delta,
        )

    def _accumulate_source_motion(
        self,
        source: EvidenceSource,
        *,
        now_s: float,
        translation_m: float = 0.0,
        yaw_delta_rad: float = 0.0,
        max_wheel_velocity_rps: float = 0.0,
        encoder_delta: int = 0,
    ) -> None:
        if source.motion_window_started_s is None:
            source.reset_motion_window(now_s)
        source.accumulated_translation_m += translation_m
        source.accumulated_yaw_rad += yaw_delta_rad
        source.accumulated_encoder_ticks += encoder_delta
        source.max_wheel_velocity_rps = max(
            source.max_wheel_velocity_rps, max_wheel_velocity_rps
        )

        window_started_s = (
            source.motion_window_started_s
            if source.motion_window_started_s is not None
            else now_s
        )
        window_age_s = now_s - window_started_s
        if window_age_s < self.config.confirmation_window_s:
            return

        odom_confirmed = (
            source.accumulated_translation_m >= self.config.min_translation_m
            or source.accumulated_yaw_rad >= self.config.min_yaw_rad
        )
        telemetry_confirmed = (
            source.max_wheel_velocity_rps >= self.config.min_wheel_velocity_rps
            and source.accumulated_encoder_ticks >= self.config.min_encoder_ticks
        )
        if odom_confirmed or telemetry_confirmed:
            self.ever_moved = True
            self.last_confirmed_motion_s = now_s
            self.last_motion_source = source.name
            source.reset_motion_window(now_s)

    def selected_source(self, now_s: float) -> EvidenceSource | None:
        """Return the highest-priority fresh source, if any."""
        fresh_sources = [
            source
            for source in self.sources.values()
            if source.fresh(now_s, self.config.stale_timeout_s)
        ]
        if not fresh_sources:
            return None
        return min(fresh_sources, key=lambda source: source.priority)

    def diagnostic(self, now_s: float) -> DiagnosticStatus:
        """Build the advisory diagnostic status."""
        selected = self.selected_source(now_s)
        elapsed_s = 0.0
        if self.armed and self.armed_at_s is not None:
            basis_s = self.last_confirmed_motion_s or self.armed_at_s
            elapsed_s = max(0.0, now_s - basis_s)

        values = {
            "armed": self.armed,
            "ever_moved": self.ever_moved,
            "seconds_since_confirmed_motion": f"{elapsed_s:.2f}",
            "timeout_s": f"{self.config.timeout_s:.2f}",
            "warn_s": f"{self.config.warn_s:.2f}",
            "last_motion_source": self.last_motion_source or "none",
            "selected_source": selected.name if selected is not None else "none",
            "drivetrain_state": self.drivetrain_state,
            "fault_code": self.fault_code,
        }
        for source in self.sources.values():
            age_s = source.age_s(now_s)
            values[f"{source.name}.age_s"] = (
                f"{age_s:.2f}" if age_s is not None else "never"
            )
        if selected is not None:
            values.update(
                {
                    "odom_delta_m": f"{selected.accumulated_translation_m:.3f}",
                    "yaw_delta_rad": f"{selected.accumulated_yaw_rad:.3f}",
                    "max_wheel_velocity_rps": (
                        f"{selected.max_wheel_velocity_rps:.3f}"
                    ),
                }
            )

        status = DiagnosticStatus()
        status.name = "mission/movement_watchdog"
        status.hardware_id = "innex1_rover"
        status.values = [_key_value(key, value) for key, value in values.items()]

        if not self.armed:
            status.level = LEVEL_OK
            status.message = "Movement watchdog disarmed"
            return status
        if selected is None:
            status.level = LEVEL_STALE
            status.message = "No fresh physical movement evidence"
            return status
        if elapsed_s >= self.config.timeout_s:
            status.level = LEVEL_ERROR
            status.message = "No confirmed rover movement for timeout"
            return status
        if elapsed_s >= self.config.warn_s:
            status.level = LEVEL_WARN
            status.message = "No confirmed rover movement near timeout"
            return status
        status.level = LEVEL_OK
        status.message = (
            "Movement confirmed recently"
            if self.ever_moved
            else "Awaiting confirmed rover movement"
        )
        return status


class MovementWatchdogNode(Node):
    """ROS wrapper for the advisory movement watchdog."""

    def __init__(self) -> None:
        super().__init__("movement_watchdog")
        self.declare_parameter(
            "odom_topics", ["/odometry/local", "/odom", "/odom_wheels"]
        )
        self.declare_parameter("telemetry_topic", "/drivetrain/telemetry")
        self.declare_parameter("timeout_s", 300.0)
        self.declare_parameter("warn_s", 240.0)
        self.declare_parameter("stale_timeout_s", 2.5)
        self.declare_parameter("confirmation_window_s", 1.0)
        self.declare_parameter("min_translation_m", 0.05)
        self.declare_parameter("min_yaw_rad", 0.05)
        self.declare_parameter("min_wheel_velocity_rps", 0.05)
        self.declare_parameter("min_encoder_ticks", 4)
        self.declare_parameter("publish_hz", 1.0)
        self.declare_parameter("auto_arm", False)

        odom_topics = tuple(
            str(topic) for topic in self.get_parameter("odom_topics").value
        )
        telemetry_topic = str(self.get_parameter("telemetry_topic").value)
        config = MovementWatchdogConfig(
            timeout_s=float(self.get_parameter("timeout_s").value),
            warn_s=float(self.get_parameter("warn_s").value),
            stale_timeout_s=float(self.get_parameter("stale_timeout_s").value),
            confirmation_window_s=float(
                self.get_parameter("confirmation_window_s").value
            ),
            min_translation_m=float(self.get_parameter("min_translation_m").value),
            min_yaw_rad=float(self.get_parameter("min_yaw_rad").value),
            min_wheel_velocity_rps=float(
                self.get_parameter("min_wheel_velocity_rps").value
            ),
            min_encoder_ticks=int(self.get_parameter("min_encoder_ticks").value),
        )
        publish_hz = float(self.get_parameter("publish_hz").value)
        if publish_hz <= 0.0:
            raise ValueError("publish_hz must be positive")
        if config.timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        if config.warn_s < 0.0 or config.warn_s > config.timeout_s:
            raise ValueError("warn_s must be between zero and timeout_s")
        if config.stale_timeout_s <= 0.0:
            raise ValueError("stale_timeout_s must be positive")
        if config.confirmation_window_s < 0.0:
            raise ValueError("confirmation_window_s must be non-negative")

        self._watchdog = MovementWatchdog(
            config=config,
            odom_topics=odom_topics,
            telemetry_topic=telemetry_topic,
        )
        if bool(self.get_parameter("auto_arm").value):
            self._watchdog.set_armed(True, time.monotonic())

        for topic in odom_topics:
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, topic=topic: self._watchdog.update_odom(
                    topic, msg, time.monotonic()
                ),
                10,
            )
        self.create_subscription(
            DrivetrainTelemetry,
            telemetry_topic,
            lambda msg: self._watchdog.update_telemetry(msg, time.monotonic()),
            10,
        )
        self.create_subscription(
            DrivetrainStatus,
            "/drivetrain/status",
            self._watchdog.update_drivetrain_status,
            10,
        )
        self.create_service(
            SetBool,
            "/mission/movement_watchdog/armed",
            self._set_armed,
        )
        self._publisher = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(1.0 / publish_hz, self._publish)
        self.get_logger().info(
            "Movement watchdog publishing advisory diagnostics on /diagnostics"
        )

    def _set_armed(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self._watchdog.set_armed(bool(request.data), time.monotonic())
        response.success = True
        response.message = (
            "Movement watchdog armed" if request.data else "Movement watchdog disarmed"
        )
        return response

    def _publish(self) -> None:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [self._watchdog.diagnostic(time.monotonic())]
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MovementWatchdogNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
