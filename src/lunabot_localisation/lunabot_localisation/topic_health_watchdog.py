"""Monitor critical topic health: rate, staleness, jitter."""

from dataclasses import dataclass
from typing import Dict

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from std_msgs.msg import Bool


@dataclass
class TopicStats:
    """Accumulate rate / staleness / jitter stats for one topic."""

    name: str
    min_hz: float
    msg_count: int = 0
    last_stamp: float = 0.0
    last_receive_time: float = 0.0
    max_gap_sec: float = 0.0
    total_gaps: float = 0.0
    gap_count: int = 0
    healthy: bool = True
    last_header_stamp: float = 0.0


class TopicHealthWatchdog(Node):
    """Periodically check topic rates and log warnings for unhealthy sources."""

    MONITORED_TOPICS = {
        "/odom": {"type": Odometry, "min_hz": 10.0, "critical": True},
        "/imu/data_raw": {"type": Imu, "min_hz": 50.0, "critical": True},
        "/camera_front/image_sync": {"type": Image, "min_hz": 2.0, "critical": True},
        "/camera_front/depth_image_sync": {"type": Image, "min_hz": 2.0, "critical": True},
        "/camera_front/camera_info_sync": {
            "type": CameraInfo,
            "min_hz": 2.0,
            "critical": True,
        },
        # Needed for costmaps, but not part of VO contract health.
        "/camera_front/points_nav": {
            "type": PointCloud2,
            "min_hz": 3.0,
            "critical": False,
        },
        # Monitor VO rate for diagnostics only (avoids circular dependency).
        "/visual_odometry": {"type": Odometry, "min_hz": 1.0, "critical": False},
    }

    def __init__(self) -> None:
        """Initialise subscriptions and health tracking state."""
        super().__init__("topic_health_watchdog")
        self.declare_parameter("check_period_sec", 5.0)
        self.declare_parameter("startup_grace_sec", 10.0)
        self.declare_parameter("max_staleness_sec", 2.5)
        self.declare_parameter("enforce_stamp_deltas", False)
        self.declare_parameter("max_image_depth_delta_sec", 0.25)
        self.declare_parameter("max_image_info_delta_sec", 0.15)

        self.check_period = float(self.get_parameter("check_period_sec").value)
        self.startup_grace_sec = float(self.get_parameter("startup_grace_sec").value)
        self.max_staleness_sec = float(self.get_parameter("max_staleness_sec").value)
        self.enforce_stamp_deltas = bool(
            self.get_parameter("enforce_stamp_deltas").value
        )
        self.max_image_depth_delta_sec = float(
            self.get_parameter("max_image_depth_delta_sec").value
        )
        self.max_image_info_delta_sec = float(
            self.get_parameter("max_image_info_delta_sec").value
        )
        self.start_time_sec = self.get_clock().now().nanoseconds / 1e9

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.stats: Dict[str, TopicStats] = {}
        self.health_pub = self.create_publisher(Bool, "/diagnostics/topics_healthy", 10)
        self.last_image_stamp = 0.0
        self.last_depth_stamp = 0.0
        self.last_info_stamp = 0.0
        self.max_image_depth_delta = 0.0
        self.max_image_info_delta = 0.0

        for topic_name, cfg in self.MONITORED_TOPICS.items():
            self.stats[topic_name] = TopicStats(
                name=topic_name, min_hz=cfg["min_hz"]
            )
            # Create a closure-based callback
            self.create_subscription(
                cfg["type"],
                topic_name,
                self._make_callback(topic_name),
                sensor_qos,
            )

        self.create_timer(self.check_period, self._check_health)
        self.get_logger().info(
            f"Topic health watchdog started, monitoring {len(self.MONITORED_TOPICS)} topics"
        )

    def _make_callback(self, topic_name: str):
        """Return a callback that records receipt of a message on topic_name."""

        def callback(msg) -> None:
            now = self.get_clock().now().nanoseconds / 1e9
            stats = self.stats[topic_name]
            stats.msg_count += 1

            if stats.last_receive_time > 0.0:
                gap = now - stats.last_receive_time
                stats.total_gaps += gap
                stats.gap_count += 1
                if gap > stats.max_gap_sec:
                    stats.max_gap_sec = gap

            stats.last_receive_time = now

            # Extract header stamp if available
            if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
                stamp = Time.from_msg(msg.header.stamp)
                stats.last_stamp = stamp.nanoseconds / 1e9
                stats.last_header_stamp = stats.last_stamp
                self._update_pair_deltas(topic_name, stats.last_stamp)

        return callback

    def _update_pair_deltas(self, topic_name: str, stamp: float) -> None:
        """Track synchronization deltas between RGB, depth, and camera_info."""
        if topic_name == "/camera_front/image_sync":
            self.last_image_stamp = stamp
            if self.last_depth_stamp > 0.0:
                self.max_image_depth_delta = max(
                    self.max_image_depth_delta,
                    abs(self.last_image_stamp - self.last_depth_stamp),
                )
            if self.last_info_stamp > 0.0:
                self.max_image_info_delta = max(
                    self.max_image_info_delta,
                    abs(self.last_image_stamp - self.last_info_stamp),
                )
        elif topic_name == "/camera_front/depth_image_sync":
            self.last_depth_stamp = stamp
            if self.last_image_stamp > 0.0:
                self.max_image_depth_delta = max(
                    self.max_image_depth_delta,
                    abs(self.last_image_stamp - self.last_depth_stamp),
                )
        elif topic_name == "/camera_front/camera_info_sync":
            self.last_info_stamp = stamp
            if self.last_image_stamp > 0.0:
                self.max_image_info_delta = max(
                    self.max_image_info_delta,
                    abs(self.last_image_stamp - self.last_info_stamp),
                )

    def _check_health(self) -> None:
        """Log a summary of each monitored topic's health."""
        all_healthy = True
        now = self.get_clock().now().nanoseconds / 1e9
        in_startup_grace = (now - self.start_time_sec) < self.startup_grace_sec

        for topic_name, stats in self.stats.items():
            is_critical = bool(self.MONITORED_TOPICS[topic_name]["critical"])
            enforce_critical = is_critical and not in_startup_grace
            if stats.msg_count == 0:
                self.get_logger().warn(
                    f"[WATCHDOG] {topic_name}: NO MESSAGES received in last "
                    f"{self.check_period:.0f}s"
                )
                stats.healthy = False
                if enforce_critical:
                    all_healthy = False
                continue

            # Compute average rate over the check period
            avg_gap = (
                stats.total_gaps / stats.gap_count if stats.gap_count > 0 else 0.0
            )
            effective_hz = 1.0 / avg_gap if avg_gap > 0 else 0.0
            staleness = now - stats.last_receive_time if stats.last_receive_time > 0 else 999.0

            healthy = effective_hz >= stats.min_hz and staleness < self.max_staleness_sec

            if not healthy:
                self.get_logger().warn(
                    f"[WATCHDOG] {topic_name}: hz={effective_hz:.1f} "
                    f"(need≥{stats.min_hz:.0f}) "
                    f"max_gap={stats.max_gap_sec:.3f}s "
                    f"stale={staleness:.2f}s "
                    f"count={stats.msg_count}"
                )
                if enforce_critical:
                    all_healthy = False
            else:
                self.get_logger().debug(
                    f"[WATCHDOG] {topic_name}: OK hz={effective_hz:.1f} "
                    f"max_gap={stats.max_gap_sec:.3f}s"
                )

            stats.healthy = healthy

            # Reset counters for next window
            stats.msg_count = 0
            stats.max_gap_sec = 0.0
            stats.total_gaps = 0.0
            stats.gap_count = 0

        if self.enforce_stamp_deltas and not in_startup_grace:
            if self.max_image_depth_delta > self.max_image_depth_delta_sec:
                self.get_logger().warn(
                    "[WATCHDOG] rgb-depth stamp delta too large: %.3fs (max %.3fs)"
                    % (self.max_image_depth_delta, self.max_image_depth_delta_sec)
                )
                all_healthy = False
            if self.max_image_info_delta > self.max_image_info_delta_sec:
                self.get_logger().warn(
                    "[WATCHDOG] rgb-camera_info stamp delta too large: %.3fs (max %.3fs)"
                    % (self.max_image_info_delta, self.max_image_info_delta_sec)
                )
                all_healthy = False

        self.max_image_depth_delta = 0.0
        self.max_image_info_delta = 0.0

        health_msg = Bool()
        health_msg.data = all_healthy
        self.health_pub.publish(health_msg)


def main(args=None) -> None:
    """Run the topic health watchdog."""
    rclpy.init(args=args)
    node = TopicHealthWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
