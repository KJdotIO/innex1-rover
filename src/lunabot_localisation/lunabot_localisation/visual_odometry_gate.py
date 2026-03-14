"""Gate visual odometry before it is fused into the EKFs."""

from dataclasses import dataclass
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import OdomInfo
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


@dataclass
class HealthSnapshot:
    """Compact snapshot of the latest odometry health decision."""

    healthy: bool
    reason: str
    inliers: int
    matches: int
    features: int
    position_variance: float
    yaw_variance: float
    moving: bool


class VisualOdometryGate(Node):
    """Republish visual odometry only when OdomInfo says tracking is healthy."""

    def __init__(self) -> None:
        """Initialise subscriptions, gating thresholds, and health publishers."""
        super().__init__("visual_odometry_gate")

        self.declare_parameter("odom_topic", "/visual_odometry")
        self.declare_parameter("odom_info_topic", "/odom_info")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("gated_odom_topic", "/visual_odometry/gated")
        self.declare_parameter("health_topic", "/visual_odometry/healthy")
        self.declare_parameter("sensor_health_topic", "/diagnostics/topics_healthy")
        self.declare_parameter("require_sensor_health", True)
        self.declare_parameter("min_inliers", 15)
        self.declare_parameter("min_matches", 30)
        self.declare_parameter("max_position_variance", 0.5)
        self.declare_parameter("max_yaw_variance", 0.5)
        self.declare_parameter("odom_info_timeout_sec", 5.0)
        self.declare_parameter("sensor_health_timeout_sec", 8.0)
        self.declare_parameter("transition_log_interval_sec", 3.0)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.min_inliers = int(self.get_parameter("min_inliers").value)
        self.min_matches = int(self.get_parameter("min_matches").value)
        self.max_position_variance = float(
            self.get_parameter("max_position_variance").value
        )
        self.max_yaw_variance = float(self.get_parameter("max_yaw_variance").value)
        self.odom_info_timeout = Duration(
            seconds=float(self.get_parameter("odom_info_timeout_sec").value)
        )
        self.sensor_health_timeout = Duration(
            seconds=float(self.get_parameter("sensor_health_timeout_sec").value)
        )
        self.transition_log_interval = Duration(
            seconds=float(self.get_parameter("transition_log_interval_sec").value)
        )
        self.require_sensor_health = bool(
            self.get_parameter("require_sensor_health").value
        )

        self.latest_info: Optional[OdomInfo] = None
        self.latest_info_receive_time = None  # clock time when last info arrived
        self.latest_health: Optional[HealthSnapshot] = None
        self.last_transition_log_time = None
        # Start optimistic and enforce freshness with sensor_health_timeout.
        # This prevents startup deadlock before watchdog's first publication.
        self.sensor_contract_healthy = True
        self.latest_sensor_health_time = self.get_clock().now()
        self.is_moving = False

        # Counters for summary logging
        self._passed = 0
        self._blocked = 0

        self.odom_pub = self.create_publisher(
            Odometry,
            self.get_parameter("gated_odom_topic").value,
            sensor_qos,
        )
        self.health_pub = self.create_publisher(
            Bool,
            self.get_parameter("health_topic").value,
            sensor_qos,
        )

        self.create_subscription(
            OdomInfo,
            self.get_parameter("odom_info_topic").value,
            self.on_odom_info,
            sensor_qos,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("sensor_health_topic").value,
            self.on_sensor_health,
            sensor_qos,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.on_odom,
            sensor_qos,
        )
        self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            self.on_cmd_vel,
            sensor_qos,
        )
        self.create_timer(0.5, self.on_timer)
        self.create_timer(10.0, self._log_summary)

    def on_cmd_vel(self, msg: Twist) -> None:
        """Track whether the rover is currently being commanded to move."""
        self.is_moving = any(
            abs(value) > 1e-3
            for value in (
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z,
            )
        )

    def on_odom_info(self, msg: OdomInfo) -> None:
        """Update the current health state from RTAB-Map odometry telemetry."""
        self.latest_info = msg
        self.latest_info_receive_time = self.get_clock().now()
        snapshot = self.evaluate_health(msg)
        self.publish_health(snapshot.healthy)

        if self.should_log_transition(snapshot):
            motion_state = "moving" if snapshot.moving else "stationary"
            self.get_logger().warn(
                "VO %s: reason=%s inliers=%d matches=%d features=%d "
                "pos_var=%.3f yaw_var=%.3f cmd=%s"
                % (
                    "HEALTHY" if snapshot.healthy else "UNHEALTHY",
                    snapshot.reason,
                    snapshot.inliers,
                    snapshot.matches,
                    snapshot.features,
                    snapshot.position_variance,
                    snapshot.yaw_variance,
                    motion_state,
                )
            )
            self.last_transition_log_time = self.get_clock().now()

        self.latest_health = snapshot

    def on_sensor_health(self, msg: Bool) -> None:
        """Track the upstream sensor data-path health contract."""
        self.sensor_contract_healthy = bool(msg.data)
        self.latest_sensor_health_time = self.get_clock().now()

    def on_odom(self, msg: Odometry) -> None:
        """Republish visual odometry only while the health state is good."""
        if self.latest_health and self.latest_health.healthy:
            self.odom_pub.publish(msg)
            self._passed += 1
        else:
            self._blocked += 1

    def on_timer(self) -> None:
        """Mark odometry unhealthy if fresh OdomInfo stops arriving."""
        if self.latest_info_receive_time is None:
            return

        now = self.get_clock().now()
        if now - self.latest_info_receive_time > self.odom_info_timeout:
            snapshot = HealthSnapshot(
                healthy=False,
                reason="odom_info_timeout",
                inliers=self.latest_info.inliers if self.latest_info else 0,
                matches=self.latest_info.matches if self.latest_info else 0,
                features=self.latest_info.features if self.latest_info else 0,
                position_variance=(
                    self.latest_info.covariance[0] if self.latest_info else 999.0
                ),
                yaw_variance=(
                    self.latest_info.covariance[35] if self.latest_info else 999.0
                ),
                moving=self.is_moving,
            )
            if self.should_log_transition(snapshot):
                self.get_logger().warn(
                    "VO UNHEALTHY: reason=%s last_inliers=%d last_matches=%d cmd=%s"
                    % (
                        snapshot.reason,
                        snapshot.inliers,
                        snapshot.matches,
                        "moving" if snapshot.moving else "stationary",
                    )
                )
                self.last_transition_log_time = now
            self.latest_health = snapshot
            self.publish_health(False)
            return

        if (
            self.require_sensor_health
            and self.latest_sensor_health_time is not None
            and now - self.latest_sensor_health_time > self.sensor_health_timeout
        ):
            snapshot = HealthSnapshot(
                healthy=False,
                reason="sensor_health_timeout",
                inliers=self.latest_info.inliers if self.latest_info else 0,
                matches=self.latest_info.matches if self.latest_info else 0,
                features=self.latest_info.features if self.latest_info else 0,
                position_variance=(
                    self.latest_info.covariance[0] if self.latest_info else 999.0
                ),
                yaw_variance=(
                    self.latest_info.covariance[35] if self.latest_info else 999.0
                ),
                moving=self.is_moving,
            )
            if self.should_log_transition(snapshot):
                self.get_logger().warn(
                    "VO UNHEALTHY: reason=%s last_inliers=%d last_matches=%d cmd=%s"
                    % (
                        snapshot.reason,
                        snapshot.inliers,
                        snapshot.matches,
                        "moving" if snapshot.moving else "stationary",
                    )
                )
                self.last_transition_log_time = now
            self.latest_health = snapshot
            self.publish_health(False)

    def _log_summary(self) -> None:
        """Periodically log pass/block counts."""
        total = self._passed + self._blocked
        if total > 0:
            pct = 100.0 * self._passed / total
            self.get_logger().info(
                f"VO gate: passed={self._passed} blocked={self._blocked} "
                f"({pct:.0f}% pass rate)"
            )
        self._passed = 0
        self._blocked = 0

    def evaluate_health(self, msg: OdomInfo) -> HealthSnapshot:
        """Classify the odometry output using inliers, matches, and covariance."""
        position_variance = msg.covariance[0] if len(msg.covariance) > 0 else 999.0
        yaw_variance = msg.covariance[35] if len(msg.covariance) > 35 else 999.0

        if self.require_sensor_health and not self.sensor_contract_healthy:
            reason = "sensor_contract_unhealthy"
            healthy = False
        elif msg.lost:
            reason = "lost"
            healthy = False
        elif msg.inliers < self.min_inliers:
            reason = "low_inliers"
            healthy = False
        elif msg.matches < self.min_matches:
            reason = "low_matches"
            healthy = False
        elif position_variance > self.max_position_variance:
            reason = "high_position_variance"
            healthy = False
        elif yaw_variance > self.max_yaw_variance:
            reason = "high_yaw_variance"
            healthy = False
        else:
            reason = "tracking_ok"
            healthy = True

        return HealthSnapshot(
            healthy=healthy,
            reason=reason,
            inliers=msg.inliers,
            matches=msg.matches,
            features=msg.features,
            position_variance=position_variance,
            yaw_variance=yaw_variance,
            moving=self.is_moving,
        )

    def should_log_transition(self, snapshot: HealthSnapshot) -> bool:
        """Log state changes and throttle repeated messages."""
        if self.latest_health is None:
            return True
        if (
            self.latest_health.healthy != snapshot.healthy
            or self.latest_health.reason != snapshot.reason
        ):
            return True
        if self.last_transition_log_time is None:
            return True
        return (
            self.get_clock().now() - self.last_transition_log_time
            >= self.transition_log_interval
        )

    def publish_health(self, healthy: bool) -> None:
        """Publish the current boolean health state for downstream consumers."""
        msg = Bool()
        msg.data = healthy
        self.health_pub.publish(msg)


def main(args=None) -> None:
    """Run the visual odometry gate node."""
    rclpy.init(args=args)
    node = VisualOdometryGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        # Seen during shutdown under heavy pub/sub churn in sim.
        if "Unable to convert call argument" not in str(exc):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
