"""Bounded autonomous AprilTag acquisition for start-zone localisation."""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from lunabot_interfaces.msg import LocalisationStartZoneStatus
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from robot_localization.srv import SetPose
from tf2_ros import Buffer, TransformListener
import tf2_ros

from lunabot_localisation.start_zone_logic import angle_delta
from lunabot_localisation.start_zone_logic import PoseSample
from lunabot_localisation.start_zone_logic import StableLockTracker


STATE_IDLE = "idle"
STATE_SEARCHING = "searching"
STATE_CANDIDATE_LOCK = "candidate_lock"
STATE_READY = "ready"
STATE_FAILED = "failed"

REASON_NONE = "NONE"
REASON_SEARCHING = "LOCALISATION_SEARCHING"
REASON_UNSTABLE = "LOCALISATION_TAG_UNSTABLE"
REASON_STALE = "LOCALISATION_TAG_STALE"
REASON_TIMEOUT = "LOCALISATION_TAG_TIMEOUT"
REASON_TF_MISSING = "LOCALISATION_TF_MISSING"
REASON_SET_POSE_FAILED = "LOCALISATION_SET_POSE_FAILED"
REASON_READY = "LOCALISATION_READY"


class StartZoneLocaliser(Node):
    """Acquire a stable start-zone tag lock before travel autonomy begins."""

    def __init__(self) -> None:
        super().__init__("start_zone_localiser")

        self.declare_parameter("status_topic", "/localisation/start_zone_status")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("odometry_topic", "/odometry/local")
        self.declare_parameter("tag_pose_topic", "/tag_pose")
        self.declare_parameter("set_pose_service", "/ekf_filter_node_map/set_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("tag_frame", "tag36h11:0")
        self.declare_parameter("camera_frame", "camera_front_optical_frame")
        self.declare_parameter("target_frame", "base_footprint")
        self.declare_parameter("search_direction", "left")
        self.declare_parameter("angular_speed_rad_s", 0.25)
        self.declare_parameter("max_search_duration_s", 30.0)
        self.declare_parameter("max_search_yaw_rad", 2.0 * math.pi)
        self.declare_parameter("candidate_alignment_angular_speed_rad_s", 0.08)
        self.declare_parameter("candidate_settle_duration_s", 0.5)
        self.declare_parameter("stable_window_duration_s", 1.0)
        self.declare_parameter("stable_window_min_samples", 5)
        self.declare_parameter("max_sample_gap_s", 0.25)
        self.declare_parameter("max_lock_translation_spread_m", 0.15)
        self.declare_parameter("max_lock_yaw_spread_rad", 0.20)
        self.declare_parameter("tf_missing_timeout_s", 5.0)
        self.declare_parameter("set_pose_service_wait_s", 0.1)
        self.declare_parameter("set_pose_timeout_s", 5.0)

        self.status_topic = self.get_parameter("status_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.odometry_topic = self.get_parameter("odometry_topic").value
        self.tag_pose_topic = self.get_parameter("tag_pose_topic").value
        self.set_pose_service = self.get_parameter("set_pose_service").value
        self.map_frame = self.get_parameter("map_frame").value
        self.tag_frame = self.get_parameter("tag_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.search_direction = self.get_parameter("search_direction").value
        self.angular_speed_rad_s = float(
            self.get_parameter("angular_speed_rad_s").value
        )
        self.max_search_duration_s = float(
            self.get_parameter("max_search_duration_s").value
        )
        self.max_search_yaw_rad = float(self.get_parameter("max_search_yaw_rad").value)
        self.candidate_alignment_angular_speed_rad_s = float(
            self.get_parameter("candidate_alignment_angular_speed_rad_s").value
        )
        self.candidate_settle_duration_s = float(
            self.get_parameter("candidate_settle_duration_s").value
        )
        self.stable_window_duration_s = float(
            self.get_parameter("stable_window_duration_s").value
        )
        self.stable_window_min_samples = int(
            self.get_parameter("stable_window_min_samples").value
        )
        self.max_sample_gap_s = float(self.get_parameter("max_sample_gap_s").value)
        self.max_lock_translation_spread_m = float(
            self.get_parameter("max_lock_translation_spread_m").value
        )
        self.max_lock_yaw_spread_rad = float(
            self.get_parameter("max_lock_yaw_spread_rad").value
        )
        self.tf_missing_timeout_s = float(
            self.get_parameter("tf_missing_timeout_s").value
        )
        self.set_pose_service_wait_s = float(
            self.get_parameter("set_pose_service_wait_s").value
        )
        self.set_pose_timeout_s = float(
            self.get_parameter("set_pose_timeout_s").value
        )

        self.search_sign = 1.0 if self.search_direction.lower() == "left" else -1.0
        self.max_sample_gap_ns = int(self.max_sample_gap_s * 1e9)
        self.stable_window_ns = int(self.stable_window_duration_s * 1e9)

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.status_pub = self.create_publisher(
            LocalisationStartZoneStatus,
            self.status_topic,
            status_qos,
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.tag_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.tag_pose_topic,
            self._on_tag_pose,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odometry_topic,
            self._on_odometry,
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.set_pose_client = self.create_client(SetPose, self.set_pose_service)

        self.lock_tracker = StableLockTracker()
        self.latest_seed_pose: PoseWithCovarianceStamped | None = None
        self.last_odom_yaw: float | None = None
        self.accumulated_search_yaw = 0.0
        self.search_started_ns: int | None = None
        self.ready_since_ns: int | None = None
        self.candidate_entered_ns: int | None = None
        self.pending_set_pose_future = None
        self.set_pose_attempt_started_ns: int | None = None
        self.discarded_set_pose_future = None
        self.discarded_set_pose_started_ns: int | None = None
        self.last_set_pose_service_check_ns: int | None = None
        self.search_motion_active = False
        self.last_timer_now_ns: int | None = None

        self.state = STATE_SEARCHING
        self.reason_code = REASON_SEARCHING
        self.reason_detail = "Searching for the start-zone AprilTag."

        self.timer = self.create_timer(0.1, self._on_timer)

    def _on_odometry(self, msg: Odometry) -> None:
        """Track planar yaw so the search remains bounded in angle as well as time."""
        yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)
        if self.state == STATE_SEARCHING and self.last_odom_yaw is not None:
            self.accumulated_search_yaw += abs(angle_delta(yaw, self.last_odom_yaw))
        self.last_odom_yaw = yaw

    def _on_tag_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """Store accepted tag poses from the bridge node."""
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= 0 or stamp_ns > (now_ns + self.max_sample_gap_ns):
            return
        yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)
        self.lock_tracker.add_sample(
            PoseSample(
                stamp_ns=stamp_ns,
                x=msg.pose.pose.position.x,
                y=msg.pose.pose.position.y,
                yaw=yaw,
            )
        )
        self.latest_seed_pose = msg

    @staticmethod
    def _yaw_from_quaternion(quaternion) -> float:
        """Return planar yaw from a quaternion."""
        siny_cosp = 2.0 * (
            quaternion.w * quaternion.z + quaternion.x * quaternion.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            quaternion.y * quaternion.y + quaternion.z * quaternion.z
        )
        return math.atan2(siny_cosp, cosy_cosp)

    def _required_tf_available(self) -> bool:
        """Return whether the static TF chain required for tag fusion exists."""
        try:
            self.tf_buffer.lookup_transform(
                self.map_frame,
                self.tag_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
            self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.target_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return False
        return True

    def _publish_cmd(self, angular_z: float) -> None:
        """Publish a velocity command."""
        twist = Twist()
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    def _publish_search_cmd(self, angular_z: float) -> None:
        """Publish a search command and mark the search path as active."""
        self.search_motion_active = True
        self._publish_cmd(angular_z)

    def _stop_search_motion(self) -> None:
        """Publish one stop command when relinquishing the search command path."""
        if not self.search_motion_active:
            return
        self.search_motion_active = False
        self._publish_cmd(0.0)

    def _reset_after_clock_jump(self, now_ns: int) -> None:
        """Clear transient state after ROS time starts late or jumps backwards."""
        self._stop_search_motion()
        self.search_started_ns = now_ns
        self.accumulated_search_yaw = 0.0
        self.candidate_entered_ns = None
        self.ready_since_ns = None
        self.latest_seed_pose = None
        self.last_odom_yaw = None
        self.last_timer_now_ns = now_ns
        self.lock_tracker.clear()
        if self.pending_set_pose_future is not None and (
            not self.pending_set_pose_future.done()
        ):
            self.discarded_set_pose_future = self.pending_set_pose_future
            self.discarded_set_pose_started_ns = now_ns
        else:
            self.discarded_set_pose_future = None
            self.discarded_set_pose_started_ns = None
        self.pending_set_pose_future = None
        self.set_pose_attempt_started_ns = None
        self.last_set_pose_service_check_ns = None
        self._set_state(
            STATE_SEARCHING,
            REASON_SEARCHING,
            "Searching for the start-zone AprilTag.",
        )

    def _set_state(self, state: str, reason_code: str, reason_detail: str) -> None:
        """Update the public state and emit a concise log when it changes."""
        changed = (
            state != self.state
            or reason_code != self.reason_code
            or reason_detail != self.reason_detail
        )
        self.state = state
        self.reason_code = reason_code
        self.reason_detail = reason_detail
        if changed:
            self.get_logger().info(f"[{state}] {reason_code}: {reason_detail}")

    def _publish_status(self, now_ns: int) -> None:
        """Publish the current start-zone localisation status."""
        status = LocalisationStartZoneStatus()
        if now_ns > 0:
            status.stamp.sec = now_ns // 1_000_000_000
            status.stamp.nanosec = now_ns % 1_000_000_000
        status.state = self.state
        status.ready = self.state == STATE_READY
        status.reason_code = self.reason_code
        status.reason_detail = self.reason_detail
        if self.ready_since_ns is None:
            status.stable_lock_age_s = 0.0
        else:
            status.stable_lock_age_s = max(
                0.0, float(now_ns - self.ready_since_ns) / 1e9
            )
        if self.search_started_ns is None:
            status.search_elapsed_s = 0.0
        else:
            status.search_elapsed_s = max(
                0.0, float(now_ns - self.search_started_ns) / 1e9
            )
        self.status_pub.publish(status)

    def _should_fail_timeout(self, now_ns: int) -> bool:
        """Return whether the bounded search window has been exhausted."""
        if self.search_started_ns is None:
            return False
        elapsed_s = float(now_ns - self.search_started_ns) / 1e9
        return (
            elapsed_s >= self.max_search_duration_s
            or self.accumulated_search_yaw >= self.max_search_yaw_rad
        )

    def _maybe_start_set_pose(self, now_ns: int) -> None:
        """Seed the global EKF once a stable lock has been achieved."""
        if self.latest_seed_pose is None:
            self._set_state(
                STATE_FAILED,
                REASON_SET_POSE_FAILED,
                "Stable lock was reported without an accepted tag pose.",
            )
            self._stop_search_motion()
            return

        if self.set_pose_attempt_started_ns is None:
            self.set_pose_attempt_started_ns = now_ns

        if (
            self.last_set_pose_service_check_ns is not None
            and self.set_pose_service_wait_s > 0.0
            and float(now_ns - self.last_set_pose_service_check_ns) / 1e9
            < self.set_pose_service_wait_s
        ):
            self._set_state(
                STATE_CANDIDATE_LOCK,
                REASON_UNSTABLE,
                f"Stable tag lock acquired; waiting for {self.set_pose_service}.",
            )
            return

        self.last_set_pose_service_check_ns = now_ns
        service_is_ready = False
        if hasattr(self.set_pose_client, "service_is_ready"):
            service_is_ready = self.set_pose_client.service_is_ready()
        else:  # pragma: no cover - compatibility fallback
            service_is_ready = self.set_pose_client.wait_for_service(timeout_sec=0.0)

        if not service_is_ready:
            if (
                float(now_ns - self.set_pose_attempt_started_ns) / 1e9
                >= self.set_pose_timeout_s
            ):
                self._set_state(
                    STATE_FAILED,
                    REASON_SET_POSE_FAILED,
                    f"Global EKF {self.set_pose_service} service never became available.",
                )
                self._stop_search_motion()
            else:
                self._set_state(
                    STATE_CANDIDATE_LOCK,
                    REASON_UNSTABLE,
                    f"Stable tag lock acquired; waiting for {self.set_pose_service}.",
                )
            return

        request = SetPose.Request()
        request.pose = self.latest_seed_pose
        self.pending_set_pose_future = self.set_pose_client.call_async(request)
        self.last_set_pose_service_check_ns = None
        self._stop_search_motion()
        self._set_state(
            STATE_CANDIDATE_LOCK,
            REASON_UNSTABLE,
            "Stable tag lock acquired; seeding the global EKF.",
        )

    def _handle_pending_set_pose(self, now_ns: int) -> bool:
        """Advance or finish an in-flight /set_pose request."""
        if self.pending_set_pose_future is None:
            return False
        if not self.pending_set_pose_future.done():
            if (
                self.set_pose_attempt_started_ns is not None
                and float(now_ns - self.set_pose_attempt_started_ns) / 1e9
                >= self.set_pose_timeout_s
            ):
                self.pending_set_pose_future = None
                self._set_state(
                    STATE_FAILED,
                    REASON_SET_POSE_FAILED,
                    "Global EKF /set_pose timed out.",
                )
                self._stop_search_motion()
            return True

        try:
            self.pending_set_pose_future.result()
        except Exception as exc:  # pragma: no cover - defensive path
            self._set_state(
                STATE_FAILED,
                REASON_SET_POSE_FAILED,
                f"Global EKF /set_pose failed: {exc}",
            )
            self.pending_set_pose_future = None
            self.last_set_pose_service_check_ns = None
            self._stop_search_motion()
            return True

        self.pending_set_pose_future = None
        self.set_pose_attempt_started_ns = None
        self.last_set_pose_service_check_ns = None
        self.ready_since_ns = now_ns
        self._set_state(
            STATE_READY,
            REASON_READY,
            "Start-zone localisation is stable and travel may begin.",
        )
        self._stop_search_motion()
        return True

    def _handle_discarded_set_pose(self, now_ns: int) -> bool:
        """Wait for a stale pre-rewind /set_pose request to clear."""
        if self.discarded_set_pose_future is None:
            return False

        if not self.discarded_set_pose_future.done():
            if (
                self.discarded_set_pose_started_ns is not None
                and float(now_ns - self.discarded_set_pose_started_ns) / 1e9
                >= self.set_pose_timeout_s
            ):
                self.discarded_set_pose_future = None
                self.discarded_set_pose_started_ns = None
                self._set_state(
                    STATE_FAILED,
                    REASON_SET_POSE_FAILED,
                    "An earlier EKF seed request did not settle after a clock reset.",
                )
                self._stop_search_motion()
                return True

            self._set_state(
                STATE_SEARCHING,
                REASON_SEARCHING,
                "Waiting for an earlier EKF seed request to settle after a clock reset.",
            )
            self._stop_search_motion()
            return True

        try:
            self.discarded_set_pose_future.result()
        except Exception:  # pragma: no cover - defensive path
            pass

        self.discarded_set_pose_future = None
        self.discarded_set_pose_started_ns = None
        return True

    def _on_timer(self) -> None:
        """Run the bounded acquisition state machine and publish status."""
        now_ns = self.get_clock().now().nanoseconds

        if now_ns <= 0:
            self._stop_search_motion()
            self.ready_since_ns = None
            self._set_state(
                STATE_SEARCHING,
                REASON_SEARCHING,
                "Searching for the start-zone AprilTag.",
            )
            self.search_started_ns = None
            self.last_timer_now_ns = None
            self._publish_status(now_ns)
            return

        if self.last_timer_now_ns is not None and now_ns < self.last_timer_now_ns:
            self._reset_after_clock_jump(now_ns)
        elif self.search_started_ns is None:
            self._reset_after_clock_jump(now_ns)

        self.last_timer_now_ns = now_ns

        if self.state in {STATE_READY, STATE_FAILED}:
            self._stop_search_motion()
            self._publish_status(now_ns)
            return

        if self._handle_discarded_set_pose(now_ns):
            self._publish_status(now_ns)
            return

        if self._handle_pending_set_pose(now_ns):
            self._publish_status(now_ns)
            return

        if not self._required_tf_available():
            self.set_pose_attempt_started_ns = None
            if self._should_fail_timeout(now_ns) or (
                float(now_ns - self.search_started_ns) / 1e9
            ) >= self.tf_missing_timeout_s:
                self._set_state(
                    STATE_FAILED,
                    REASON_TF_MISSING,
                    "Required TF for start-zone localisation never became available.",
                )
            else:
                self._set_state(
                    STATE_SEARCHING,
                    REASON_TF_MISSING,
                    "Waiting for the map/tag or camera/base TF chain.",
                )
            self._stop_search_motion()
            self._publish_status(now_ns)
            return

        self.lock_tracker.prune(now_ns - self.stable_window_ns)
        self.lock_tracker.prune_future(now_ns)
        latest_sample = self.lock_tracker.latest()
        has_fresh_sample = self.lock_tracker.is_fresh(now_ns, self.max_sample_gap_ns)

        if has_fresh_sample:
            if self.state != STATE_CANDIDATE_LOCK:
                self.candidate_entered_ns = now_ns
            self._set_state(
                STATE_CANDIDATE_LOCK,
                REASON_UNSTABLE,
                "Tag detected; waiting for a stable lock window.",
            )
            self._publish_search_cmd(
                self.search_sign * self.candidate_alignment_angular_speed_rad_s
            )

            candidate_elapsed_ns = 0
            if self.candidate_entered_ns is not None:
                candidate_elapsed_ns = now_ns - self.candidate_entered_ns

            if (
                candidate_elapsed_ns >= int(self.candidate_settle_duration_s * 1e9)
                and self.lock_tracker.has_stable_lock(
                    now_ns=now_ns,
                    window_ns=self.stable_window_ns,
                    min_samples=self.stable_window_min_samples,
                    max_gap_ns=self.max_sample_gap_ns,
                    max_translation_spread_m=self.max_lock_translation_spread_m,
                    max_yaw_spread_rad=self.max_lock_yaw_spread_rad,
                )
            ):
                self._maybe_start_set_pose(now_ns)
        else:
            self.candidate_entered_ns = None
            self.set_pose_attempt_started_ns = None
            if latest_sample is not None:
                self._set_state(
                    STATE_SEARCHING,
                    REASON_STALE,
                    "Tag lock was lost before it became stable.",
                )
            else:
                self._set_state(
                    STATE_SEARCHING,
                    REASON_SEARCHING,
                    "Searching for the start-zone AprilTag.",
                )

            if self._should_fail_timeout(now_ns):
                self._set_state(
                    STATE_FAILED,
                    REASON_TIMEOUT,
                    "Timed out before a stable start-zone tag lock was achieved.",
                )
                self._stop_search_motion()
            else:
                self._publish_search_cmd(self.search_sign * self.angular_speed_rad_s)

        self._publish_status(now_ns)


def main(args=None) -> None:
    """Run the start-zone localiser node."""
    rclpy.init(args=args)
    node = StartZoneLocaliser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # pragma: no cover - shutdown can already be in progress
            pass
