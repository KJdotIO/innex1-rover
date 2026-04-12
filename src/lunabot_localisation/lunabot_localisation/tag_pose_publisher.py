"""Bridge AprilTag TF detections into PoseWithCovarianceStamped updates."""

import math

import rclpy
import tf2_geometry_msgs
import tf2_ros
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class TagPosePublisher(Node):
    """Publish a map-frame pose estimate derived from the detected AprilTag."""

    def __init__(self):
        super().__init__("tag_pose_publisher")

        self.declare_parameter("tag_frame", "tag36h11:0")
        self.declare_parameter("detected_tag_frame", "tag36h11:0_detection")
        self.declare_parameter("camera_frame", "camera_front_optical_frame")
        self.declare_parameter("target_frame", "base_footprint")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("tag_id", 0)
        self.declare_parameter("detections_topic", "/camera_front/tags")
        self.declare_parameter("max_detection_age_sec", 0.35)
        self.declare_parameter("max_tag_distance_m", 3.0)
        self.declare_parameter("max_detection_sync_slop_sec", 0.05)
        self.declare_parameter("max_hamming", 0)
        self.declare_parameter("min_decision_margin", 35.0)
        self.declare_parameter("correction_log_period_sec", 1.0)
        self.declare_parameter("correction_log_min_translation_delta_m", 0.20)
        self.declare_parameter("correction_log_min_yaw_delta_rad", 0.20)

        self.tag_frame = self.get_parameter("tag_frame").value
        self.detected_tag_frame = self.get_parameter("detected_tag_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.map_frame = self.get_parameter("map_frame").value
        self.tag_id = int(self.get_parameter("tag_id").value)
        self.detections_topic = self.get_parameter("detections_topic").value
        self.max_detection_age_sec = float(
            self.get_parameter("max_detection_age_sec").value
        )
        self.max_tag_distance_m = float(self.get_parameter("max_tag_distance_m").value)
        self.max_detection_sync_slop_sec = float(
            self.get_parameter("max_detection_sync_slop_sec").value
        )
        self.max_hamming = int(self.get_parameter("max_hamming").value)
        self.min_decision_margin = float(
            self.get_parameter("min_decision_margin").value
        )
        self.correction_log_period_sec = max(
            0.0, float(self.get_parameter("correction_log_period_sec").value)
        )
        self.correction_log_min_translation_delta_m = max(
            0.0,
            float(self.get_parameter("correction_log_min_translation_delta_m").value),
        )
        self.correction_log_min_yaw_delta_rad = max(
            0.0, float(self.get_parameter("correction_log_min_yaw_delta_rad").value)
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/tag_pose", 10)
        detection_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            self.detections_topic,
            self.on_detections,
            detection_qos,
        )
        self.last_processed_stamp_ns = None
        self.latest_detection_stamp_ns = None
        self.latest_detection_hamming = None
        self.latest_detection_margin = None
        self.last_report_time_ns = 0
        self.last_logged_pose = None

        self.timer = self.create_timer(0.1, self.on_timer)

    def _detection_metadata_ready(self) -> bool:
        """Return whether the current TF sample has matching detection metadata."""
        if self.latest_detection_stamp_ns is None:
            self.get_logger().debug("Dropping tag detection without matching metadata")
            return False
        if self.latest_detection_hamming is None or self.latest_detection_margin is None:
            self.get_logger().debug("Dropping tag detection with incomplete metadata")
            return False
        return True

    def _metadata_allows_publication(self, detection_stamp_ns: int) -> bool:
        """Return whether metadata and timing allow this tag sample to be used."""
        if not self._detection_metadata_ready():
            return False

        detection_metadata_stamp_ns = self.latest_detection_stamp_ns
        detection_hamming = self.latest_detection_hamming
        detection_margin = self.latest_detection_margin
        assert detection_metadata_stamp_ns is not None
        assert detection_hamming is not None
        assert detection_margin is not None

        sync_error_sec = abs(detection_stamp_ns - detection_metadata_stamp_ns) / 1e9
        if sync_error_sec > self.max_detection_sync_slop_sec:
            self.get_logger().debug(
                "Dropping unsynchronised tag detection "
                f"(tf_vs_msg={sync_error_sec:.3f}s)"
            )
            return False

        # Past this point the metadata and TF sample are aligned strongly enough
        # that retries are no longer useful.
        self.last_processed_stamp_ns = detection_stamp_ns

        if detection_hamming > self.max_hamming:
            self.get_logger().debug(
                "Dropping low-quality tag detection "
                f"(hamming={detection_hamming})"
            )
            return False

        if detection_margin < self.min_decision_margin:
            self.get_logger().debug(
                "Dropping low-confidence tag detection "
                f"(margin={detection_margin:.1f})"
            )
            return False

        return True

    def _age_allows_publication(self, detection_stamp_ns: int) -> bool:
        """Return whether the tag sample is recent enough to trust."""
        if detection_stamp_ns <= 0:
            return True

        now_ns = self.get_clock().now().nanoseconds
        age_sec = (now_ns - detection_stamp_ns) / 1e9
        if age_sec > self.max_detection_age_sec:
            self.get_logger().debug(
                f"Dropping stale tag detection (age={age_sec:.3f}s)"
            )
            return False
        if age_sec < -0.05:
            self.get_logger().debug(
                f"Dropping future-dated tag detection (age={age_sec:.3f}s)"
            )
            return False
        return True

    def _should_log_correction(
        self, pose_msg: PoseWithCovarianceStamped, yaw: float
    ) -> bool:
        """Return whether the current correction differs enough to report."""
        if self.last_logged_pose is None:
            return True

        dx_log = pose_msg.pose.pose.position.x - self.last_logged_pose[0]
        dy_log = pose_msg.pose.pose.position.y - self.last_logged_pose[1]
        translation_delta = math.sqrt(dx_log * dx_log + dy_log * dy_log)
        yaw_delta = abs(self._angle_delta(yaw, self.last_logged_pose[2]))
        return (
            translation_delta >= self.correction_log_min_translation_delta_m
            or yaw_delta >= self.correction_log_min_yaw_delta_rad
        )

    @staticmethod
    def _stamp_to_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

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

    @staticmethod
    def _angle_delta(a: float, b: float) -> float:
        """Return the wrapped angular difference between two yaw values."""
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def on_detections(self, msg: AprilTagDetectionArray) -> None:
        """Cache the latest AprilTag metadata for the configured tag id."""
        for detection in msg.detections:
            if detection.id != self.tag_id:
                continue

            self.latest_detection_stamp_ns = self._stamp_to_ns(msg.header.stamp)
            self.latest_detection_hamming = int(detection.hamming)
            self.latest_detection_margin = float(detection.decision_margin)
            return

    def on_timer(self):
        """Publish a tag-derived base pose when the required TF chain exists."""
        # Build an independent map->base estimate using:
        # map->tag (static), camera->tag (AprilTag), and camera->base (URDF static).
        try:
            map_to_tag = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.tag_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
            camera_to_detected_tag = self.tf_buffer.lookup_transform(
                self.detected_tag_frame,
                self.camera_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
            base_to_camera = self.tf_buffer.lookup_transform(
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
            return

        detection_stamp_ns = self._stamp_to_ns(camera_to_detected_tag.header.stamp)
        if detection_stamp_ns == self.last_processed_stamp_ns:
            return

        if not self._metadata_allows_publication(detection_stamp_ns):
            return

        if not self._age_allows_publication(detection_stamp_ns):
            return

        detected_tag_to_map = TransformStamped()
        detected_tag_to_map.header = map_to_tag.header
        detected_tag_to_map.child_frame_id = self.detected_tag_frame
        detected_tag_to_map.transform = map_to_tag.transform

        base_origin = Pose()
        base_origin.orientation.w = 1.0
        pose_in_camera = tf2_geometry_msgs.do_transform_pose(
            base_origin, base_to_camera
        )
        pose_in_detected_tag = tf2_geometry_msgs.do_transform_pose(
            pose_in_camera, camera_to_detected_tag
        )
        pose_in_map = tf2_geometry_msgs.do_transform_pose(
            pose_in_detected_tag, detected_tag_to_map
        )

        dx = camera_to_detected_tag.transform.translation.x
        dy = camera_to_detected_tag.transform.translation.y
        dz = camera_to_detected_tag.transform.translation.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if distance > self.max_tag_distance_m:
            self.get_logger().debug(
                f"Dropping far tag detection (distance={distance:.2f}m)"
            )
            return

        # Distance-based covariance: accuracy degrades with range
        cov_xy = max((distance * 0.1) ** 2, 0.01)
        cov_yaw = (distance * 0.05) ** 2 + 0.01

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = camera_to_detected_tag.header.stamp
        msg.header.frame_id = self.map_frame
        msg.pose.pose = pose_in_map

        # 6x6 covariance matrix (row-major, only diagonal populated)
        cov = [0.0] * 36
        cov[0] = cov_xy  # X
        cov[7] = cov_xy  # Y
        cov[14] = 1e6  # Z (ignored in 2D mode)
        cov[21] = 1e6  # Roll (ignored)
        cov[28] = 1e6  # Pitch (ignored)
        cov[35] = cov_yaw  # Yaw
        msg.pose.covariance = cov

        self.pub.publish(msg)
        now_ns = self.get_clock().now().nanoseconds
        yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)
        should_log = self._should_log_correction(msg, yaw)

        if (
            should_log
            and now_ns - self.last_report_time_ns
            >= int(self.correction_log_period_sec * 1e9)
        ):
            self.get_logger().info(
                "Accepted AprilTag correction "
                f"distance={distance:.2f}m "
                f"margin={self.latest_detection_margin:.1f} "
                f"map_pose=({msg.pose.pose.position.x:.2f}, "
                f"{msg.pose.pose.position.y:.2f}, yaw={yaw:.2f})"
            )
            self.last_report_time_ns = now_ns
            self.last_logged_pose = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                yaw,
            )
        self.get_logger().debug(
            f"Tag at {distance:.2f}m, cov_xy={cov_xy:.4f}, cov_yaw={cov_yaw:.4f}"
        )


def main(args=None):
    """Run the AprilTag pose bridge node."""
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = TagPosePublisher()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
