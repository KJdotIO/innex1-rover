"""Bridge AprilTag TF detections into PoseWithCovarianceStamped updates."""

import math
import rclpy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
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
        self.declare_parameter("max_detection_age_sec", 0.35)
        self.declare_parameter("max_tag_distance_m", 4.0)
        self.declare_parameter("correction_log_period_sec", 1.0)

        self.tag_frame = self.get_parameter("tag_frame").value
        self.detected_tag_frame = self.get_parameter("detected_tag_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.map_frame = self.get_parameter("map_frame").value
        self.max_detection_age_sec = float(
            self.get_parameter("max_detection_age_sec").value
        )
        self.max_tag_distance_m = float(self.get_parameter("max_tag_distance_m").value)
        self.correction_log_period_sec = float(
            self.get_parameter("correction_log_period_sec").value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/tag_pose", 10)
        self.last_detection_stamp_ns = None
        self.last_report_time_ns = 0

        self.timer = self.create_timer(0.1, self.on_timer)

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
        if detection_stamp_ns == self.last_detection_stamp_ns:
            return
        # Consume this stamp even when rejected to avoid reprocessing the same sample.
        self.last_detection_stamp_ns = detection_stamp_ns

        if detection_stamp_ns > 0:
            now_ns = self.get_clock().now().nanoseconds
            age_sec = (now_ns - detection_stamp_ns) / 1e9
            if age_sec > self.max_detection_age_sec:
                self.get_logger().debug(
                    f"Dropping stale tag detection (age={age_sec:.3f}s)"
                )
                return
            if age_sec < -0.05:
                self.get_logger().debug(
                    f"Dropping future-dated tag detection (age={age_sec:.3f}s)"
                )
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
        if now_ns - self.last_report_time_ns >= int(
            self.correction_log_period_sec * 1e9
        ):
            yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)
            self.get_logger().info(
                "Accepted AprilTag correction "
                f"distance={distance:.2f}m "
                f"map_pose=({msg.pose.pose.position.x:.2f}, "
                f"{msg.pose.pose.position.y:.2f}, yaw={yaw:.2f})"
            )
            self.last_report_time_ns = now_ns
        self.get_logger().debug(
            f"Tag at {distance:.2f}m, cov_xy={cov_xy:.4f}, cov_yaw={cov_yaw:.4f}"
        )


def main(args=None):
    """Run the AprilTag pose bridge node."""
    rclpy.init(args=args)
    node = TagPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
