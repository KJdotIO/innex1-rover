import math
import rclpy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class TagPosePublisher(Node):
    def __init__(self):
        super().__init__("tag_pose_publisher")

        self.declare_parameter("tag_frame", "tag36h11:0")
        self.declare_parameter("detected_tag_frame", "tag36h11:0_detection")
        self.declare_parameter("camera_frame", "camera_front_optical_frame")
        self.declare_parameter("target_frame", "base_footprint")
        self.declare_parameter("map_frame", "map")

        self.tag_frame = self.get_parameter("tag_frame").value
        self.detected_tag_frame = self.get_parameter("detected_tag_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.map_frame = self.get_parameter("map_frame").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/tag_pose", 10)

        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        # Build an independent map->base estimate using:
        # map->tag (static), camera->tag (AprilTag), and camera->base (URDF static).
        try:
            map_to_tag = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.tag_frame,
                Time(),
                timeout=rclpy.duration.Duration(seconds=0.0),
            )
            camera_to_detected_tag = self.tf_buffer.lookup_transform(
                self.detected_tag_frame,
                self.camera_frame,
                Time(),
                timeout=rclpy.duration.Duration(seconds=0.0),
            )
            base_to_camera = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.target_frame,
                Time(),
                timeout=rclpy.duration.Duration(seconds=0.0),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
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
        self.get_logger().debug(
            f"Tag at {distance:.2f}m, cov_xy={cov_xy:.4f}, cov_yaw={cov_yaw:.4f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TagPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
