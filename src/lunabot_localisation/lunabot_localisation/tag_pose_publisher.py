import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseWithCovarianceStamped


class TagPosePublisher(Node):

    def __init__(self):
        super().__init__("tag_pose_publisher")

        self.declare_parameter("tag_frame", "tag36h11:0")
        self.declare_parameter("camera_frame", "camera_front_optical_frame")
        self.declare_parameter("target_frame", "base_footprint")
        self.declare_parameter("map_frame", "map")

        self.tag_frame = self.get_parameter("tag_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.map_frame = self.get_parameter("map_frame").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, "/tag_pose", 10
        )

        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.target_frame, Time(), timeout=rclpy.duration.Duration(seconds=0.0)
            )
        except Exception:
            return

        # Distance from camera to tag for covariance scaling
        try:
            cam_to_tag = self.tf_buffer.lookup_transform(
                self.camera_frame, self.tag_frame, Time(), timeout=rclpy.duration.Duration(seconds=0.0)
            )
        except Exception:
            return

        dx = cam_to_tag.transform.translation.x
        dy = cam_to_tag.transform.translation.y
        dz = cam_to_tag.transform.translation.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        # Distance-based covariance: accuracy degrades with range
        cov_xy = (distance * 0.1) ** 2
        cov_yaw = (distance * 0.05) ** 2 + 0.01

        msg = PoseWithCovarianceStamped()
        msg.header = t.header
        msg.pose.pose.position.x = t.transform.translation.x
        msg.pose.pose.position.y = t.transform.translation.y
        msg.pose.pose.position.z = t.transform.translation.z
        msg.pose.pose.orientation = t.transform.rotation

        # 6x6 covariance matrix (row-major, only diagonal populated)
        cov = [0.0] * 36
        cov[0] = cov_xy   # X
        cov[7] = cov_xy   # Y
        cov[14] = 1e6     # Z (ignored in 2D mode)
        cov[21] = 1e6     # Roll (ignored)
        cov[28] = 1e6     # Pitch (ignored)
        cov[35] = cov_yaw # Yaw
        msg.pose.covariance = cov

        self.pub.publish(msg)
        self.get_logger().debug(
            f"Tag at {distance:.2f}m, cov_xy={cov_xy:.4f}, cov_yaw={cov_yaw:.4f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TagPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
