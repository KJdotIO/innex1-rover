import math
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseWithCovarianceStamped


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _quat_normalise(q):
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    inv = 1.0 / norm
    return (x * inv, y * inv, z * inv, w * inv)


def _quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def _quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _rotate_vector(q, v):
    x, y, z, w = _quat_normalise(q)
    u = (x, y, z)
    uu = _dot(u, u)
    uv = _dot(u, v)
    uxv = _cross(u, v)
    return (
        2.0 * uv * u[0] + (w * w - uu) * v[0] + 2.0 * w * uxv[0],
        2.0 * uv * u[1] + (w * w - uu) * v[1] + 2.0 * w * uxv[1],
        2.0 * uv * u[2] + (w * w - uu) * v[2] + 2.0 * w * uxv[2],
    )


def _transform_components(transform):
    t = transform.transform.translation
    q = transform.transform.rotation
    return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)


def _invert_transform(t_ab, q_ab):
    q_ba = _quat_conjugate(_quat_normalise(q_ab))
    t_ba = _rotate_vector(q_ba, (-t_ab[0], -t_ab[1], -t_ab[2]))
    return t_ba, q_ba


def _compose_transform(t_ab, q_ab, t_bc, q_bc):
    q_ab_n = _quat_normalise(q_ab)
    q_bc_n = _quat_normalise(q_bc)
    t_bc_in_a = _rotate_vector(q_ab_n, t_bc)
    t_ac = (
        t_ab[0] + t_bc_in_a[0],
        t_ab[1] + t_bc_in_a[1],
        t_ab[2] + t_bc_in_a[2],
    )
    q_ac = _quat_multiply(q_ab_n, q_bc_n)
    return t_ac, _quat_normalise(q_ac)


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
            cam_to_tag = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.detected_tag_frame,
                Time(),
                timeout=rclpy.duration.Duration(seconds=0.0),
            )
            cam_to_base = self.tf_buffer.lookup_transform(
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

        map_to_tag_t, map_to_tag_q = _transform_components(map_to_tag)
        cam_to_tag_t, cam_to_tag_q = _transform_components(cam_to_tag)
        cam_to_base_t, cam_to_base_q = _transform_components(cam_to_base)

        tag_to_cam_t, tag_to_cam_q = _invert_transform(cam_to_tag_t, cam_to_tag_q)
        map_to_cam_t, map_to_cam_q = _compose_transform(
            map_to_tag_t, map_to_tag_q, tag_to_cam_t, tag_to_cam_q
        )
        map_to_base_t, map_to_base_q = _compose_transform(
            map_to_cam_t, map_to_cam_q, cam_to_base_t, cam_to_base_q
        )

        dx, dy, dz = cam_to_tag_t
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        # Distance-based covariance: accuracy degrades with range
        cov_xy = max((distance * 0.1) ** 2, 0.01)
        cov_yaw = (distance * 0.05) ** 2 + 0.01

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = cam_to_tag.header.stamp
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = map_to_base_t[0]
        msg.pose.pose.position.y = map_to_base_t[1]
        msg.pose.pose.position.z = map_to_base_t[2]
        msg.pose.pose.orientation.x = map_to_base_q[0]
        msg.pose.pose.orientation.y = map_to_base_q[1]
        msg.pose.pose.orientation.z = map_to_base_q[2]
        msg.pose.pose.orientation.w = map_to_base_q[3]

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
